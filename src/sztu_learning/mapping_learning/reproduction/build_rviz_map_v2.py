"""Computer/offline: paired FAST-LIO frames -> conservative 3-D evidence -> 2-D map.

The map is a static-height-band candidate, not validated robot clearance. Rays
use the scan-reference LiDAR origin (an explicit approximation, not point times).
No previous-run poses/maps are loaded. PGM export flips Y exactly once.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parent


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def level_rotation(up, heading):
    up = np.asarray(up, float)
    up /= np.linalg.norm(up)
    x = np.array([0., 1., 0.])
    x -= up * (up @ x)
    x /= np.linalg.norm(x)
    basis = np.array([x, np.cross(up, x), up])
    return Rotation.from_euler('z', heading, degrees=True).as_matrix() @ basis


def fit_floor(points, cfg):
    lower, upper = cfg['floor_search_height_m']
    ground = points[(points[:, 2] >= lower) & (points[:, 2] <= upper)]
    if len(ground) < 2000:
        raise ValueError('地板候选点不足')
    _, indices = np.unique(np.floor(ground / .08).astype(np.int64), axis=0, return_index=True)
    ground = ground[indices]
    rng = np.random.default_rng(cfg['floor_seed'])
    if len(ground) > 30000:
        ground = ground[rng.choice(len(ground), 30000, replace=False)]
    best = 0
    for _ in range(400):
        a, b, c = ground[rng.choice(len(ground), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        if normal[2] < np.cos(np.deg2rad(10)):
            continue
        count = np.count_nonzero(np.abs((ground - a) @ normal) < cfg['floor_inlier_m'])
        if count > best:
            best, anchor, fitted_normal = count, a, normal
    if best < 1000:
        raise ValueError('没有找到有足够支持的近水平地板')
    for _ in range(3):
        inliers = ground[np.abs((ground - anchor) @ fitted_normal) < cfg['floor_inlier_m'] * 1.4]
        anchor = inliers.mean(axis=0)
        _, _, vh = np.linalg.svd(inliers - anchor, full_matrices=False)
        fitted_normal = vh[-1]
        if fitted_normal[2] < 0:
            fitted_normal = -fitted_normal
    if fitted_normal[2] < np.cos(np.deg2rad(10)):
        raise ValueError('地板倾角超出候选范围')
    # Preserve the chosen heading as closely as possible, then put the fitted
    # plane at Z=0. X/Y origin stays at the original origin's floor projection.
    x = np.array([1., 0., 0.])
    x -= fitted_normal * (x @ fitted_normal)
    x /= np.linalg.norm(x)
    rotation = np.array([x, np.cross(fitted_normal, x), fitted_normal])
    translation = np.array([0., 0., -(fitted_normal @ anchor)])
    return rotation, translation, {
        'normal_in_initial_level_frame': fitted_normal.tolist(),
        'point_on_plane_in_initial_level_frame_m': anchor.tolist(),
        'candidate_points_after_voxel_sample': len(ground), 'inliers': len(inliers),
        'inlier_absolute_error_p95_m': float(np.quantile(np.abs((inliers-anchor)@fitted_normal), .95)),
        'inlier_xy_extent_m': np.ptp(inliers[:, :2], axis=0).tolist(),
        'tilt_from_gravity_hint_degrees': float(np.rad2deg(np.arccos(fitted_normal[2]))),
        'method': 'Seeded near-horizontal RANSAC followed by SVD; global flat-floor candidate',
    }


def project_columns(hits, frees, cfg):
    """Fuse contradictory observations without permanently latching a noise hit.

    All vertical bins must still be observed. Low hit ratio alone cannot clear
    a voxel: at least 20 separate frames must also have traversed that voxel.
    These frames may be temporally correlated; this is not a probability model.
    Supported occupied voxels retain priority over the column's free evidence.
    """
    ratio = hits / np.maximum(hits.astype(float) + frees, 1)
    occupied_voxels = (hits >= cfg['minimum_hit_frames']) & (ratio >= cfg['occupied_hit_ratio'])
    uncontested = (frees >= cfg['minimum_free_frames']) & (hits == 0)
    weak_conflict = (frees >= cfg['conflict_minimum_free_frames']) & (ratio <= cfg['free_maximum_hit_ratio'])
    free_voxels = uncontested | weak_conflict
    grid = np.full(hits.shape[1:], -1, dtype=np.int8)
    grid[np.all(free_voxels, axis=0)] = 0
    grid[np.any(occupied_voxels, axis=0)] = 100
    return grid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'map_pipeline_v2.yaml')
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    source, output = ROOT/cfg['source'], ROOT/cfg['output']
    if output.exists():
        raise FileExistsError(f'拒绝覆盖已有结果：{output}')
    for key in ['resolution_xy_m', 'resolution_z_m', 'maximum_range_m', 'minimum_range_m']:
        if not np.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'参数无效：{key}')
    if cfg['maximum_range_m'] <= cfg['minimum_range_m']:
        raise ValueError('距离上下界错误')
    height_min, height_max = cfg['obstacle_height_m']
    if not 0 <= height_min < height_max:
        raise ValueError('高度带错误')
    nz_float = (height_max-height_min)/cfg['resolution_z_m']
    if not np.isclose(nz_float, round(nz_float)):
        raise ValueError('高度带必须恰好覆盖整数层体素')
    if not 0 <= cfg['free_maximum_hit_ratio'] < cfg['occupied_hit_ratio'] <= 1:
        raise ValueError('自由与占据证据之间必须保留未知区间')
    if cfg['conflict_minimum_free_frames'] < cfg['minimum_free_frames']:
        raise ValueError('冲突消解需要更充分穿过观测')
    reference = ROOT/cfg['reference_map']
    reference_manifest = json.loads((reference/'manifest.json').read_text())
    if reference_manifest['input_manifest_sha256'] != sha(source/'manifest.json'):
        raise ValueError('参考坐标与本轮输入不是同源档案')
    for name in ['transform.json', 'summary.json']:
        if sha(reference/name) != reference_manifest['files_sha256'][name]:
            raise ValueError('参考坐标内容变化')
    output.mkdir()
    evidence = output/'evidence'
    evidence.mkdir()
    with (source/'frames.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    manifest = json.loads((source/'manifest.json').read_text())
    # Check actual archive identity before accepting it as same-run input.
    for name, digest in manifest['output_sha256'].items():
        if sha(source/name) != digest:
            raise ValueError(f'来源内容改变：{name}')
    positions, quats = [], []
    for i, row in enumerate(rows):
        with np.load(source/row['file'], allow_pickle=False) as data:
            positions.append(data['position'])
            quats.append(data['quaternion_xyzw'])
    # Freeze the baseline geometry: changed coverage must not move walls or
    # waypoints merely by fitting a different floor or shifting grid bounds.
    transform = json.loads((reference/'transform.json').read_text())
    rotation = np.array(transform['rotation_target_from_source'])
    translation = np.array(transform['translation_m'])
    floor_info = transform['floor']
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-10) or not np.isclose(np.linalg.det(rotation), 1):
        raise ValueError('坐标旋转无效')
    positions, quats = np.array(positions), np.array(quats)
    body_rotations = Rotation.from_quat(quats)
    extrinsic = yaml.safe_load((ROOT/'fastlio_extrinsics_candidate.yaml').read_text())['laser_mapping']['ros__parameters']['mapping']
    offset = np.array(extrinsic['extrinsic_T'])
    origins_source = positions + body_rotations.apply(np.tile(offset, (len(positions), 1)))
    origins = origins_source @ rotation.T + translation
    map_positions = positions @ rotation.T + translation
    map_rotations = Rotation.from_matrix(rotation) * body_rotations
    times = np.array([int(row['sec'])-int(rows[0]['sec']) + (int(row['nanosec'])-int(rows[0]['nanosec']))*1e-9 for row in rows])
    chosen = [0]
    for i in range(1, len(rows)):
        last = chosen[-1]
        moved = np.linalg.norm(origins[i]-origins[last]) >= cfg['keyframe_translation_m']
        turned = (body_rotations[last].inv()*body_rotations[i]).magnitude() >= np.deg2rad(cfg['keyframe_rotation_degrees'])
        waited = times[i]-times[last] >= cfg['keyframe_maximum_interval_s']
        if moved or turned or waited or i == len(rows)-1:
            chosen.append(i)
    if cfg['use_all_frames']:
        chosen = list(range(len(rows)))
    # Bounds include all accepted ray endpoints and selected origins. Endpoints
    # outside the collision height band still contribute clipped 3-D rays.
    lo = origins[chosen].min(axis=0)
    hi = origins[chosen].max(axis=0)
    prepared = []
    for i in chosen:
        with np.load(source/rows[i]['file'], allow_pickle=False) as data:
            points = data['xyz'].astype(float)
        distance = np.linalg.norm(points-origins_source[i], axis=1)
        points = points[(distance >= cfg['minimum_range_m']) & (distance <= cfg['maximum_range_m'])]
        points = points @ rotation.T + translation
        # One retained return per endpoint voxel; choose the nearest measured
        # endpoint in that voxel. This reduces duplicate work without averaging
        # a return into an unmeasured location.
        res = np.array([cfg['resolution_xy_m']]*2+[cfg['resolution_z_m']])
        order = np.argsort(np.linalg.norm(points-origins[i], axis=1), kind='stable')
        points = points[order]
        _, keep = np.unique(np.floor(points/res).astype(np.int64), axis=0, return_index=True)
        points = points[np.sort(keep)].astype('<f4')
        lo = np.minimum(lo, points.min(axis=0))
        hi = np.maximum(hi, points.max(axis=0))
        prepared.append(points)
    xyres, zres = cfg['resolution_xy_m'], cfg['resolution_z_m']
    lower = np.array([*(np.floor((lo[:2]-.3)/xyres)*xyres), height_min])
    dims = np.array([*(np.ceil((hi[:2]+.3-lower[:2])/xyres).astype(int)), round(nz_float)], dtype='<u4')
    reference_summary = json.loads((reference/'summary.json').read_text())
    if reference_summary['frame_id'] != cfg['frame_id'] or reference_summary['resolution_m'] != xyres or reference_summary['height_band_m'] != cfg['obstacle_height_m']:
        raise ValueError('比较坐标或栅格规格不一致')
    lower = np.array([*reference_summary['origin_xy_m'], height_min])
    dims = np.array([reference_summary['width'], reference_summary['height'], round(nz_float)], dtype='<u4')
    resolutions = np.array([xyres, xyres, zres], dtype='<f8')
    binary = ROOT/'runtime_logs/lesson58/map_voxel_rays'
    binary.parent.mkdir(exist_ok=True)
    subprocess.run(['g++','-O3','-std=c++17','-Wall','-Wextra',str(ROOT/'map_voxel_rays.cpp'),'-o',str(binary)],check=True)
    ray_input = evidence/'rays.bin'
    with ray_input.open('xb') as stream:
        stream.write(b'SZVOX001')
        stream.write(dims.tobytes());stream.write(lower.astype('<f8').tobytes())
        stream.write(resolutions.tobytes());stream.write(struct.pack('<I',len(chosen)))
        for i, points in zip(chosen, prepared):
            stream.write(origins[i].astype('<f8').tobytes())
            stream.write(struct.pack('<I',len(points)));stream.write(points.tobytes())
    completed = subprocess.run([str(binary),str(ray_input),str(evidence/'voxel_counts.bin')],check=True,text=True,capture_output=True)
    (evidence/'raycast.log').write_text(completed.stderr+completed.stdout)
    counts = np.fromfile(evidence/'voxel_counts.bin',dtype='<u2').reshape(2,int(dims[2]),int(dims[1]),int(dims[0]))
    grid = project_columns(counts[0],counts[1],cfg)
    np.save(output/'occupancy.npy', grid, allow_pickle=False)
    # NumPy/OccupancyGrid row zero = minimum Y; PGM row zero = maximum Y.
    pixels = np.full(grid.shape,205,dtype=np.uint8)
    pixels[grid==0]=254;pixels[grid==100]=0
    Image.fromarray(np.flipud(pixels)).save(output/'map.pgm')
    map_yaml = {'image':'map.pgm','mode':'trinary','resolution':xyres,'origin':[float(lower[0]),float(lower[1]),0.0],'negate':0,'occupied_thresh':.65,'free_thresh':.196}
    (output/'map.yaml').write_text(yaml.safe_dump(map_yaml,sort_keys=False))
    with (output/'trajectory.csv').open('x',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['sec','nanosec','x','y','z','qx','qy','qz','qw','lidar_x','lidar_y','lidar_z'])
        for row,p,q,o in zip(rows,map_positions,map_rotations.as_quat(),origins):
            writer.writerow([row['sec'],row['nanosec'],*p,*q,*o])
    write_json(output/'transform.json',{'source_frame':'camera_init','target_frame':cfg['frame_id'],'rotation_target_from_source':rotation.tolist(),'translation_m':translation.tolist(),'formula':'p_target = R @ p_source + t; q_target = q_R * q_source','lidar_origin_formula':'p_lidar_source = p_body_source + R_body_source @ extrinsic_T','floor':floor_info,'extrinsic_T_m':offset.tolist(),'up_hint_source':cfg['up_hint_record'],'up_hint_source_sha256':sha(ROOT/cfg['up_hint_record'])})
    write_json(output/'keyframes.json',[{'index':i,'sec':int(rows[i]['sec']),'nanosec':int(rows[i]['nanosec']),'rays':len(p)} for i,p in zip(chosen,prepared)])
    summary={'input_frames':len(rows),'keyframes':len(chosen),'rays':sum(map(len,prepared)),'width':int(dims[0]),'height':int(dims[1]),'vertical_bins':int(dims[2]),'resolution_m':xyres,'origin_xy_m':lower[:2].tolist(),'frame_id':cfg['frame_id'],'occupied_cells':int(np.sum(grid==100)),'free_cells':int(np.sum(grid==0)),'unknown_cells':int(np.sum(grid==-1)),'height_band_m':[height_min,height_max],'projection':'occupied if any supported voxel; free only if ALL height bins are uncontested free or have >=20 traversal frames and <=0.10 hit fraction; conflicting or unobserved bins stay unknown','ray_origin_approximation':'one LiDAR origin at each scan reference time; registered/undistorted endpoints; not exact instantaneous per-beam origins','floor':floor_info,'limits':['Static-map candidate; not navigation or collision-clearance acceptance','Uncalibrated candidate external parameters and point-time interpretation inherited from FAST-LIO run','Global flat-floor model; no slope/step/drop-off handling','Below/above configured height band not covered','No interpolation, hole filling or trajectory carving; only evidence-supported unknown-to-free updates','No real robot command or localization deployment']}
    write_json(output/'summary.json',summary)
    extent=[lower[0],lower[0]+dims[0]*xyres,lower[1],lower[1]+dims[1]*xyres]
    fig,ax=plt.subplots(figsize=(9,9),layout='constrained')
    display=np.where(grid==-1,1,np.where(grid==0,2,0))
    ax.imshow(display,origin='lower',extent=extent,cmap=ListedColormap(['black','#cdcdcd','#fefefe']),vmin=0,vmax=2,interpolation='nearest')
    ax.plot(map_positions[:,0],map_positions[:,1],color='#df4f00',lw=1,label='Same-run estimated body trajectory')
    ax.scatter(*map_positions[0,:2],c='limegreen',s=45,label='Start')
    ax.scatter(*map_positions[-1,:2],c='royalblue',s=30,label='End')
    ax.set(xlabel='X (m)',ylabel='Y (m)',title='Static map candidate: black occupied / white free / grey unknown')
    ax.legend(fontsize=8);ax.set_aspect('equal');fig.savefig(output/'map_preview.png',dpi=160);plt.close(fig)
    (output/'effective_config.yaml').write_text(yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False))
    files=[p for p in output.rglob('*') if p.is_file()]
    source_files=['build_rviz_map_v2.py','map_voxel_rays.cpp',str(args.config.resolve().relative_to(ROOT.resolve())),'fastlio_extrinsics_candidate.yaml']
    write_json(output/'manifest.json',{'input_run':str(source.relative_to(ROOT)),'reference_map':cfg['reference_map'],'reference_manifest_sha256':sha(reference/'manifest.json'),'input_manifest_sha256':sha(source/'manifest.json'),'input_index_sha256':sha(source/'frames.csv'),'source_sha256':{name:sha(ROOT/name) for name in source_files},'binary_sha256':sha(binary),'invocation':sys.argv,'files_sha256':{str(p.relative_to(output)):sha(p) for p in files}})
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
