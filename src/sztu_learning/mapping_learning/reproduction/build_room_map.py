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


from build_rviz_map import sha, write_json, level_rotation, fit_floor
from build_rviz_map_v2 import project_columns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'room_map_config.yaml')
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
        raise ValueError('自由与占据证据需要保留未知区间')
    cfg['up_hint'] = json.loads((ROOT/cfg['up_hint_record']).read_text())['up_hint']
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
    base = level_rotation(cfg['up_hint'], cfg['turn_heading_degrees'])
    positions, quats, samples = [], [], []
    for i, row in enumerate(rows):
        with np.load(source/row['file'], allow_pickle=False) as data:
            positions.append(data['position'])
            quats.append(data['quaternion_xyzw'])
            if i % 5 == 0:
                xyz = data['xyz'][::3].astype(float)
                ranges = np.linalg.norm(xyz-data['position'], axis=1)
                xyz = xyz[ranges <= cfg['maximum_range_m']]
                samples.append(xyz @ base.T)
    floor_r, translation, floor_info = fit_floor(np.concatenate(samples), cfg)
    rotation = floor_r @ base
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
    resolutions = np.array([xyres, xyres, zres], dtype='<f8')
    binary = ROOT/'runtime_logs/room_mapping_20261120/map_voxel_rays'
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
    summary={'input_frames':len(rows),'keyframes':len(chosen),'rays':sum(map(len,prepared)),'width':int(dims[0]),'height':int(dims[1]),'vertical_bins':int(dims[2]),'resolution_m':xyres,'origin_xy_m':lower[:2].tolist(),'frame_id':cfg['frame_id'],'occupied_cells':int(np.sum(grid==100)),'free_cells':int(np.sum(grid==0)),'unknown_cells':int(np.sum(grid==-1)),'height_band_m':[height_min,height_max],'projection':'occupied if any supported voxel; free only if all height bins have uncontested traversal or >=20 traversal frames and <=0.10 hit ratio; unresolved conflicts stay unknown','ray_origin_approximation':'one LiDAR origin at each scan reference time; registered/undistorted endpoints; not exact instantaneous per-beam origins','floor':floor_info,'limits':['Static-map candidate; not navigation or collision-clearance acceptance','Uncalibrated candidate external parameters and point-time interpretation inherited from FAST-LIO run','Global flat-floor model; no slope/step/drop-off handling','Below/above configured height band not covered','No interpolation, hole filling or trajectory carving; evidence-based classification only','No real robot command or localization deployment']}
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
    source_files=['build_room_map.py','build_rviz_map.py','build_rviz_map_v2.py','map_voxel_rays.cpp',str(args.config.resolve().relative_to(ROOT.resolve())),'fastlio_extrinsics_candidate.yaml']
    write_json(output/'manifest.json',{'input_run':str(source.relative_to(ROOT)),'input_manifest_sha256':sha(source/'manifest.json'),'input_index_sha256':sha(source/'frames.csv'),'source_sha256':{name:sha(ROOT/name) for name in source_files},'binary_sha256':sha(binary),'invocation':sys.argv,'files_sha256':{str(p.relative_to(output)):sha(p) for p in files}})
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
