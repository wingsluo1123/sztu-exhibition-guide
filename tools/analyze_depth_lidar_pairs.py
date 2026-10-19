#!/usr/bin/env python3
"""Offline paired-depth diagnostics; no ROS or hardware control.

Inputs are undistorted, zero-skew NPZ captures with depth, K, camera_rotation/translation,
lidar_points, normal/offset and depth/cloud stamp_ns. Coordinates are the same
leveled pelvis reference. A depth scale is an explicit hypothesis, not inferred
from image encoding. Frustum membership is only a necessary visibility condition.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def intrinsics(matrix):
    """Accept the undistorted, zero-skew pinhole model used by these captures."""
    matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
    if (not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1], atol=1e-12, rtol=0)
            or matrix[0, 1] != 0 or matrix[1, 0] != 0):
        raise ValueError('finite zero-skew pinhole intrinsics required')
    return matrix


def depth_points(depth, camera_intrinsics, scale, stride=4):
    """Back-project positive depth using the supplied axial-depth hypothesis."""
    depth = np.asarray(depth)
    camera_intrinsics = intrinsics(camera_intrinsics)
    if (depth.ndim != 2 or not math.isfinite(scale) or scale <= 0
            or not isinstance(stride, int) or stride < 1):
        raise ValueError('finite pinhole intrinsics, positive scale/stride and 2-D depth required')
    row, col = np.mgrid[:depth.shape[0]:stride, :depth.shape[1]:stride]
    axial = depth[::stride, ::stride] * scale
    valid = np.isfinite(axial) & (axial > 0)
    return np.stack(((col-camera_intrinsics[0, 2])/camera_intrinsics[0, 0]*axial,
                     (row-camera_intrinsics[1, 2])/camera_intrinsics[1, 1]*axial,
                     axial), axis=-1)[valid]


def project(points, camera_intrinsics, rotation, translation, shape):
    """Project reference-frame points into the supplied camera frame."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    camera_intrinsics = intrinsics(camera_intrinsics)
    if (len(shape) != 2 or any(not isinstance(size, (int, np.integer)) or size <= 0
                               for size in shape)):
        raise ValueError('positive 2-D image shape required')
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)
    if (rotation.shape != (3, 3) or translation.shape != (3,)
            or not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(float(np.linalg.det(rotation)), 1, abs_tol=1e-6)):
        raise ValueError('a proper rigid camera-to-reference transform is required')
    camera = (points-translation) @ rotation
    pixels = np.full((len(points), 2), np.nan)
    positive = np.all(np.isfinite(camera), axis=1) & (camera[:, 2] > 0)
    pixels[positive, 0] = (camera_intrinsics[0, 0]*camera[positive, 0]
                           / camera[positive, 2]+camera_intrinsics[0, 2])
    pixels[positive, 1] = (camera_intrinsics[1, 1]*camera[positive, 1]
                           / camera[positive, 2]+camera_intrinsics[1, 2])
    inside = (positive & (pixels[:, 0] >= 0) & (pixels[:, 0] < shape[1])
              & (pixels[:, 1] >= 0) & (pixels[:, 1] < shape[0]))
    return pixels, camera[:, 2], inside


def percentiles(values):
    return np.percentile(values, [5, 50, 90, 95]).tolist() if len(values) else None


def swept_grid(radius, advance, spacing):
    """Sample newly swept XY outside the initial circular footprint."""
    if not all(math.isfinite(value) and value > 0 for value in (radius, advance, spacing)):
        raise ValueError('positive finite circle, advance and spacing required')
    forward, lateral = np.meshgrid(np.arange(0, advance+radius+spacing/2, spacing),
                                   np.arange(-radius, radius+spacing/2, spacing))
    mask = ((np.hypot(forward-np.clip(forward, 0, advance), lateral) <= radius)
            & (np.hypot(forward, lateral) > radius))
    return np.column_stack((forward[mask], lateral[mask]))


def analyze_pair(path, scale, radius, advance, spacing):
    with np.load(path) as capture:
        depth = capture['depth']
        camera_intrinsics = capture['k'].reshape(3, 3)
        rotation, translation = capture['camera_rotation'], capture['camera_translation']
        normal, offset = capture['normal'], float(capture['offset'])
        if (normal.shape != (3,) or not np.all(np.isfinite(normal))
                or not math.isfinite(offset)
                or not np.isclose(np.linalg.norm(normal), 1) or normal[2] <= 0):
            raise ValueError('unit upward reference plane required')
        camera = depth_points(depth, camera_intrinsics, scale)
        points = camera @ rotation.T + translation
        height = points @ normal-offset
        control_height = (camera*[-1, -1, 1]) @ rotation.T @ normal+translation @ normal-offset
        pixels, axial, inside = project(capture['lidar_points'], camera_intrinsics,
                                        rotation, translation, depth.shape)
        pixels = np.floor(pixels[inside]+.5).astype(int)
        axial = axial[inside]
        valid_pixel = ((pixels[:, 0] < depth.shape[1]) & (pixels[:, 1] < depth.shape[0]))
        pixels, axial = pixels[valid_pixel], axial[valid_pixel]
        observed = depth[pixels[:, 1], pixels[:, 0]] * scale
        valid = np.isfinite(observed) & (observed > 0)
        residual = observed[valid]-axial[valid]
        xy = swept_grid(radius, advance, spacing)
        visibility = []
        masks = []
        for target_height in [.05, .20, .60, 1.0]:
            target_z = (offset+target_height-xy @ normal[:2])/normal[2]
            _, _, inside = project(np.column_stack((xy, target_z)), camera_intrinsics,
                                    rotation, translation, depth.shape)
            masks.append(inside)
            visibility.append(dict(height_above_plane=target_height, total=len(xy),
                                   within_frustum=int(inside.sum()), outside_frustum=int((~inside).sum()),
                                   fraction_within_frustum=float(inside.mean())))
        report = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      stamp_difference_s=abs(int(capture['depth_stamp_ns'])-int(capture['cloud_stamp_ns']))/1e9,
                      sampled_depth_points=len(points), positive_depth_height_percentiles=percentiles(height),
                      fraction_below_lidar_plane_5cm=float(np.mean(height < -.05)) if len(height) else None,
                      flipped_xy_control_fraction_below_plane_5cm=float(np.mean(control_height < -.05)) if len(height) else None,
                      projected_lidar_positive_depth_samples=len(residual),
                      axial_difference_signed_percentiles=percentiles(residual),
                      axial_difference_absolute_percentiles=percentiles(np.abs(residual)),
                      axial_depth_ratio_percentiles=percentiles(observed[valid]/axial[valid]),
                      depth_frustum_only=visibility)
        return report, (xy, masks, points, height, capture['lidar_points'].copy(), normal, offset)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pairs', type=Path, nargs='+')
    parser.add_argument('--depth-scale', type=float, required=True)
    parser.add_argument('--radius', type=float, default=.55)
    parser.add_argument('--advance', type=float, default=.85,
                        help='total diagnostic travel including stopping allowance, metres')
    parser.add_argument('--spacing', type=float, default=.025)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    if args.output.exists() or (args.plot and args.plot.exists()):
        parser.error('preserve prior output; choose a new path')
    results = [analyze_pair(path, args.depth_scale, args.radius, args.advance, args.spacing)
               for path in args.pairs]
    report = dict(scope='Offline diagnostic only. Explicit axial depth scale hypothesis; project using recorded TF without axis corrections. Same-pose repeats are not independent calibration. No visibility certification, occlusion/range/reflectivity validation, motion release or hardware commands.',
                  analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  depth_scale=args.depth_scale, model=dict(radius=args.radius, total_advance=args.advance, spacing=args.spacing),
                  percentile_order=[5, 50, 90, 95], pairs=[result[0] for result in results])
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        xy, masks, points, height, lidar, normal, offset = results[0][1]
        fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
        for axis in axes:
            axis.set_aspect('equal')
            axis.set_xlabel('Forward x (m)')
            axis.set_ylabel('Left y (m)')
            axis.add_patch(plt.Circle((0, 0), args.radius, fill=False, color='black', label='Initial model footprint'))
            axis.plot(0, 0, 'k+')
            axis.grid(alpha=.2)
        axis = axes[0]
        axis.scatter(xy[masks[0], 0], xy[masks[0], 1], s=4, color='steelblue', label='Inside depth frustum only')
        axis.scatter(xy[~masks[0], 0], xy[~masks[0], 1], s=12, color='crimson', label='Outside depth frustum')
        axis.set_title('Newly swept grid, 5 cm above LiDAR floor\nGeometry only; inside does not mean visible/free')
        axis.legend(fontsize=8)
        axis = axes[1]
        selected = (height >= .05) & (height <= 1.8)
        lidar_height = lidar @ normal-offset
        keep = (lidar_height >= .05) & (lidar_height <= 1.8)
        axis.scatter(lidar[keep, 0], lidar[keep, 1], s=2, alpha=.35, label='LiDAR returns above floor')
        axis.scatter(points[selected, 0], points[selected, 1], s=2, alpha=.3, label='Depth hypothesis above floor')
        axis.set_xlim(-.6, 3)
        axis.set_ylim(-1.8, 1.8)
        axis.set_title('Recorded returns, not a free-space map')
        axis.legend(fontsize=8)
        fig.savefig(args.plot, dpi=170)
        plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
