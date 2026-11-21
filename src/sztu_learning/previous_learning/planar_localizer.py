"""Seeded planar scan-to-map prototype; estimates a leveled sensor origin, not base_link."""
from dataclasses import asdict, dataclass, replace
import copy
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import yaml


@dataclass(frozen=True)
class MatchingConfig:
    search_xy_m: float = .6
    search_yaw_rad: float = np.deg2rad(20)
    coarse_xy_step_m: float = .15
    coarse_yaw_step_rad: float = np.deg2rad(5)
    scan_voxel_m: float = .08
    maximum_points: int = 1200
    minimum_points: int = 80
    normal_radius_m: float = .20
    correspondence_m: float = .25
    inlier_distance_m: float = .15
    minimum_inlier_fraction: float = .55
    maximum_plane_rmse_m: float = .08
    maximum_endpoint_rmse_m: float = .08
    minimum_geometry_ratio: float = .02
    ambiguity_cost_margin_m2: float = .0008
    distinct_xy_m: float = .30
    distinct_yaw_rad: float = np.deg2rad(8)


def angle_difference(left, right):
    return float(np.arctan2(np.sin(left - right), np.cos(left - right)))


def transform_points(points, pose):
    cosine, sine = np.cos(pose[2]), np.sin(pose[2])
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + pose[:2]


def voxel_sample(points, resolution, maximum):
    _, indices = np.unique(np.floor(points / resolution).astype(np.int64), axis=0, return_index=True)
    sampled = points[np.sort(indices)]
    if len(sampled) > maximum:
        sampled = sampled[np.linspace(0, len(sampled) - 1, maximum, dtype=int)]
    return sampled


class PlanarLocalizer:
    def match_tracking(self, scan, seed):
        local = copy.copy(self)
        local.config = replace(self.config, search_xy_m=.20, search_yaw_rad=np.deg2rad(6),
                               coarse_xy_step_m=.10, coarse_yaw_step_rad=np.deg2rad(2))
        return local.match(scan, seed)

    def __init__(self, grid, resolution, origin, config=None):
        self.config = config or MatchingConfig()
        self.grid = np.asarray(grid)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        for name, value in asdict(self.config).items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError('匹配配置需要有限正数：' + name)
        if self.config.maximum_points < self.config.minimum_points or self.config.inlier_distance_m > self.config.correspondence_m:
            raise ValueError('点数或匹配距离上下界无效')
        if self.config.minimum_inlier_fraction > 1 or self.config.minimum_geometry_ratio > 1:
            raise ValueError('比例配置不能超过1')
        if self.grid.ndim != 2 or not np.isin(self.grid, [-1, 0, 100]).all():
            raise ValueError('地图必须为二维三态栅格')
        if not np.isfinite(self.resolution) or self.resolution <= 0 or self.origin.shape != (2,) or not np.isfinite(self.origin).all():
            raise ValueError('地图分辨率或原点无效')
        rows, columns = np.nonzero(self.grid == 100)
        occupied = self.origin + (np.column_stack([columns, rows]) + .5) * self.resolution
        if len(occupied) < self.config.minimum_points:
            raise ValueError('参考地图占据格不足')
        self.all_tree = cKDTree(occupied)
        normal_points, normals = [], []
        for point, neighbors in zip(occupied, self.all_tree.query_ball_point(occupied, self.config.normal_radius_m)):
            if len(neighbors) < 5:
                continue
            neighborhood = occupied[neighbors]
            centered = neighborhood - neighborhood.mean(axis=0)
            eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered)
            if eigenvalues[1] > 1e-8 and eigenvalues[0] / eigenvalues[1] < .20:
                normal_points.append(point)
                normals.append(eigenvectors[:, 0])
        if len(normal_points) < self.config.minimum_points:
            raise ValueError('地图中可用于匹配的局部表面不足')
        self.surface_points = np.array(normal_points)
        self.normals = np.array(normals)
        self.surface_tree = cKDTree(self.surface_points)

    @classmethod
    def from_directory(cls, directory, config=None):
        directory = Path(directory)
        metadata = yaml.safe_load((directory / 'map.yaml').read_text())
        if metadata.get('mode', 'trinary') != 'trinary' or abs(metadata['origin'][2]) > 1e-12:
            raise ValueError('本原型只支持无原点旋转的三态地图')
        pixels = np.array(Image.open(directory / metadata['image']))
        if pixels.ndim != 2:
            raise ValueError('本原型需要灰度地图')
        probability = pixels / 255. if metadata.get('negate', 0) else (255. - pixels) / 255.
        grid = np.full(pixels.shape, -1, dtype=np.int8)
        grid[probability > metadata['occupied_thresh']] = 100
        grid[probability < metadata['free_thresh']] = 0
        return cls(np.flipud(grid), metadata['resolution'], metadata['origin'][:2], config)

    def _cost(self, points, pose):
        distances, indices = self.all_tree.query(transform_points(points, pose))
        return float(np.mean(np.minimum(distances, .30) ** 2))

    def _correspondences(self, points, pose, maximum_distance=None):
        transformed = transform_points(points, pose)
        distances, indices = self.surface_tree.query(transformed)
        limit = self.config.correspondence_m if maximum_distance is None else maximum_distance
        keep = distances < limit
        rotated = transformed[keep] - pose[:2]
        normals = self.normals[indices[keep]]
        residual = np.sum(normals * (transformed[keep] - self.surface_points[indices[keep]]), axis=1)
        derivative = np.column_stack([-rotated[:, 1], rotated[:, 0]])
        jacobian = np.column_stack([normals, np.sum(normals * derivative, axis=1)])
        return residual, jacobian

    def _refine(self, points, initial):
        pose = initial.copy()
        for iteration in range(35):
            residual, jacobian = self._correspondences(points, pose)
            if len(residual) < self.config.minimum_points:
                break
            weights = np.sqrt(np.minimum(1., .06 / np.maximum(np.abs(residual), 1e-9)))
            step = np.linalg.lstsq(jacobian * weights[:, None], -residual * weights, rcond=1e-6)[0]
            if np.linalg.norm(step[:2]) > .3 or abs(step[2]) > np.deg2rad(10):
                break
            pose += step
            if np.linalg.norm(step[:2]) < .0005 and abs(step[2]) < .0002:
                break
        return pose

    def _distinct(self, left, right):
        return np.linalg.norm(left[:2] - right[:2]) > self.config.distinct_xy_m or abs(angle_difference(left[2], right[2])) > self.config.distinct_yaw_rad

    def _inside_search(self, pose, seed):
        return np.max(np.abs(pose[:2] - seed[:2])) <= self.config.search_xy_m + 1e-8 and abs(angle_difference(pose[2], seed[2])) <= self.config.search_yaw_rad + 1e-8

    def match(self, scan_xy, seed_xy_yaw):
        points = np.asarray(scan_xy, dtype=float)
        seed = np.asarray(seed_xy_yaw, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or seed.shape != (3,) or not np.isfinite(seed).all():
            raise ValueError('输入需要N×2米坐标和[x米,y米,yaw弧度]初值')
        if not np.isfinite(points).all():
            return {'accepted': False, 'pose_xy_yaw': None, 'reasons': ['nonfinite_scan']}
        points = voxel_sample(points, self.config.scan_voxel_m, self.config.maximum_points)
        if len(points) < self.config.minimum_points:
            return {'accepted': False, 'pose_xy_yaw': None, 'reasons': ['insufficient_scan_points']}
        xy_steps = max(1, int(np.ceil(self.config.search_xy_m / self.config.coarse_xy_step_m)))
        yaw_steps = max(1, int(np.ceil(self.config.search_yaw_rad / self.config.coarse_yaw_step_rad)))
        shifts = np.linspace(-self.config.search_xy_m, self.config.search_xy_m, 2 * xy_steps + 1)
        angles = np.linspace(-self.config.search_yaw_rad, self.config.search_yaw_rad, 2 * yaw_steps + 1)
        coarse = []
        search_points = points[::max(1, len(points) // 500)]
        for yaw in angles:
            for shift_x in shifts:
                for shift_y in shifts:
                    pose = seed + np.array([shift_x, shift_y, yaw])
                    coarse.append((self._cost(search_points, pose), pose))
        coarse.sort(key=lambda item: item[0])
        starts = []
        for cost, pose in coarse:
            if not any(np.linalg.norm(pose[:2] - previous[:2]) < .15 and abs(angle_difference(pose[2], previous[2])) < np.deg2rad(4) for previous in starts):
                starts.append(pose)
            if len(starts) == 12:
                break
        candidates = []
        for initial in starts:
            pose = self._refine(points, initial)
            if self._inside_search(pose, seed):
                candidates.append((self._cost(points, pose), pose))
        if not candidates:
            return {'accepted': False, 'pose_xy_yaw': None, 'reasons': ['outside_search_window']}
        candidates.sort(key=lambda item: item[0])
        cost, pose = candidates[0]
        distances, indices = self.all_tree.query(transform_points(points, pose))
        fraction = float(np.mean(distances < self.config.inlier_distance_m))
        endpoint_inliers = distances[distances < self.config.inlier_distance_m]
        endpoint_rmse = float(np.sqrt(np.mean(endpoint_inliers ** 2))) if len(endpoint_inliers) else None
        residual, jacobian = self._correspondences(points, pose, self.config.inlier_distance_m)
        surface_fraction = len(residual) / len(points)
        rmse = float(np.sqrt(np.mean(residual ** 2))) if len(residual) else None
        lever = max(1., float(np.median(np.linalg.norm(points, axis=1))))
        normalized = jacobian / np.array([1., 1., lever])
        eigenvalues = np.linalg.eigvalsh(normalized.T @ normalized / max(1, len(normalized)))
        geometry = float(eigenvalues[0] / max(eigenvalues[-1], 1e-12))
        reasons = []
        if fraction < self.config.minimum_inlier_fraction:
            reasons.append('insufficient_overlap')
        if endpoint_rmse is None or endpoint_rmse > self.config.maximum_endpoint_rmse_m:
            reasons.append('poor_endpoint_fit')
        if len(residual) < self.config.minimum_points or rmse is None or rmse > self.config.maximum_plane_rmse_m:
            reasons.append('poor_surface_fit')
        if geometry < self.config.minimum_geometry_ratio:
            reasons.append('unconstrained_geometry')
        alternate = next((item for item in candidates[1:] if self._distinct(item[1], pose)), None)
        if alternate is not None and alternate[0] - cost < self.config.ambiguity_cost_margin_m2:
            reasons.append('ambiguous_match')
        column, row = np.floor((pose[:2] - self.origin) / self.resolution).astype(int)
        if not (0 <= row < self.grid.shape[0] and 0 <= column < self.grid.shape[1]) or self.grid[row, column] != 0:
            reasons.append('sensor_origin_not_in_known_free_cell')
        accepted = not reasons
        return {'accepted': accepted, 'pose_xy_yaw': pose.tolist() if accepted else None,
                'best_candidate_for_diagnostics_only': pose.tolist(), 'reasons': reasons,
                'inlier_fraction': fraction, 'surface_inlier_fraction': surface_fraction,
                'surface_inlier_count': len(residual), 'endpoint_rmse_m': endpoint_rmse,
                'surface_rmse_m': rmse, 'geometry_ratio': geometry,
                'cost_m2': cost, 'alternative_cost_m2': alternate[0] if alternate else None,
                'sampled_points': len(points), 'seed_xy_yaw': seed.tolist(), 'config': asdict(self.config),
                'reference_point': 'Input leveled scan origin, not robot base_link',
                'limits': 'Seeded local planar prototype; not global localization, covariance, collision clearance or navigation acceptance.'}
