"""Match XYZ against frozen XYZ surfaces while estimating only planar motion."""
import json
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from scipy.spatial import cKDTree

from build_rviz_map import sha
from continuous_localizer import transform_scan
from planar_localizer import PlanarLocalizer, voxel_sample


def geometry_ratio(jacobian):
    if len(jacobian) == 0:
        return 0.
    horizontal_scale = np.sqrt(np.mean(np.sum(jacobian[:, :2] ** 2, axis=1)) / 2)
    angular_scale = np.sqrt(np.mean(jacobian[:, 2] ** 2))
    if min(horizontal_scale, angular_scale) < 1e-9:
        return 0.
    normalized = jacobian / [horizontal_scale, horizontal_scale, angular_scale]
    eigenvalues = np.linalg.eigvalsh(normalized.T @ normalized / len(normalized))
    return float(max(0., eigenvalues[0]) / max(eigenvalues[-1], 1e-12))


class SpatialLocalizer(PlanarLocalizer):
    @classmethod
    def from_reference(cls, map_directory, directory):
        metadata = json.loads((directory / 'manifest.json').read_text())
        if metadata['map_manifest_sha256'] != sha(map_directory / 'manifest.json') or metadata['reference_sha256'] != sha(directory / 'reference.npz'):
            raise ValueError('三维表面参考身份错误')
        instance = cls.from_directory(map_directory)
        with np.load(directory / 'reference.npz', allow_pickle=False) as data:
            instance.all_tree = cKDTree(data['points3d'])
            instance.surface_points = data['surfaces3d'].copy()
            instance.normals = data['normals3d'].copy()
        instance.surface_tree = cKDTree(instance.surface_points)
        return instance

    def _cost(self, points, pose):
        distances, indices = self.all_tree.query(transform_scan(points, pose))
        return float(np.mean(np.minimum(distances, .30) ** 2))

    def _correspondences(self, points, pose, maximum_distance=None):
        transformed = transform_scan(points, pose)
        distances, indices = self.surface_tree.query(transformed)
        limit = self.config.correspondence_m if maximum_distance is None else maximum_distance
        keep = distances < limit
        rotated = transformed[keep, :2] - pose[:2]
        normals = self.normals[indices[keep]]
        residual = np.sum(normals * (transformed[keep] - self.surface_points[indices[keep]]), axis=1)
        derivative = np.column_stack([-rotated[:, 1], rotated[:, 0]])
        jacobian = np.column_stack([normals[:, :2], np.sum(normals[:, :2] * derivative, axis=1)])
        return residual, jacobian

    def _refine(self, points, initial):
        if len(points) > 500:
            points = points[np.linspace(0, len(points) - 1, 500, dtype=int)]
        pose = initial.copy()
        transformed = transform_scan(points, pose)
        distances, indices = self.all_tree.query(transformed)
        cost = float(np.mean(np.minimum(distances, .30) ** 2))
        for iteration in range(20):
            keep = distances < self.config.correspondence_m
            if np.count_nonzero(keep) < self.config.minimum_points:
                break
            rotated = transformed[keep, :2] - pose[:2]
            residual = (transformed[keep, :2] - self.all_tree.data[indices[keep], :2]).reshape(-1)
            jacobian = np.zeros((len(residual), 3))
            jacobian[0::2, 0], jacobian[1::2, 1] = 1., 1.
            jacobian[0::2, 2], jacobian[1::2, 2] = -rotated[:, 1], rotated[:, 0]
            weights = np.repeat(np.sqrt(np.minimum(1., .06 / np.maximum(distances[keep], 1e-9))), 2)
            step = np.linalg.lstsq(jacobian * weights[:, None], -residual * weights, rcond=1e-6)[0]
            if np.linalg.norm(step[:2]) > .3 or abs(step[2]) > np.deg2rad(10):
                break
            candidate, candidate_cost = pose, cost
            candidate_data = None
            for scale in [1., .5, .25, .125]:
                trial = pose + scale * step
                trial_transformed = transform_scan(points, trial)
                trial_distances, trial_indices = self.all_tree.query(trial_transformed)
                trial_cost = float(np.mean(np.minimum(trial_distances, .30) ** 2))
                if trial_cost < cost - 1e-9:
                    candidate, candidate_cost = trial, trial_cost
                    candidate_data = trial_transformed, trial_distances, trial_indices
                    break
            if candidate_cost >= cost - 1e-9:
                break
            movement = np.linalg.norm(candidate - pose)
            pose, cost = candidate, candidate_cost
            transformed, distances, indices = candidate_data
            if movement < .0005:
                break
        return pose

    def match(self, scan_xy, seed_xy_yaw):
        points = np.asarray(scan_xy, dtype=float)
        seed = np.asarray(seed_xy_yaw, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or seed.shape != (3,) or not np.isfinite(seed).all():
            raise ValueError('需要N×3观测，XY相对IMU原点，Z为本次地板高度')
        if not np.isfinite(points).all():
            return {'accepted': False, 'pose_xy_yaw': None, 'reasons': ['nonfinite_scan']}
        points = voxel_sample(points, self.config.scan_voxel_m, self.config.maximum_points)
        if len(points) < self.config.minimum_points:
            return {'accepted': False, 'pose_xy_yaw': None, 'reasons': ['insufficient_scan_points']}
        coarse_points = points[np.linspace(0, len(points) - 1, min(200, len(points)), dtype=int)]
        xy_steps = int(np.ceil(self.config.search_xy_m / self.config.coarse_xy_step_m))
        yaw_steps = int(np.ceil(self.config.search_yaw_rad / self.config.coarse_yaw_step_rad))
        offsets = np.array(np.meshgrid(np.arange(-yaw_steps, yaw_steps + 1) * self.config.coarse_yaw_step_rad,
                                      np.arange(-xy_steps, xy_steps + 1) * self.config.coarse_xy_step_m,
                                      np.arange(-xy_steps, xy_steps + 1) * self.config.coarse_xy_step_m,
                                      indexing='ij')).reshape(3, -1).T[:, [1, 2, 0]]
        poses = seed + offsets
        cosine, sine = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
        transformed = np.empty((len(poses), len(coarse_points), 3))
        transformed[:, :, 0] = cosine * coarse_points[:, 0] - sine * coarse_points[:, 1] + poses[:, 0, None]
        transformed[:, :, 1] = sine * coarse_points[:, 0] + cosine * coarse_points[:, 1] + poses[:, 1, None]
        transformed[:, :, 2] = coarse_points[:, 2]
        distances, indices = self.all_tree.query(transformed, workers=getattr(self, 'refinement_workers', 1))
        costs = np.mean(np.minimum(distances, .30) ** 2, axis=1)
        coarse = list(zip(costs, poses))
        coarse.sort(key=lambda value: value[0])
        starts = [seed]
        for cost, pose in coarse:
            if all(self._distinct(pose, other) for other in starts):
                starts.append(pose)
            if len(starts) >= 12:
                break
        workers = getattr(self, 'refinement_workers', 1)
        if workers > 1:
            with ThreadPoolExecutor(max_workers=workers) as executor:
                refined = list(executor.map(lambda start: self._refine(points, start), starts))
        else:
            refined = [self._refine(points, start) for start in starts]
        candidates = []
        for pose in refined:
            if self._inside_search(pose, seed):
                candidates.append((self._cost(points, pose), pose))
        if not candidates:
            return {'accepted': False, 'pose_xy_yaw': None, 'reasons': ['outside_search_window']}
        candidates.sort(key=lambda value: value[0])
        cost, pose = candidates[0]
        distances, indices = self.all_tree.query(transform_scan(points, pose))
        inliers = distances < self.config.inlier_distance_m
        fraction = float(inliers.mean())
        endpoint_rmse = float(np.sqrt(np.mean(distances[inliers] ** 2))) if any(inliers) else None
        residual, jacobian = self._correspondences(points, pose, self.config.inlier_distance_m)
        plane_rmse = float(np.sqrt(np.mean(residual ** 2))) if len(residual) else None
        geometry = geometry_ratio(jacobian)
        reasons = []
        if fraction < self.config.minimum_inlier_fraction:
            reasons.append('insufficient_overlap')
        if endpoint_rmse is None or endpoint_rmse > self.config.maximum_endpoint_rmse_m:
            reasons.append('poor_endpoint_fit')
        if len(residual) < self.config.minimum_points or plane_rmse is None or plane_rmse > self.config.maximum_plane_rmse_m:
            reasons.append('poor_surface_fit')
        if geometry < self.config.minimum_geometry_ratio:
            reasons.append('unconstrained_geometry')
        alternate = next((value for value in candidates[1:] if self._distinct(value[1], pose)), None)
        if alternate and alternate[0] - cost < self.config.ambiguity_cost_margin_m2:
            reasons.append('ambiguous_match')
        column, row = np.floor((pose[:2] - self.origin) / self.resolution).astype(int)
        if not (0 <= row < self.grid.shape[0] and 0 <= column < self.grid.shape[1]) or self.grid[row, column] != 0:
            reasons.append('sensor_origin_not_in_known_free_cell')
        return {'accepted': not reasons, 'pose_xy_yaw': pose.tolist() if not reasons else None,
                'reasons': reasons, 'best_candidate_for_diagnostics_only': pose.tolist(),
                'inlier_fraction': fraction, 'endpoint_rmse_m': endpoint_rmse, 'surface_rmse_m': plane_rmse,
                'surface_inlier_count': len(residual), 'geometry_ratio': geometry, 'seed_xy_yaw': seed.tolist()}
