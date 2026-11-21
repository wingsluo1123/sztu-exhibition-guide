"""Same-map dense surface targets retain walls lost by coarse occupancy projection."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from build_rviz_map import sha, write_json
from planar_localizer import PlanarLocalizer, voxel_sample


def build(map_directory, archive, output, neighbors=20, radius=.35):
    if output.exists():
        raise FileExistsError('参考数据不覆盖')
    mapping = json.loads((map_directory / 'manifest.json').read_text())
    if sha(archive / 'manifest.json') != mapping['input_manifest_sha256']:
        raise ValueError('参考点云必须来自固定地图的同次档案')
    manifest = json.loads((archive / 'manifest.json').read_text())
    transform = json.loads((map_directory / 'transform.json').read_text())
    rotation = np.array(transform['rotation_target_from_source'])
    translation = np.array(transform['translation_m'])
    for name, digest in mapping['files_sha256'].items():
        if sha(map_directory / name) != digest:
            raise ValueError('固定地图身份变化')
    frames = []
    rows = list(csv.DictReader((archive / 'frames.csv').open()))
    for row in rows:
        if sha(archive / row['file']) != manifest['output_sha256'][row['file']]:
            raise ValueError('参考档案内容变化')
        with np.load(archive / row['file']) as frame:
            points = frame['xyz'][::3] @ rotation.T + translation
        points = points[(points[:, 2] >= .25) & (points[:, 2] <= 1.85)]
        frames.append(points)
    dense = voxel_sample(np.concatenate(frames), .07, 200000)
    tree = cKDTree(dense)
    distances, indices = tree.query(dense, k=neighbors)
    neighborhoods = dense[indices]
    centered = neighborhoods - neighborhoods.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered)
    values, vectors = np.linalg.eigh(covariance)
    normals = vectors[:, :, 0]
    keep = ((distances[:, -1] < radius) & (values[:, 0] / np.maximum(values[:, 1], 1e-12) < .2)
            & (np.abs(normals[:, 2]) < .5))
    surfaces = dense[keep, :2]
    normals = normals[keep, :2]
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    _, indices = np.unique(np.floor(surfaces / .04).astype(np.int64), axis=0, return_index=True)
    endpoints = voxel_sample(dense[:, :2], .04, 100000)
    output.mkdir(parents=True)
    np.savez_compressed(output / 'reference.npz', endpoints=endpoints, surfaces=surfaces[indices], normals=normals[indices],
                       points3d=dense, surfaces3d=dense[keep], normals3d=vectors[keep, :, 0])
    write_json(output / 'manifest.json', {
        'map_manifest_sha256': sha(map_directory / 'manifest.json'),
        'archive_manifest_sha256': sha(archive / 'manifest.json'),
        'reference_sha256': sha(output / 'reference.npz'), 'implementation_sha256': sha(Path(__file__)),
        'frames': len(rows), 'surface_targets': len(indices), 'endpoint_targets': len(endpoints),
        'method': 'Same-map registered XYZ, height .25-1.85m,7cm3D voxels, local planes, eigenvalue ratio<.2, abs(normal.z)<.5.',
        'normal_neighbors': neighbors, 'normal_maximum_radius_m': radius,
        'limits': 'Nominal calibration and mapping trajectory retained. Dense targets supplement fixed PGM, no changed map pixels, unknown cells remain unknown.'})
    print('Fixed surface reference:', len(endpoints), 'endpoints,', len(indices), 'surfaces', flush=True)


class FixedSurfaceLocalizer(PlanarLocalizer):
    @classmethod
    def from_reference(cls, map_directory, directory):
        metadata = json.loads((directory / 'manifest.json').read_text())
        if metadata['map_manifest_sha256'] != sha(map_directory / 'manifest.json'):
            raise ValueError('表面参考和地图不同源')
        if metadata['reference_sha256'] != sha(directory / 'reference.npz'):
            raise ValueError('表面参考数据改变')
        instance = cls.from_directory(map_directory)
        with np.load(directory / 'reference.npz', allow_pickle=False) as data:
            instance.all_tree = cKDTree(data['endpoints'])
            instance.surface_points = data['surfaces'].copy()
            instance.normals = data['normals'].copy()
        instance.surface_tree = cKDTree(instance.surface_points)
        return instance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-directory', type=Path, required=True)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--neighbors', type=int, default=20)
    parser.add_argument('--radius', type=float, default=.35)
    args = parser.parse_args()
    build(args.map_directory, args.archive, args.output, args.neighbors, args.radius)


if __name__ == '__main__':
    main()
