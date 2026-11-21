#!/usr/bin/env python3
"""Pin input data, configuration, installed executables and exported maps."""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            h.update(block)
    return {'sha256':h.hexdigest(), 'size':path.stat().st_size}


def main():
    upstream = ROOT/'upstream'
    patch = subprocess.check_output(['git','diff','--binary'],cwd=upstream)
    (ROOT/'evidence/upstream.patch').write_bytes(patch)
    (ROOT/'evidence/upstream_status.txt').write_bytes(
        subprocess.check_output(['git','status','--short'],cwd=upstream))
    files = []
    for folder in ['config','scripts','launch','tests']:
        files.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    files.extend([ROOT/'env.bash', ROOT/'bin/pcd_merge', ROOT/'evidence/upstream.patch'])
    for folder in ['mapping02','localization01','localization02']:
        files.extend(p for p in (ROOT/'maps'/folder).glob('*') if p.is_file())
    for folder in ['maps/mapping02/registered_bag','maps/mapping02/pcd']:
        files.extend(p for p in (ROOT/folder).rglob('*') if p.is_file())
    for bag in ['room_7x3_20261120_154812','room_7x3_20261120_164421']:
        files.extend(p for p in (ROOT.parent/'previous_learning/bags'/bag).glob('*') if p.is_file())
    packages=['spark_fast_lio','ndt_omp_ros2','lidar_localization_ros2','py_examples','my_nav2_pointcloud_pkg']
    prefixes={p:subprocess.check_output(['ros2','pkg','prefix',p],text=True).strip() for p in packages}
    for prefix in prefixes.values():
        files.extend(p for p in (Path(prefix)/'lib').rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    files.extend([Path('/opt/ros/humble/lib/nav2_controller/controller_server'),
                  Path('/opt/ros/humble/lib/nav2_planner/planner_server'),
                  ROOT/'vendor/opt/ros/humble/lib/octomap_server/octomap_server_node'])
    report={'upstream_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=upstream,text=True).strip(),
            'ros_distro':os.environ.get('ROS_DISTRO'), 'python':platform.python_version(),
            'package_prefixes':prefixes, 'files':{str(p.resolve()):digest(p) for p in sorted(set(files))}}
    (ROOT/'evidence/manifest.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print('Saved hashes for',len(report['files']),'files')


if __name__ == '__main__':
    main()
