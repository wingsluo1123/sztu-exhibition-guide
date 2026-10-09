import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.fixture
def live_workspace(tmp_path):
    root = Path(__file__).resolve().parents[3]
    workspace = tmp_path / 'workspace'
    tools_dir = workspace / 'tools'
    tools_dir.mkdir(parents=True)
    shutil.copyfile(root / 'tools/run_lio_live.sh', tools_dir / 'run_lio_live.sh')
    commands = tmp_path / 'bin'
    commands.mkdir()
    home = tmp_path / 'home'
    vendor = home / '.aima/env'
    vendor.mkdir(parents=True)
    (vendor / 'bashrc').write_text('export SZTU_VENDOR_LOADED=1\n')
    install = workspace / 'install'
    install.mkdir()
    (install / 'setup.bash').write_text(f'export PATH="{commands}:$PATH"\n')
    for package, executable in (
        ('fast_lio', 'fastlio_mapping'),
        ('sztu_slam', 'cloud_to_velodyne'),
        ('sztu_slam', 'lio_odom_bridge'),
    ):
        target = install / package / 'lib' / package / executable
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('#!/bin/bash\ntouch "$HOME/node_started"\n')
        target.chmod(0o755)
    config = install / 'sztu_slam/share/sztu_slam/config'
    config.mkdir(parents=True)
    for filename in ('fastlio_x2_chest.yaml', 'fastdds_large_cloud.xml'):
        (config / filename).touch()
    ros2 = commands / 'ros2'
    ros2.write_text('''#!/bin/bash
if [ "${SZTU_VENDOR_LOADED:-}" != 1 ]; then
    echo "vendor environment not loaded" >&2
    exit 2
fi
if [ "$1" = run ]; then
    echo Translation
    exit 0
fi
if [ "$1 $2 $3 $4 $5" != "topic info --no-daemon --spin-time 5" ]; then
    echo "cached daemon has no publishers" >&2
    exit 2
fi
if [ "${PROBE_FAILURE:-0}" = 1 ]; then
    echo "DDS probe failed" >&2
    exit 2
fi
case "$6" in
    */lidar_pointcloud|*/imu) count="${SENSOR_PUBLISHERS:-1}" ;;
    /slam/lidar_odom) count="${BRIDGE_PUBLISHERS:-0}" ;;
    /Odometry) count="${ODOM_PUBLISHERS:-0}" ;;
    *) echo "Unknown topic '$6'"; exit 1 ;;
esac
echo "Publisher count: $count"
''')
    ros2.chmod(0o755)
    environment = os.environ.copy()
    environment.update(HOME=str(home), ROS_DOMAIN_ID='0',
                       RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    return workspace, environment


def run_preflight(live_workspace, **overrides):
    workspace, environment = live_workspace
    environment.update(overrides)
    result = subprocess.run(
        ['bash', 'tools/run_lio_live.sh', '--bridge', '--check-only'],
        cwd=workspace, env=environment, capture_output=True, text=True,
        timeout=20,
    )
    assert not (Path(environment['HOME']) / 'node_started').exists()
    assert not (Path(environment['HOME']) / 'sztu_lio_runs').exists()
    return result


def test_live_preflight_discovers_without_cached_daemon(live_workspace):
    result = run_preflight(live_workspace)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '启动前检查通过' in result.stdout


@pytest.mark.parametrize('overrides, expected', [
    ({'ROS_DOMAIN_ID': '42'}, '真机在 domain 0'),
    ({'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}, '64 MB 点云传输配置不会生效'),
    ({'SENSOR_PUBLISHERS': '0'}, '没有发布者'),
    ({'BRIDGE_PUBLISHERS': '1'}, '/slam/lidar_odom 已经有 1 个发布者'),
    ({'ODOM_PUBLISHERS': '1'}, '/Odometry 已经有 1 个发布者'),
    ({'PROBE_FAILURE': '1'}, 'ROS 图检查失败'),
])
def test_live_preflight_rejects_unready_graph(live_workspace, overrides, expected):
    result = run_preflight(live_workspace, **overrides)
    assert result.returncode == 1, result.stdout + result.stderr
    assert expected in result.stdout
