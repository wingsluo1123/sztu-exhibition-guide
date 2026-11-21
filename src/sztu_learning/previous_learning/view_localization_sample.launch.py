"""Fixed map and one independently captured localization sample; no actuation."""
from pathlib import Path
import sys
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root = Path(__file__).resolve().parent
    config = yaml.safe_load((root / 'localization_sample.yaml').read_text())
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(root / 'view_learning_map.launch.py')),
            launch_arguments={'map_directory': str(root / config['map_directory']),
                              'rviz_config': str(root / 'localization_sample.rviz')}.items()),
        ExecuteProcess(cmd=[sys.executable, str(root / 'localize_scan.py'),
                            '--map-directory', str(root / config['map_directory']),
                            '--scan-file', str(root / config['scan_file']),
                            '--scan-key', config['scan_key'], '--seed',
                            *map(str, config['seed_xy_yaw_degrees']), '--publish-sample'], output='screen'),
    ])
