from glob import glob

from setuptools import find_packages, setup

package_name = 'sztu_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # FAST-LIO2 的配置。放在我们仓库里而不是改 FAST_LIO 的 config/，
        # 这样上游升级不会冲突（launch 的 config_path 参数可以指到这儿）。
        # .xml 是 Fast DDS 的传输配置，不装上点云会丢 92%，见那个文件的注释。
        ('share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.xml')),
        # 真机实时建图的 rviz2 配置。tools/run_lio_live.sh --rviz 读的是
        # 源码树那份，这里装一份是为了从 install/ 单独起 rviz 时也能指到。
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agi',
    maintainer_email='robotics@sztu.edu.cn',
    description='SZTU SLAM module for exhibition hall mapping and localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_manager = sztu_slam.slam_manager:main',
            'pose_monitor = sztu_slam.pose_monitor:main',
            'mapping_tool = sztu_slam.mapping_tool:main',
            'cloud_to_velodyne = sztu_slam.cloud_to_velodyne:main',
            'lio_odom_bridge = sztu_slam.lio_odom_bridge:main',
        ],
    },
)
