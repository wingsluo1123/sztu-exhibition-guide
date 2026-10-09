from setuptools import find_packages, setup
from glob import glob

package_name = 'sztu_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agi',
    maintainer_email='robotics@sztu.edu.cn',
    description='SZTU navigation module for waypoint navigation and path execution',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_manager = sztu_navigation.navigation_manager:main',
            'waypoint_executor = sztu_navigation.waypoint_executor:main',
            'nav_sensor_adapter = sztu_navigation.nav_sensor_adapter:main',
            'safety_gateway = sztu_navigation.safety_gateway:main',
            'initialpose_retimer = sztu_navigation.initialpose_retimer:main',
        ],
    },
)
