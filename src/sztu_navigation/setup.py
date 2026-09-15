from setuptools import find_packages, setup

package_name = 'sztu_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agi',
    maintainer_email='robotics@sztu.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_manager = sztu_navigation.navigation_manager:main',
            'waypoint_executor = sztu_navigation.waypoint_executor:main',
        ],
    },
)
