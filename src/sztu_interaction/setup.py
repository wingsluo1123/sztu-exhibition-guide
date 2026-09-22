from setuptools import find_packages, setup

package_name = 'sztu_interaction'

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
    description='SZTU interaction module for TTS, emotion, and gesture control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_controller = sztu_interaction.tts_controller:main',
            'emotion_controller = sztu_interaction.emotion_controller:main',
            'gesture_controller = sztu_interaction.gesture_controller:main',
            'locomotion_controller = sztu_interaction.locomotion_controller:main',
            'motion_mode_controller = sztu_interaction.motion_mode_controller:main',
        ],
    },
)
