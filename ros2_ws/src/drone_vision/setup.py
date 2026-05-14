import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdullah',
    maintainer_email='abdullah@example.com',
    description='Autonomous SAR drone — person detection, gimbal-aware tracking, GPS-aided approach, payload delivery.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'person_detector    = drone_vision.person_detector:main',
            'gimbal_sim         = drone_vision.gimbal.gimbal_sim:main',
            'gimbal_controller  = drone_vision.gimbal.gimbal_controller:main',
            'rtsp_camera        = drone_vision.gimbal.rtsp_camera:main',
            'visual_servo       = drone_vision.gimbal.visual_servo:main',
            'geo_localiser      = drone_vision.geo.geo_localiser:main',
            'mavlink_bridge     = drone_vision.bridge.mavlink_bridge:main',
            'gcs_link           = drone_vision.bridge.gcs_link:main',
            'payload_servo      = drone_vision.payload.payload_servo:main',
            'sar_orchestrator   = drone_vision.mission.sar_orchestrator:main',
        ],
    },
)
