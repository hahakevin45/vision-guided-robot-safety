from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vgr_safety_gate'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Su',
    maintainer_email='portfolio@users.noreply.github.com',
    description='ROS2 real-robot safety gate package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_gate_node = vgr_safety_gate.safety_gate_node:main',
            'bench_pseudo_pose = vgr_safety_gate.bench_pseudo_pose:main',
            'aruco_camera_pose = vgr_safety_gate.aruco_camera_pose:main',
            'pid_go_to_pose = vgr_safety_gate.pid_go_to_pose:main',
            'pose_fusion = vgr_safety_gate.pose_fusion_node:main',
            'vision_gate = vgr_safety_gate.vision_gate:main',
            'blind_distance_driver = vgr_safety_gate.blind_distance_driver:main',
            'sapf_nominal = vgr_safety_gate.sapf_nominal:main',
        ],
    },
)
