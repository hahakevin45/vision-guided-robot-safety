from setuptools import find_packages, setup

package_name = 'vgr_runtime'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Su',
    maintainer_email='portfolio@users.noreply.github.com',
    description='Real-robot ROS 2 nodes for VGR hardware.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hardware_bridge = vgr_runtime.ros.hardware_bridge:main',
            'cmd_vel_bridge = vgr_runtime.ros.cmd_vel_bridge:main',
            'reverse_cmd_publisher = vgr_runtime.ros.reverse_cmd_publisher:main',
        ],
    },
)
