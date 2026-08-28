from setuptools import find_packages, setup

ROOT_PACKAGES = find_packages(
    where=".",
    include=("safety_sim*", "gazebo_sim*", "nav2_integration*"),
    exclude=("tests*", "ros2_ws*", "cpp_safe_apf*"),
)
CORE_PACKAGES = find_packages(where="ros2_ws/src/vgr_core")
DRIVER_PACKAGES = find_packages(where="ros2_ws/src/vgr_driver")


setup(
    name="vision-guided-robot-safety",
    version="0.1.0",
    description=(
        "Safety simulation and ROS 2 integration for a vision-guided "
        "differential-drive robot"
    ),
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    packages=ROOT_PACKAGES + CORE_PACKAGES + DRIVER_PACKAGES,
    package_dir={
        "vgr_core": "ros2_ws/src/vgr_core/vgr_core",
        "vgr_driver": "ros2_ws/src/vgr_driver/vgr_driver",
    },
    python_requires=">=3.10",
    install_requires=("numpy>=1.23", "PyYAML>=6.0"),
    extras_require={
        "demo": (
            "opencv-python-headless>=4.8",
            "scipy>=1.9",
            "matplotlib>=3.6",
            "Pillow>=9.0",
            "mcap-ros2-support>=0.5",
        ),
        "dev": ("pytest>=7.0", "pytest-timeout>=2.1"),
    },
    license="MIT",
    author="Kevin Su",
)
