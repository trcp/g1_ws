from setuptools import find_packages, setup

package_name = "bell_sound_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="0xNOY",
    maintainer_email="noy@abc-net.jp",
    description="Two-tone bell sound detector for ROS 2.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "bell_sound_detector_node = bell_sound_detector.bell_sound_detector_node:main",
        ],
    },
)
