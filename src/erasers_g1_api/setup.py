#!/usr/bin/env python3
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'erasers_g1_api'

data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))


def package_files(directory, data_files):
    for path, directories, filenames in os.walk(directory):
        for filename in filenames:
            data_files.append(
                (
                    "share/" + package_name + "/" + path,
                    glob(path + "/**/*.*", recursive=True),
                )
            )
    return data_files

# Add directories
data_files = package_files("samples", data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sample_tts = samples.sample_tts:main',
            'sample_head_control = samples.sample_head_control:main',
            'sample_hand_control = samples.sample_hand_control:main',
            'sample_arm_control = samples.sample_arm_control:main',
            'sample_navigation = samples.sample_navigation:main'
        ],
    },
)
