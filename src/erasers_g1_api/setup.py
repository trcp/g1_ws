#!/usr/bin/env python3
import os
from setuptools import find_packages, setup

package_name = 'erasers_g1_api'

data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))


def package_files(directory, data_files):
    """ディレクトリ階層を保って，実ファイルを一度ずつインストールする．"""
    for path, directories, filenames in os.walk(directory):
        directories[:] = [name for name in directories
                          if not name.startswith('.') and name != '__pycache__']
        files = [os.path.join(path, name) for name in filenames
                 if not name.startswith('.') and not name.endswith(('.pyc', '.swp'))]
        if files:
            data_files.append(
                (
                    "share/" + package_name + "/" + path,
                    files,
                )
            )
    return data_files

# Add directories
data_files = package_files("samples", data_files)
data_files = package_files("config", data_files)

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
            'sample_whisper_recongnition = samples.sample_whisper_recongnition:main',
            'whisper_node = nodes.whisper_node:main',
            'emergency_stop_announcer = nodes.emergency_stop_announcer:main',
            'sample_robot_pose = samples.sample_robot_pose:main',
            'sample_robot_service_client = samples.sample_robot_service_client:main',
            'sample_led = samples.sample_led:main',
            'sample_head_control = samples.sample_head_control:main',
            'sample_hand_control = samples.sample_hand_control:main',
            'sample_amazing_hand_control = samples.sample_amazing_hand_control:main',
            'sample_arm_control = samples.sample_arm_control:main',
            'sample_navigation = samples.sample_navigation:main',
            'sample_audio_capture = samples.sample_audio_capture:main',
            'sample_play_audio = samples.sample_play_audio:main',
            'sample_voice_recongnition = samples.sample_state_voice_recong:main',
            'sample_gemini = samples.sample_state_gemini:main',
            'sample_wait_push_hand = samples.sample_state_wait_push_hand:main',
            'sample_object_grasp = samples.manipulation.sample_object_grasp:main'
        ],
    },
)
