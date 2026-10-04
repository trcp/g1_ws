from setuptools import find_packages, setup

package_name = 'erasers_g1_person_tracker'

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
    maintainer='roboworks',
    maintainer_email='sakamaki@aibot.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'person_tracker = erasers_g1_person_tracker.person_tracker:main',
            'person_tracker_avoid = erasers_g1_person_tracker.person_tracker_avoid:main',
        ],
    },
)
