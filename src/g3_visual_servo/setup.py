from setuptools import find_packages, setup
import os
from glob import glob

setup(
    name='g3_visual_servo',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/g3_visual_servo']),
        ('share/g3_visual_servo', ['package.xml']),
        ('share/g3_visual_servo/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    entry_points={
        'console_scripts': [
            'aruco_dock = g3_visual_servo.aruco_dock_node:main',
        ],
    },
)
