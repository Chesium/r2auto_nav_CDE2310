from setuptools import find_packages, setup

package_name = 'g3_mission_control'

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
    maintainer='ros',
    maintainer_email='chengxi03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_controller = g3_mission_control.mission_controller:main',
            'mission_controller_2 = g3_mission_control.mission_controller_2:main',
            'mission_controller_stub = g3_mission_control.mission_controller_stub:main',
        ],
    },
)
