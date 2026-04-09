from setuptools import find_packages, setup

package_name = 'g3_aligner'

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
    maintainer='chesium',
    maintainer_email='chesium@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'station_a_aligner = g3_aligner.station_a_aligner:main',
            'station_b_aligner = g3_aligner.station_b_aligner:main',
        ],
    },
)
