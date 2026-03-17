from setuptools import find_packages, setup
from glob import glob

package_name = "g3gzsim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chesium",
    maintainer_email="chesium@hotmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={},
    entry_points={
        "console_scripts": [],
    },
)
