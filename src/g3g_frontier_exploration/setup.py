from glob import glob
import os

from setuptools import find_packages, setup


package_name = "g3g_frontier_exploration"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chesium",
    maintainer_email="chesium@hotmail.com",
    description="Frontier exploration support for Nav2 simulation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "frontier_explorer = g3g_frontier_exploration.frontier_explorer:main",
            "post_exploration_traverser = g3g_frontier_exploration.post_exploration_traverser:main",
        ],
    },
)
