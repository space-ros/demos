import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ros_trick_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "params"), glob("params/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Blazej Fiderek (xfiderek)",
    maintainer_email="fiderekblazej@gmail.com",
    description="",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros_trick_bridge_node = ros_trick_bridge.ros_trick_bridge_node:main"
        ],
    },
)
