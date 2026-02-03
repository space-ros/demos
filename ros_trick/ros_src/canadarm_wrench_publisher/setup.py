from setuptools import find_packages, setup

package_name = "canadarm_wrench_publisher"

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
    maintainer="Blazej Fiderek (xfiderek)",
    maintainer_email="fiderekblazej@gmail.com",
    description="Get joint torques from trick and publish them as GeometryWrench",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "canadarm_wrench_publisher_node = canadarm_wrench_publisher.canadarm_wrench_publisher_node:main"
        ],
    },
)
