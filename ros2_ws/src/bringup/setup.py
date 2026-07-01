import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install the launch files into the package's share dir so `ros2 launch bringup <file>`
        # can find them. Without this line colcon builds the package but ships no launch files.
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Charles Howard",
    maintainer_email="charleskh@gmail.com",
    description="Launch the whole Phase-1 perception graph (camera + TRT detector + foxglove bridge).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},  # launch-only package: no nodes of its own
)
