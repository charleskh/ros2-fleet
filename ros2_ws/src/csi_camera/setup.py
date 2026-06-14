from setuptools import find_packages, setup

package_name = "csi_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Charles Howard",
    maintainer_email="charleskh@gmail.com",
    description="Publish a Jetson CSI camera (IMX219 via Argus) as a ROS2 Image topic.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_node = csi_camera.camera_node:main",
        ],
    },
)
