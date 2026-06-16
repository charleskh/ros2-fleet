from setuptools import find_packages, setup

package_name = "yolo_detector"

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
    description="Run pretrained YOLO on /image_raw and publish vision_msgs/Detection2DArray.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = yolo_detector.detector_node:main",
        ],
    },
)
