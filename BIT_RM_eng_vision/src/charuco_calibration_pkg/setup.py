from glob import glob
from setuptools import find_packages, setup

package_name = "charuco_calibration_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zz, lsq",
    maintainer_email="1352234739@qq.com, 1574468941@qq.com",
    description="High-quality manual ChArUco and circle-grid camera calibration node",
    license="MIT",
    entry_points={
        "console_scripts": [
            "calibration_node = charuco_calibration_pkg.calibration_node:main",
            "offline_calibration = charuco_calibration_pkg.offline_calibration:main",
        ],
    },
)
