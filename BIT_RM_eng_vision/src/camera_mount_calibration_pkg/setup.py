from glob import glob

from setuptools import find_packages, setup


package_name = "camera_mount_calibration_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/figures", glob("figures/*.png")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="zz, lsq",
    maintainer_email="1352234739@qq.com, 1574468941@qq.com",
    description="Interactive SO(3) camera installation-angle calibration",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mount_calibration_node = "
            "camera_mount_calibration_pkg.mount_calibration_node:main",
            "so3_visualization = "
            "camera_mount_calibration_pkg.visualize_so3:main",
        ],
    },
)
