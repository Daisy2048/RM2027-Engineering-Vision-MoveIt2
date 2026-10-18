from glob import glob
from setuptools import find_packages, setup

package_name = "recorder_pkg"

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
    description="H.264 video recorder for shared camera frames",
    license="MIT",
    entry_points={
        "console_scripts": [
            "recorder_node = recorder_pkg.recorder_node:main",
        ],
    },
)
