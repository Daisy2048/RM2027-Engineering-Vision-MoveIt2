from glob import glob
from setuptools import find_packages, setup

package_name = "comm_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zz, lsq",
    maintainer_email="1352234739@qq.com, 1574468941@qq.com",
    description="Serial communication node for world-frame object poses",
    license="MIT",
    entry_points={
        "console_scripts": [
            "comm_node = comm_pkg.comm_node:main",
        ],
    },
)
