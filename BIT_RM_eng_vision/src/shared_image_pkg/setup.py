from setuptools import find_packages, setup

package_name = "shared_image_pkg"

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
    maintainer="zz, lsq",
    maintainer_email="1352234739@qq.com, 1574468941@qq.com",
    description="Latest-frame shared memory transport for local ROS processes",
    license="MIT",
)
