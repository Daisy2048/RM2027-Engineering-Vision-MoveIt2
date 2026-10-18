from glob import glob
from setuptools import find_packages, setup

package_name = 'assets'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models/openvino_best',
            glob('models/openvino_best/*')),
        ('share/' + package_name + '/videos',
            glob('videos/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zz, lsq',
    maintainer_email='1352234739@qq.com, 1574468941@qq.com',
    description='Shared model and video assets for ROI ROS packages',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
