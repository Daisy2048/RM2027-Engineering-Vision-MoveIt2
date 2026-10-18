from glob import glob
from setuptools import find_packages, setup

package_name = 'virtual_camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zz, lsq',
    maintainer_email='1352234739@qq.com, 1574468941@qq.com',
    description='Virtual camera node writing video frames to local shared memory',
    license='MIT',
    entry_points={
        'console_scripts': [
            'virtual_camera_node = virtual_camera_pkg.virtual_camera_node:main',
        ],
    },
)
