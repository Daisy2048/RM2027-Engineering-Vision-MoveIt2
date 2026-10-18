from glob import glob
from setuptools import find_packages, setup

package_name = 'pnp_solver_pkg'

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
    description='PnP pose solver node: subscribes detection keypoints, solves camera pose via solvePnP',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pnp_node = pnp_solver_pkg.pnp_node:main',
        ],
    },
)
