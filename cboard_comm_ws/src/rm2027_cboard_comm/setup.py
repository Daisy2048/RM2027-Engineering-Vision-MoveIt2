from setuptools import find_packages, setup

package_name = 'rm2027_cboard_comm'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='daisy',
    maintainer_email='fengpei3479982118@qq.com',

    description='RM2027 C-board bidirectional communication',

    license='Apache-2.0',

    extras_require={
        'test': ['pytest'],
    },

    entry_points={
        'console_scripts': [

            'cboard_feedback = '
            'rm2027_cboard_comm.cboard_feedback_node:main',

            'cboard_comm = '
            'rm2027_cboard_comm.cboard_comm_node:main',

            'trajectory_bridge = '
            'rm2027_cboard_comm.trajectory_bridge_node:main',

        ],
    },
)
