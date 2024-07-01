from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rlilc_leg_pkg2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*/*.zip'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='yursds@gmail.com',
    description='TOLODOES: Package description',
    license='TOLODOES: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rlilc_controller = rlilc_leg_pkg2.rlilc_ctrl_node_sim:main',
            'trajectory_node = rlilc_leg_pkg2.trajectory_node:main',
            'fake_pd_node  =  rlilc_leg_pkg2.fake_pd_node:main',
            'command_rlilc_node  =  rlilc_leg_pkg2.command_rlilc_node:main',
            'homing_node  =  rlilc_leg_pkg2.homing_node:main',
        ],
    },
)
