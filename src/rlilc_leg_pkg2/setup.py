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
        (os.path.join('share', package_name, 'models/rl_ilc'), glob('models/rl_ilc/*.yaml')),
        (os.path.join('share', package_name, 'models/rl_ilc'), glob('models/rl_ilc/*.zip')),
        (os.path.join('share', package_name, 'models/rl_classic'), glob('models/rl_classic/*.yaml')),
        (os.path.join('share', package_name, 'models/rl_classic'), glob('models/rl_classic/*.zip')),
        (os.path.join('share', package_name, 'models/rl_new'), glob('models/rl_new/*.yaml')),
        (os.path.join('share', package_name, 'models/rl_new'), glob('models/rl_new/*.zip')),
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
            'trajectory_node = rlilc_leg_pkg2.trajectory_node:main',
            'fake_pd_node  =  rlilc_leg_pkg2.fake_pd_node:main',
            'id_selector_node=  rlilc_leg_pkg2.id_selector_node:main',
            'command_rlilc_node  =  rlilc_leg_pkg2.command_rlilc_node:main',
            'homing_node  =  rlilc_leg_pkg2.homing_node:main',
            'real_command_rlilc_node  =  rlilc_leg_pkg2.real_command_rlilc_node:main',
            'real_homing_node  =  rlilc_leg_pkg2.real_homing_node:main',
        ],
    },
)
