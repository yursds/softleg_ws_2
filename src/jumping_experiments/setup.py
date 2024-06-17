from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jumping_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='tolomei.simone5@gmail.com',
    description='TOLODOES: Package description',
    license='TOLODOES: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jumping_node = jumping_experiments.jumping_node:main',
            'getup = jumping_experiments.getup:main',
        ],
    },
)
