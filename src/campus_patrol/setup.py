import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'campus_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyojun03',
    maintainer_email='kgywns03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrol_node = campus_patrol.patrol_node:main',
            'logger_node = campus_patrol.logger_node:main',
            'monitor_node = campus_patrol.monitor_node:main',
            'stopstart_control_node = campus_patrol.stopstart_control_node:main',
        ],
    },
)
