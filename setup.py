from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ieee_robotics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='IEEE Firefighting Robot Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fire_detection_node = ieee_robotics.fire_detection_node:main',
            'firehose_controller = ieee_robotics.firehose_controller:main',
            'odometry_publisher = ieee_robotics.odometry_publisher:main',
        ],
    },
)

