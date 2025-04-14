from setuptools import setup
from glob import glob
import os

package_name = 'node1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install package.xml into the share directory
        ('share/ament_index/resource_index/packages', glob('resource/*')),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py']),
        # Install the launch files into share/node1/launch
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 node for wall following navigation in a Search & Navigation Challenge.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = node1.navigation_node:main'
        ],
    },
)
