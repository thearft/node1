from setuptools import setup
from glob import glob
import os

package_name = 'node1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install resource files (if any)
        ('share/ament_index/resource_index/packages', glob('resource/*')),
        # Install package.xml so that ROS2 can find your package
        ('share/' + package_name, ['package.xml']),
        # Install all launch files into the package's share folder under "launch"
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Navigation node (Node#1) for autonomous maze exploration on Husarion ROSBot 2/3 Pro',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = node1.navigation_node:main',
        ],
    },
)
