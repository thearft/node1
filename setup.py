from setuptools import setup
from glob import glob
import os

package_name = 'node1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', glob('resource/*')),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Navigation and state nodes for autonomous exploration.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_node_executable = node1.state_node:main',
            'navigation_logic_node = node1.navigation_logic_node:main'
        ],
    },
)
