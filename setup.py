from setuptools import setup
from glob import glob
import os

package_name = 'cde2310_custom_robot_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jitisha Arora',
    maintainer_email='jitisha.arora19@gmail.com',
    description='CDE2310 final project custom robot stack',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_node = cde2310_custom_robot_stack.frontier_node:main',
            'docking_node = cde2310_custom_robot_stack.docking_node:main',
            'marker_watcher_node = cde2310_custom_robot_stack.marker_watcher_node:main',
            'supervisor_node = cde2310_custom_robot_stack.supervisor_node:main',
            'launching_node = cde2310_custom_robot_stack.launching_node:main',
        ],
    },
)
