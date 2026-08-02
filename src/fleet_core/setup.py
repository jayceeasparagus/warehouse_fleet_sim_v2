from setuptools import find_packages, setup
from glob import glob

package_name = 'fleet_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/layouts', glob('layouts/*.json')),(
    'share/' + package_name + '/launch',
    glob('launch/*.launch.py'),
),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayce',
    maintainer_email='jayce@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'layout_loader_node = fleet_core.layout_loader_node:main', 'map_builder_node = fleet_core.map_builder_node:main', 'world_builder_node = fleet_core.world_builder_node:main', 'robot_spawner_node = fleet_core.robot_spawner_node:main',
        ],
    },
)
