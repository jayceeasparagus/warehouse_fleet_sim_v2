from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace = Path.home() / 'projects' / 'warehouse_fleet_sim_v2'

    default_world = (
        workspace
        / 'generated_worlds'
        / 'basic_warehouse_world.sdf'
    )
    default_map = (
        workspace
        / 'generated_maps'
        / 'basic_warehouse_map.yaml'
    )

    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_file,
                'use_sim_time': False,
            }
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['map_server'],
                'use_sim_time': False,
            }
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value=str(default_world),
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value=str(default_map),
        ),
        gazebo,
        map_server,
        lifecycle_manager,
        rviz,
    ])
