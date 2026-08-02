import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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
    default_layout = (
        workspace
        / 'src'
        / 'fleet_core'
        / 'layouts'
        / 'basic_warehouse.json'
    )

    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')
    layout_file = LaunchConfiguration('layout_file')
    world_name = LaunchConfiguration('world_name')

    turtlebot_models = Path(
        '/opt/ros/jazzy/share/turtlebot3_gazebo/models'
    )
    existing_resource_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        '',
    )

    if existing_resource_path:
        gazebo_resource_path = (
            f'{turtlebot_models}:{existing_resource_path}'
        )
    else:
        gazebo_resource_path = str(turtlebot_models)

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': gazebo_resource_path,
        },
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

    robot_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='fleet_core',
                executable='robot_spawner_node',
                name='robot_spawner_node',
                output='screen',
                parameters=[
                    {
                        'layout_file': layout_file,
                        'world_name': world_name,
                    }
                ],
            )
        ],
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
        DeclareLaunchArgument(
            'layout_file',
            default_value=str(default_layout),
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='basic_warehouse',
        ),
        gazebo,
        map_server,
        lifecycle_manager,
        rviz,
        robot_spawner,
    ])
