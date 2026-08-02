import json
import subprocess
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from fleet_core.layout_loader_node import prepare_layout, validate_layout


class RobotSpawnerNode(Node):
    def __init__(self):
        super().__init__('robot_spawner_node')

        package_share = Path(get_package_share_directory('fleet_core'))

        default_layout = (
            package_share / 'layouts' / 'basic_warehouse.json'
        )

        default_model = Path(
            '/opt/ros/jazzy/share/turtlebot3_gazebo/models/'
            'turtlebot3_waffle_pi/model.sdf'
        )

        self.declare_parameter('layout_file', str(default_layout))
        self.declare_parameter('world_name', 'basic_warehouse')
        self.declare_parameter('robot_model_file', str(default_model))
        self.declare_parameter('robot_z', 0.05)

        layout_file = Path(self.get_parameter('layout_file').value)
        self.world_name = self.get_parameter('world_name').value
        self.robot_model_file = Path(
            self.get_parameter('robot_model_file').value
        )
        self.robot_z = self.get_parameter('robot_z').value

        if not layout_file.exists():
            self.get_logger().error(f'Layout file not found: {layout_file}')
            return

        if not self.robot_model_file.exists():
            self.get_logger().error(
                f'Robot model file not found: {self.robot_model_file}'
            )
            return

        with layout_file.open('r', encoding='utf-8') as file:
            layout = json.load(file)

        layout = prepare_layout(layout)
        valid, message = validate_layout(layout)

        if not valid:
            self.get_logger().error(f'Layout error: {message}')
            return

        self.spawn_robots(layout)

    def grid_to_world(self, layout, grid_x, grid_y):
        grid = layout['grid']

        world_x = (
            grid_x - grid['width'] / 2
        ) * grid['resolution']

        world_y = (
            grid_y - grid['height'] / 2
        ) * grid['resolution']

        return world_x, world_y

    def spawn_robot(self, layout, robot):
        x, y = self.grid_to_world(layout, robot['x'], robot['y'])
        yaw = robot.get('yaw', 0.0)

        command = [
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-world',
            self.world_name,
            '-file',
            str(self.robot_model_file),
            '-name',
            robot['name'],
            '-x',
            str(x),
            '-y',
            str(y),
            '-z',
            str(self.robot_z),
            '-Y',
            str(yaw),
        ]

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
        )

        if result.returncode == 0:
            self.get_logger().info(
                f'Spawned {robot["name"]}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
            )
        else:
            self.get_logger().error(
                f'Failed to spawn {robot["name"]}: '
                f'{result.stderr.strip()}'
            )

    def spawn_robots(self, layout):
        robots = layout.get('robot_spawns', [])

        self.get_logger().info(f'Robot spawns: {len(robots)}')

        for robot in robots:
            self.spawn_robot(layout, robot)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSpawnerNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
