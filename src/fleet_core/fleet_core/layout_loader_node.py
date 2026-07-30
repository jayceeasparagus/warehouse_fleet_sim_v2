import json
from pathlib import Path

import rclpy
from rclpy.node import Node


def grid_to_world(grid_x, grid_y, grid):
    resolution = grid['resolution']
    world_x = (grid_x - grid['width'] / 2) * resolution
    world_y = (grid_y - grid['height'] / 2) * resolution

    return world_x, world_y


class LayoutLoaderNode(Node):
    def __init__(self):
        super().__init__('layout_loader_node')

        self.declare_parameter(
            'layout_file',
            'src/fleet_core/layouts/basic_warehouse.json'
        )

        layout_file = self.get_parameter('layout_file').value
        layout_path = Path(layout_file)

        if not layout_path.exists():
            self.get_logger().error(f'Layout file not found: {layout_path}')
            return

        with layout_path.open('r') as file:
            layout = json.load(file)

        self.print_layout(layout)

    def print_layout(self, layout):
        grid = layout['grid']

        self.get_logger().info(f"Loaded layout: {layout['name']}")
        self.get_logger().info(
            f"Grid: {grid['width']} x {grid['height']} cells, "
            f"resolution={grid['resolution']} m/cell"
        )

        self.get_logger().info(f"Shelves: {len(layout['shelves'])}")
        for shelf in layout['shelves']:
            world_x, world_y = grid_to_world(shelf['x'], shelf['y'], grid)

            self.get_logger().info(
                f"  shelf {shelf['id']}: "
                f"grid=({shelf['x']}, {shelf['y']}), "
                f"world=({world_x:.2f}, {world_y:.2f}), "
                f"yaw={shelf['yaw']}"
            )

        self.get_logger().info(f"Stations: {len(layout['stations'])}")
        for station in layout['stations']:
            world_x, world_y = grid_to_world(station['x'], station['y'], grid)

            self.get_logger().info(
                f"  station {station['id']}: "
                f"grid=({station['x']}, {station['y']}), "
                f"world=({world_x:.2f}, {world_y:.2f}), "
                f"yaw={station['yaw']}"
            )

        self.get_logger().info(f"Robot spawns: {len(layout['robot_spawns'])}")
        for robot in layout['robot_spawns']:
            world_x, world_y = grid_to_world(robot['x'], robot['y'], grid)

            self.get_logger().info(
                f"  {robot['name']}: "
                f"grid=({robot['x']}, {robot['y']}), "
                f"world=({world_x:.2f}, {world_y:.2f}), "
                f"yaw={robot['yaw']}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LayoutLoaderNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
