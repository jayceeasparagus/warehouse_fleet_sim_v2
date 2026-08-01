import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


def grid_to_world(grid_x, grid_y, grid):
    resolution = grid['resolution']
    world_x = (grid_x - grid['width'] / 2) * resolution
    world_y = (grid_y - grid['height'] / 2) * resolution

    return world_x, world_y

def validate_layout(layout):
    required_fields = ['name', 'grid', 'shelves', 'stations', 'robot_spawns']

    for field in required_fields:
        if field not in layout:
            return False, f'missing required field: {field}'

    grid = layout['grid']
    required_grid_fields = ['width', 'height', 'resolution']

    for field in required_grid_fields:
        if field not in grid:
            return False, f'missing required grid field: {field}'

    if grid['width'] <= 0 or grid['height'] <= 0:
        return False, 'grid width and height must be positive'

    if grid['resolution'] <= 0:
        return False, 'grid resolution must be positive'

    shelf_ids = set()

    for shelf in layout['shelves']:
        valid, message = validate_named_pose(shelf, 'shelf', grid)
        if not valid:
            return False, message

        if shelf['id'] in shelf_ids:
            return False, f'duplicate shelf id: {shelf["id"]}'

        shelf_ids.add(shelf['id'])
        
        for field in ['width', 'depth', 'height']:
            if field not in shelf:
                raise ValueError(f"Shelf {shelf.get('id', '?')} is missing '{field}'")

        if shelf['width'] <= 0 or shelf['depth'] <= 0:
            raise ValueError(f"Shelf {shelf['id']} width and depth must be positive")

        if shelf['height'] <= 0:
            raise ValueError(f"Shelf {shelf['id']} height must be positive")

        station_ids = set()

    for station in layout['stations']:
        valid, message = validate_named_pose(station, 'station', grid)
        if not valid:
            return False, message

        if station['id'] in station_ids:
            return False, f'duplicate station id: {station["id"]}'

        if station['id'] not in shelf_ids:
            return False, f'station {station["id"]} does not match a shelf id'

        station_ids.add(station['id'])

    robot_names = set()

    for robot in layout['robot_spawns']:
        valid, message = validate_named_pose(robot, 'robot spawn', grid, id_field='name')
        if not valid:
            return False, message

        if robot['name'] in robot_names:
            return False, f'duplicate robot name: {robot["name"]}'

        robot_names.add(robot['name'])

    return True, 'layout is valid'

def validate_named_pose(item, item_type, grid, id_field='id'):
    required_fields = [id_field, 'x', 'y', 'yaw']

    for field in required_fields:
        if field not in item:
            return False, f'{item_type} is missing field: {field}'

    if item['x'] < 0 or item['x'] >= grid['width']:
        return False, f'{item_type} {item[id_field]} x is outside the grid'

    if item['y'] < 0 or item['y'] >= grid['height']:
        return False, f'{item_type} {item[id_field]} y is outside the grid'

    return True, 'pose is valid'

class LayoutLoaderNode(Node):
    def __init__(self):
        super().__init__('layout_loader_node')

        package_share = Path(get_package_share_directory('fleet_core'))
        default_layout = package_share / 'layouts' / 'basic_warehouse.json'

        self.declare_parameter('layout_file', str(default_layout))

        layout_file = self.get_parameter('layout_file').value
        layout_path = Path(layout_file)

        if not layout_path.exists():
            self.get_logger().error(f'Layout file not found: {layout_path}')
            return

        with layout_path.open('r') as file:
            layout = json.load(file)

        valid, message = validate_layout(layout)
        if not valid:
            self.get_logger().error(f'Layout error: {message}')
            return

        self.get_logger().info(message)
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
