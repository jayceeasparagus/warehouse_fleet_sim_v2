import json
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from fleet_core.layout_loader_node import prepare_layout, validate_layout


FREE = 254
OCCUPIED = 0


def fill_rotated_rectangle(
    pixels,
    center_x,
    center_y,
    width_meters,
    depth_meters,
    yaw,
    map_resolution,
):
    image_height = len(pixels)
    image_width = len(pixels[0])

    half_width = width_meters / 2
    half_depth = depth_meters / 2

    search_radius = math.ceil(
        math.hypot(half_width, half_depth) / map_resolution
    )

    cosine = math.cos(yaw)
    sine = math.sin(yaw)

    for pixel_y in range(
        center_y - search_radius,
        center_y + search_radius + 1,
    ):
        for pixel_x in range(
            center_x - search_radius,
            center_x + search_radius + 1,
        ):
            if not (
                0 <= pixel_x < image_width
                and 0 <= pixel_y < image_height
            ):
                continue

            offset_x = (pixel_x - center_x) * map_resolution
            offset_y = (pixel_y - center_y) * map_resolution

            local_x = cosine * offset_x + sine * offset_y
            local_y = -sine * offset_x + cosine * offset_y

            if (
                abs(local_x) <= half_width
                and abs(local_y) <= half_depth
            ):
                image_row = image_height - 1 - pixel_y
                pixels[image_row][pixel_x] = OCCUPIED


def create_map(layout, map_resolution):
    grid = layout['grid']
    grid_resolution = grid['resolution']

    map_width_meters = grid['width'] * grid_resolution
    map_height_meters = grid['height'] * grid_resolution

    pixel_width = round(map_width_meters / map_resolution)
    pixel_height = round(map_height_meters / map_resolution)

    pixels = [
        bytearray([FREE] * pixel_width)
        for _ in range(pixel_height)
    ]

    wall_size = max(1, round(0.1 / map_resolution))

    for row in range(wall_size):
        pixels[row] = bytearray([OCCUPIED] * pixel_width)
        pixels[pixel_height - 1 - row] = bytearray(
            [OCCUPIED] * pixel_width
        )

    for row in pixels:
        for column in range(wall_size):
            row[column] = OCCUPIED
            row[pixel_width - 1 - column] = OCCUPIED

    for shelf in layout['shelves']:
        center_x = round(
            shelf['x'] * grid_resolution / map_resolution
        )
        center_y = round(
            shelf['y'] * grid_resolution / map_resolution
        )

        width_meters = shelf['width'] * grid_resolution
        depth_meters = shelf['depth'] * grid_resolution

        fill_rotated_rectangle(
            pixels=pixels,
            center_x=center_x,
            center_y=center_y,
            width_meters=width_meters,
            depth_meters=depth_meters,
            yaw=shelf['yaw'],
            map_resolution=map_resolution,
        )

    origin_x = -map_width_meters / 2
    origin_y = -map_height_meters / 2

    return pixels, origin_x, origin_y


def write_pgm(path, pixels):
    height = len(pixels)
    width = len(pixels[0])

    with path.open('wb') as file:
        header = f'P5\n{width} {height}\n255\n'
        file.write(header.encode('ascii'))

        for row in pixels:
            file.write(row)


def write_yaml(path, image_name, resolution, origin_x, origin_y):
    contents = (
        f'image: {image_name}\n'
        'mode: trinary\n'
        f'resolution: {resolution}\n'
        f'origin: [{origin_x}, {origin_y}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    )

    path.write_text(contents)


class MapBuilderNode(Node):
    def __init__(self):
        super().__init__('map_builder_node')

        package_share = Path(get_package_share_directory('fleet_core'))
        default_layout = package_share / 'layouts' / 'basic_warehouse.json'
        default_output = Path.cwd() / 'generated_maps'

        self.declare_parameter('layout_file', str(default_layout))
        self.declare_parameter('output_dir', str(default_output))
        self.declare_parameter('map_resolution', 0.05)

        layout_path = Path(self.get_parameter('layout_file').value)
        output_dir = Path(self.get_parameter('output_dir').value)
        map_resolution = self.get_parameter('map_resolution').value

        if not layout_path.exists():
            self.get_logger().error(
                f'Layout file does not exist: {layout_path}'
            )
            return

        with layout_path.open('r') as file:
            layout = json.load(file)

        layout = prepare_layout(layout)

        valid, message = validate_layout(layout)

        if not valid:
            self.get_logger().error(f'Layout error: {message}')
            return

        output_dir.mkdir(parents=True, exist_ok=True)

        map_name = f'{layout["name"]}_map'
        pgm_path = output_dir / f'{map_name}.pgm'
        yaml_path = output_dir / f'{map_name}.yaml'

        pixels, origin_x, origin_y = create_map(
            layout,
            map_resolution,
        )

        write_pgm(pgm_path, pixels)
        write_yaml(
            yaml_path,
            pgm_path.name,
            map_resolution,
            origin_x,
            origin_y,
        )

        self.get_logger().info(f'Generated map: {pgm_path}')
        self.get_logger().info(f'Generated metadata: {yaml_path}')


def main(args=None):
    rclpy.init(args=args)
    node = MapBuilderNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
