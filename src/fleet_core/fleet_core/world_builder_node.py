import json
import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from fleet_core.layout_loader_node import validate_layout


class WorldBuilderNode(Node):
    def __init__(self):
        super().__init__('world_builder_node')

        package_dir = get_package_share_directory('fleet_core')
        default_layout = os.path.join(
            package_dir, 'layouts', 'basic_warehouse.json'
        )

        self.declare_parameter('layout_file', default_layout)
        self.declare_parameter(
            'output_dir',
            os.path.join(os.getcwd(), 'generated_worlds')
        )

        layout_file = self.get_parameter('layout_file').value
        output_dir = self.get_parameter('output_dir').value

        with open(layout_file, 'r', encoding='utf-8') as file:
            layout = json.load(file)

        validate_layout(layout)
        os.makedirs(output_dir, exist_ok=True)

        output_file = os.path.join(
            output_dir,
            f"{layout['name']}_world.sdf"
        )

        self.build_world(layout, output_file)
        self.get_logger().info(f'Generated Gazebo world: {output_file}')

    def grid_to_world(self, layout, grid_x, grid_y):
        grid = layout['grid']
        world_x = (
            grid_x - grid['width'] / 2
        ) * grid['resolution']
        world_y = (
            grid_y - grid['height'] / 2
        ) * grid['resolution']
        return world_x, world_y

    def add_box(
        self, world, name, x, y, z,
        size_x, size_y, size_z, yaw,
        color, collision=True
    ):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = (
            f'{x} {y} {z} 0 0 {yaw}'
        )

        link = ET.SubElement(model, 'link', name='link')

        if collision:
            collision_tag = ET.SubElement(
                link, 'collision', name='collision'
            )
            geometry = ET.SubElement(collision_tag, 'geometry')
            box = ET.SubElement(geometry, 'box')
            ET.SubElement(box, 'size').text = (
                f'{size_x} {size_y} {size_z}'
            )

        visual = ET.SubElement(link, 'visual', name='visual')
        geometry = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = (
            f'{size_x} {size_y} {size_z}'
        )

        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = color
        ET.SubElement(material, 'diffuse').text = color

    def build_world(self, layout, output_file):
        grid = layout['grid']
        resolution = grid['resolution']
        world_width = grid['width'] * resolution
        world_height = grid['height'] * resolution

        sdf = ET.Element('sdf', version='1.9')
        world = ET.SubElement(
            sdf, 'world', name=layout['name']
        )

        ET.SubElement(world, 'gravity').text = '0 0 -9.8'

        light = ET.SubElement(
            world, 'light', name='sun', type='directional'
        )
        ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
        ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
        ET.SubElement(light, 'direction').text = '-0.5 0.2 -1'

        self.add_box(
            world, 'floor', 0, 0, -0.05,
            world_width, world_height, 0.1, 0,
            '0.75 0.75 0.75 1'
        )

        wall_height = 1.5
        wall_thickness = 0.1

        self.add_box(
            world, 'north_wall', 0, world_height / 2,
            wall_height / 2, world_width, wall_thickness,
            wall_height, 0, '0.25 0.25 0.28 1'
        )
        self.add_box(
            world, 'south_wall', 0, -world_height / 2,
            wall_height / 2, world_width, wall_thickness,
            wall_height, 0, '0.25 0.25 0.28 1'
        )
        self.add_box(
            world, 'east_wall', world_width / 2, 0,
            wall_height / 2, wall_thickness, world_height,
            wall_height, 0, '0.25 0.25 0.28 1'
        )
        self.add_box(
            world, 'west_wall', -world_width / 2, 0,
            wall_height / 2, wall_thickness, world_height,
            wall_height, 0, '0.25 0.25 0.28 1'
        )

        for shelf in layout['shelves']:
            x, y = self.grid_to_world(
                layout, shelf['x'], shelf['y']
            )
            size_x = shelf['width'] * resolution
            size_y = shelf['depth'] * resolution

            self.add_box(
                world,
                f"shelf_{shelf['id']}",
                x,
                y,
                shelf['height'] / 2,
                size_x,
                size_y,
                shelf['height'],
                shelf['yaw'],
                '0.18 0.22 0.25 1'
            )

        for station in layout['stations']:
            x, y = self.grid_to_world(
                layout, station['x'], station['y']
            )
            self.add_box(
                world,
                f"station_{station['id']}",
                x, y, 0.01,
                0.35, 0.35, 0.02,
                station['yaw'],
                '0.1 0.65 0.35 1',
                collision=False
            )

        ET.indent(sdf, space='  ')
        tree = ET.ElementTree(sdf)
        tree.write(
            output_file,
            encoding='utf-8',
            xml_declaration=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = WorldBuilderNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
