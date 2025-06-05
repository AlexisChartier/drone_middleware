#!/usr/bin/env python3
"""
Launch ROS 2 pour le middleware drone (Octomap binaire → UDP).

Arguments :
  • drone_id         : identifiant numérique du drone          (def : 1)
  • transport_url    : URL backend UDP (udp://host:port)       (def : udp://127.0.0.1:48484)
  • mtu_payload      : taille payload UDP en octets            (def : 1300)
  • compress         : compresser le blob Octomap (bool)       (def : true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1', description='ID unique du drone'),
        DeclareLaunchArgument(
            'transport_url',
            default_value=LaunchConfiguration('TRANSPORT_URL', default='udp://udp_sink:48484'),
            description='UDP destination for OctoMap stream'
        ),
        DeclareLaunchArgument('mtu_payload', default_value='1300', description='Payload UDP max (octets)'),
        DeclareLaunchArgument('compress', default_value='true', description='Activer compression zlib'),

        Node(
            package='drone_middleware',
            executable='dmw_node',
            name='dmw',
            output='screen',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'transport_url': LaunchConfiguration('transport_url'),
                'mtu_payload': LaunchConfiguration('mtu_payload'),
                'compress': LaunchConfiguration('compress'),
            }]
        )
    ])
