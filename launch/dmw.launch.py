#!/usr/bin/env python3
"""
Démarre le middleware drone (dmw_node).

Paramètres launch :
  • drone_id        : identifiant entier du drone      (défaut : 1)
  • transport_url   : URL du backend transport         (défaut : udp://127.0.0.1:48484)
  • flush_ms        : période d’envoi réseau en ms     (défaut : 30)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ---------- arguments ----------
    drone_id_arg = DeclareLaunchArgument(
        'drone_id', default_value='1',
        description='Identifiant numérique unique du drone')
    url_arg = DeclareLaunchArgument(
        'transport_url', default_value='udp://127.0.0.1:48484',
        description='URL du backend transport (udp://host:port, quic://…)')
    flush_arg = DeclareLaunchArgument(
        'flush_ms', default_value='30',
        description='Période de flush réseau en millisecondes')

    # ---------- nœud ---------------
    dmw_node = Node(
        package='drone_middleware',
        executable='dmw_node',
        name='dmw',
        output='screen',
        parameters=[{
            'drone_id':       LaunchConfiguration('drone_id'),
            'transport_url':  LaunchConfiguration('transport_url'),
            'flush_ms':       LaunchConfiguration('flush_ms'),
        }]
    )

    return LaunchDescription([
        drone_id_arg,
        url_arg,
        flush_arg,
        dmw_node
    ])
