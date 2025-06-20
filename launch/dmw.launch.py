#!/usr/bin/env python3
"""
Lance le middleware « Drone » : Octomap binaire → UDP.

Arguments disponibles (CLI ou docker-compose) :
  • drone_id         : identifiant littéral (ex : DT1)
  • transport_url    : udp://hôte:port           (def. udp://udp_sink:48484)
  • mtu_payload      : taille max fragment (o)   (def. 1300)
  • compress         : bool → zlib               (def. true)
  • octomap_topic    : topic Octomap ABSOLU      (def. /octomap_binary)
  • enable_net_guard : activer buffer wifi       (def. true)
  • wifi_iface       : interface Wi-Fi à sonder  (def. wlan0)
  • rssi_min         : RSSI seuil (dBm)          (def. −75)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    # ───────────────  arguments externes  ────────────────────────
    drone_id_arg   = DeclareLaunchArgument('drone_id', default_value='DT1')
    transport_arg  = DeclareLaunchArgument(
        'transport_url',
        default_value=LaunchConfiguration(
            'TRANSPORT_URL', default='udp://map-server:9000'))
    mtu_arg        = DeclareLaunchArgument('mtu_payload',    default_value='1300')
    compress_arg   = DeclareLaunchArgument('compress',       default_value='true')
    topic_arg      = DeclareLaunchArgument('octomap_topic',  default_value='/octomap_binary')
    guard_arg      = DeclareLaunchArgument('enable_net_guard', default_value='false ')
    wifi_arg       = DeclareLaunchArgument('wifi_iface',     default_value='wlan0')
    rssi_arg       = DeclareLaunchArgument('rssi_min',       default_value='-75')

    # ───────────────  groupe /<drone_id>/dmw  ────────────────────
    drone_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('drone_id')),        # → /DT1/…
        Node(
            package   ='drone_middleware',
            executable='dmw_node',
            name      ='dmw',
            output    ='screen',
            parameters=[{
                'drone_id'        : LaunchConfiguration('drone_id'),
                'transport_url'   : LaunchConfiguration('transport_url'),
                'mtu_payload'     : LaunchConfiguration('mtu_payload'),
                'compress'        : LaunchConfiguration('compress'),
                'octomap_topic'   : LaunchConfiguration('octomap_topic'),
                'enable_net_guard': LaunchConfiguration('enable_net_guard'),
                'wifi_iface'      : LaunchConfiguration('wifi_iface'),
                'rssi_min'        : LaunchConfiguration('rssi_min'),
            }]
        )
    ])

    # ───────────────  description globale  ───────────────────────
    return LaunchDescription([
        drone_id_arg, transport_arg, mtu_arg, compress_arg,
        topic_arg, guard_arg, wifi_arg, rssi_arg,      # ← tous les arguments
        drone_group                                    # ← puis le groupe
    ])
