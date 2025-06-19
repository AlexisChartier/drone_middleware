#!/usr/bin/env python3
"""
Lance le middleware « Drone » : écoute /octomap_binary et diffuse en UDP.
Arguments côté CLI ou docker-compose :
  • drone_id        : identifiant littéral (ex : DT1)
  • transport_url   : udp://hôte:port        (def. udp://udp_sink:48484)
  • mtu_payload     : taille max fragment    (def. 1300)
  • compress        : bool → zlib            (def. true)
  • octomap_topic   : topic à écouter        (def. /octomap_binary)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:

    # ── arguments « externes » ────────────────────────────────────
    drone_id_arg = DeclareLaunchArgument(
        'drone_id', default_value='DT1',
        description='Identifiant du drone (lettres+chiffres, ex. DT1)'
    )

    transport_arg = DeclareLaunchArgument(
        'transport_url',
        default_value=LaunchConfiguration(
            'TRANSPORT_URL', default='udp://udp_sink:48484'),
        description='Destination UDP pour le flux OctoMap'
    )

    mtu_arg       = DeclareLaunchArgument('mtu_payload',    default_value='1300')
    compress_arg  = DeclareLaunchArgument('compress',       default_value='true')
    topic_arg     = DeclareLaunchArgument('octomap_topic',  default_value='/octomap_binary',
                                          description='Nom ABSOLU du topic OctoMap binaire')

    # ── groupe <namespace>/<dmw> ─────────────────────────────────
    drone_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('drone_id')),          # → /DT1/…
        Node(
            package   = 'drone_middleware',
            executable= 'dmw_node',
            name      = 'dmw',                                      # ne commence pas par un chiffre
            output    = 'screen',
            parameters=[{
                'drone_id'     : LaunchConfiguration('drone_id'),
                'transport_url': LaunchConfiguration('transport_url'),
                'mtu_payload'  : LaunchConfiguration('mtu_payload'),
                'compress'     : LaunchConfiguration('compress'),
                'octomap_topic': LaunchConfiguration('octomap_topic'),  # <── nouveau
            }]
        )
    ])

    # ── description globale ──────────────────────────────────────
    return LaunchDescription([
        drone_id_arg, transport_arg, mtu_arg, compress_arg, topic_arg,
        drone_group
    ])
