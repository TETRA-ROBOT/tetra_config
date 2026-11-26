"""TETRA-M Cartographer 2D 매핑을 손으로 띄울 때 (점검·개발용).

웹 관제와 같은 cartographer_mapping.launch.py 를 include 하고, use_rviz:=true 면 RViz 를 같이 띄운다.
예전 파일은 없는 occupancy_grid.launch.py·rviz/tetra_cartographer.rviz 를 불러 뜨지 않았고,
Cartographer 가 내는 map→odom 을 고정 TF 로 한 번 더 내서 위치가 튀었다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIGURATION_BASENAME = 'tetra_lds_2d.lua'
SCAN_TOPIC = 'scan'
RVIZ_CONFIG = 'tetra_cartographer_m.rviz'


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('tetra_cartographer')
    use_sim_time = LaunchConfiguration('use_sim_time')

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'cartographer_mapping.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'configuration_basename': LaunchConfiguration('configuration_basename'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'resolution': LaunchConfiguration('resolution'),
            'publish_period_sec': LaunchConfiguration('publish_period_sec'),
        }.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(share, 'rviz', RVIZ_CONFIG)],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('configuration_basename', default_value=CONFIGURATION_BASENAME),
        DeclareLaunchArgument('scan_topic', default_value=SCAN_TOPIC),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        mapping,
        rviz,
    ])
