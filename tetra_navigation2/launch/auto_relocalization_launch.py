"""모델별 내비게이션 런치(tetra_navigation_*.launch.py)가 함께 띄우는 자동 위치 재추정 노드.

AMCL 의 무작위 파티클 주입(recovery_alpha)은 위치를 튀게 해서 모든 모델에서 껐다 (amcl_localization_*.yaml).
대신 이 노드가 스캔이 지도와 맞는 비율을 재다가, 확실히 더 잘 맞고 헷갈리는 후보가 없는 자세일 때만 /initialpose 를 보낸다
(tetra_util/tetra_relocalization/README.md).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('auto_relocalize', default_value='true',
                              description='AMCL 이 위치를 잃으면 스캔을 지도에 맞춰 찾은 자세로 바로잡는다'),
        # AMCL 의 scan_topic 과 같아야 한다 (H·MH 는 scan_merge)
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        Node(
            package='tetra_relocalization',
            executable='auto_relocalizer',
            name='auto_relocalizer',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': LaunchConfiguration('scan_topic'),
            }],
            condition=IfCondition(LaunchConfiguration('auto_relocalize')),
        ),
    ])
