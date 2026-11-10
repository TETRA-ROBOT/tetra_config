"""웹 관제가 매핑 모드에서 띄우는 Cartographer 2D SLAM (지금은 EDU 만 쓴다, launch-plan.ts resolveSlamLaunch).

Nav2 는 끄지 않고 lifecycle 로 일시정지만 하므로(service-caller.ts runMapping) 여기서는 SLAM 노드만 띄운다.
모델별 cartographer_<model>.launch.py 와 다른 점:
  - RViz 를 띄우지 않는다 (로봇 PC 에 화면이 없을 수 있고, 지도는 웹 관제가 /map 으로 받는다).
  - map→odom 고정 TF 를 발행하지 않는다. Cartographer 가 map→odom 을 직접 발행하므로 같은 변환을 두 곳에서 내면
    TF 가 두 값 사이를 오가 지도와 로봇 위치가 튄다.
  - cartographer_node 를 -collect_metrics 로 띄운다. 웹 관제가 read_metrics 로 스캔 매칭 점수를 읽어 매핑 정확도로 보여 준다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config_dir = os.path.join(get_package_share_directory('tetra_cartographer'), 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cartographer_config_dir', default_value=default_config_dir),
        DeclareLaunchArgument(
            'configuration_basename', default_value='tetra_edu_2d.lua',
            description='Cartographer lua file in cartographer_config_dir'),
        # 웹 관제의 지도 저장(PGM)과 Nav2 지도 해상도에 맞춘다
        DeclareLaunchArgument('resolution', default_value='0.05'),
        # 웹 관제는 /map 을 받아 화면에 그리고, 저장할 때 최근 지도인지(SLAM_MAP_MAX_AGE_MS) 확인한다
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', configuration_basename,
                '-collect_metrics',
            ],
            # 바퀴 odom 대신 EKF 출력(odom→base_footprint TF 와 같은 값)을 쓴다
            remappings=[('/odom', '/odometry/filtered')],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
        ),
    ])
