"""TETRA-SM 지도 없는 주행

지도 없이 odom 좌표계에서 주행한다 (bt_navigator global_frame: odom). AMCL·map_server 를 띄우지 않는다.
가상벽은 API 가 발행하는 /Local_virtual_wall, /Global_virtual_wall 을 costmap 장애물 레이어가 직접 읽는다.
예전에는 없는 virtual_wall 패키지와 params/navigation.yaml(통합 때 navigation_<모델>.yaml 로 이름이 바뀜)을 찾아
런치 생성 단계에서 예외가 나서 뜨지 않았다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

DEFAULT_PARAMS_FILE = 'nomap_navigation_sm.yaml'


def generate_launch_description() -> LaunchDescription:
    nav_share = get_package_share_directory('tetra_navigation2')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(nav_share, 'params', DEFAULT_PARAMS_FILE),
            description='Nav2 parameter file'),
        navigation_launch,
    ])
