#!/usr/bin/env python3
"""
TETRA Unified Bringup Launch

단일 런치 파일로 6종의 TETRA 모델(M, H, EDU, MH, S, SM)을 통합 실행합니다.
사용 예시:
  ros2 launch tetra_bringup tetra_bringup.launch.py robot_model:=m
  ros2 launch tetra_bringup tetra_bringup.launch.py robot_model:=h
"""

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # 1. 런치 파라미터 정의
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='m',
        description='로봇 모델 선택 (m, h, edu, mh, s, sm)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    ekf_option_arg = DeclareLaunchArgument(
        'ekf_option',
        default_value='true',
        description='EKF 센서 융합 활성화 여부'
    )

    robot_model = LaunchConfiguration('robot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_option = LaunchConfiguration('ekf_option')

    # 2. 공통 하드웨어 드라이버 노드 (Wheel & Power)
    wheel_driver_node = Node(
        package='tetra_wheel_driver',
        executable='tetra_wheel_driver',
        name='tetra_wheel_driver',
        output='screen',
        parameters=[{
            'm_bEKF_option': PythonExpression(["'", ekf_option, "' == 'true'"])
        }]
    )

    power_driver_node = Node(
        package='tetra_power_driver',
        executable='tetra_power_driver',
        name='tetra_power_driver',
        output='screen',
        parameters=[{
            'm_bConveyor_option': False,
            'm_bUltrasonic_option': False
        }]
    )

    # 3. IMU 및 조이스틱 드라이버
    imu_node = Node(
        package='iahrs_driver',
        executable='iahrs_driver',
        name='iahrs_driver',
        output='screen',
        parameters=[{
            'm_bSingle_TF_option': False
        }]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.05
        }]
    )

    # 4. EKF Localization Filter
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('tetra_bringup'), 'params', 'ekf.yaml')],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 5. Robot State Publisher (URDF xacro)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('tetra_description'),
                    'urdf',
                    'tetra.xacro'
                ])
            ])
        }]
    )

    # 6. rosbridge_server (웹 UI 연동 웹소켓 포트 9090)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'max_message_size': 10000000
        }]
    )

    return LaunchDescription([
        robot_model_arg,
        use_sim_time_arg,
        ekf_option_arg,
        wheel_driver_node,
        power_driver_node,
        imu_node,
        joy_node,
        ekf_node,
        rsp_node,
        rosbridge_node
    ])
