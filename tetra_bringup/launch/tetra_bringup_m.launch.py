#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

# 같은 디렉터리의 공용 모듈: 직렬 포트 스캔(tetra_serial_ports), 모델 공통 노드(bringup_common)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tetra_serial_ports import serial_port_actions  # noqa: E402
from bringup_common import (  # noqa: E402
    ekf_node,
    imu_standstill_gate_node,
    joint_state_publisher_node,
    joystick_node,
    robot_pose_publisher_node,
    robot_state_publisher_node,
    rosbridge_nodes,
    usb_camera_with_apriltag,
    use_sim_time_argument,
)


# this is the function launch  system will look for
def generate_launch_description():

    # tetra_wheel_driver Board
    tetra_wheel_driver_node = Node(
        package='tetra_wheel_driver', 
        executable='tetra_wheel_driver',
        output='screen',
        parameters=[
            {
                # 바퀴 반지름·간격을 모델별로 고른다. 빠지면 M 값으로 주행해 회전 속도와 오도메트리가 틀어진다
                "robot_model": "m",
                "port": LaunchConfiguration('wheel_port'),
                "ekf_option": True,  # default: False
                # 구형 M 은 뒤쪽 서보 버튼이 없어 비상정지 버튼으로 서보를 푼다 (손으로 밀 수 있게)
                "emg_releases_servo": ParameterValue(LaunchConfiguration('emg_releases_servo'), value_type=bool),
                # ekf_m.yaml 이 바퀴 속도와 IMU 를 공분산으로 섞어 쓴다
                "publish_odom_covariance": True,
            }
        ]
    )
    
    # EKF Localization
    ekf_localization_node = ekf_node('ekf_m.yaml')
    # 바퀴가 서 있으면 IMU 를 EKF 로 넘기지 않는다 (ekf_m.yaml imu0: imu/data_gated)
    imu_gate_node = imu_standstill_gate_node()
    
    # tetra_power_driver Board
    tetra_power_driver_node = Node(
        package='tetra_power_driver', 
        executable='tetra_power_driver',
        output='screen',
        parameters=[
            {"port": LaunchConfiguration('power_port')},
            {"conveyor_option": False},
            # 예전 M 은 후방 Cyglidar 대신 초음파로 뒤를 본다 (use_ultrasonic:=true use_cyglidar:=false)
            {"ultrasonic_option": ParameterValue(LaunchConfiguration('use_ultrasonic'), value_type=bool)}
        ]
    )
    
    # IMU Sensor
    iahrs_driver_node = Node(
        package='iahrs_driver', 
        executable='iahrs_driver',
        output='screen',
        parameters=[
            {"port": LaunchConfiguration('imu_port')},
            {"m_bSingle_TF_option": False} # default: True
        ]
    )
    
    # Joystick
    joy_node = joystick_node()
    
    # tetra_URDF
    use_sim_time = use_sim_time_argument()
    jsp_node = joint_state_publisher_node()
    # M 후면 초음파 프레임 (tetra_description/urdf/tetra.xacro). 초음파를 쓰는 구형 M 만 켠다 (Cyglidar 신형은 끔)
    rsp_node = robot_state_publisher_node(xacro_arguments={'m_rear_sonar': LaunchConfiguration('use_ultrasonic')})
    
    # rosbridge_server + rosapi
    rosbridge_server, rosapi_node = rosbridge_nodes(max_message_size=10000000)


    # web_video_server_node 
    web_video_server_node = Node(
        package='web_video_server', 
        executable='web_video_server',
        output='screen',
        parameters=[
            {"quality": 1}
        ]
    )
    
    
    # create and return launch description object
    return LaunchDescription(
        [
            # 구형 M은 초음파를, 신형 M은 Cyglidar를 사용한다.
            DeclareLaunchArgument('use_cyglidar', default_value='false',
                                  description='신형 M 후방 Cyglidar D2 실행'),
            DeclareLaunchArgument('use_ultrasonic', default_value='true',
                                  description='구형 M 후방 초음파 켜기·발행 (range_points_dl/rl/rr/dr)'),
            # 기본값은 구형 M(초음파 사용)일 때만 켠다. 신형 M 은 뒤쪽 서보 버튼이 있어 끈다
            DeclareLaunchArgument('emg_releases_servo', default_value=LaunchConfiguration('use_ultrasonic'),
                                  description='비상정지 버튼을 누르면 서보를 풀기 (구형 M)'),
            DeclareLaunchArgument('use_realsense', default_value='false',
                                  description='Realsense 카메라 실행 (M 기본값: 비활성)'),
            *serial_port_actions({'wheel': '/dev/ttyS0', 'power': '/dev/ttyS1', 'imu': '/dev/IMU'}),
            tetra_wheel_driver_node,
            ekf_localization_node,
            imu_gate_node,
            tetra_power_driver_node, 
            iahrs_driver_node,
            joy_node,
            use_sim_time,
            rsp_node,
            jsp_node,
            # 웹 관제가 로봇 위치로 쓰는 /footprint 를 TF(map/odom → base_footprint)에서 발행
            robot_pose_publisher_node(),
            rosbridge_server,
            rosapi_node,            
            web_video_server_node,
            
            # 후면 USB 카메라 + AprilTag. 후면 usb_cam 은 왜곡이 커서 태그 모서리를 보정해서 쓴다 (bringup_common.py)
            *usb_camera_with_apriltag(undistort_corners=True),

		# sick_tim_571
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('sick_scan_xd'), '/launch/sick_tim_5xx.launch.py']),
		),
		
		# cygbot 2D lidar
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(PathJoinSubstitution([
			FindPackageShare('cyglidar_d2_ros2'), 'launch', 'cyglidar.launch.py'])),
			condition=IfCondition(LaunchConfiguration('use_cyglidar')),
		),

		# realsense D455 
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(PathJoinSubstitution([
			FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])), # rs_multi_camera_launch.py
			condition=IfCondition(LaunchConfiguration('use_realsense')),
		),

        ]
    )
