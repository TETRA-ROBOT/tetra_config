#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 같은 디렉터리의 공용 모듈: 직렬 포트 스캔(tetra_serial_ports), 모델 공통 노드(bringup_common)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tetra_serial_ports import serial_port_actions  # noqa: E402
from bringup_common import (  # noqa: E402
    ekf_node,
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
                "robot_model": "s",
                "port": LaunchConfiguration('wheel_port'),
                "ekf_option": True,  # default: False
            }
        ]
    )
    
    # EKF Localization
    ekf_localization_node = ekf_node('ekf_s.yaml')
    
    # tetra_power_driver Board
    tetra_power_driver_node = Node(
        package='tetra_power_driver', 
        executable='tetra_power_driver',
        output='screen',
        parameters=[
            {"port": LaunchConfiguration('power_port')},
            {"conveyor_option": False},
            {"ultrasonic_option": False}
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
    rsp_node = robot_state_publisher_node()
    
    # rosbridge_server + rosapi
    rosbridge_server, rosapi_node = rosbridge_nodes()

    
    
    # create and return launch description object
    return LaunchDescription(
        [
            # 드라이버 노드보다 먼저 와야 노드가 스캔한 포트를 받는다
            *serial_port_actions({'wheel': '/dev/ttyS0', 'power': '/dev/ttyS1', 'imu': '/dev/IMU'}),
            tetra_wheel_driver_node,
            ekf_localization_node,
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
            
            # 후면 USB 카메라 + AprilTag (bringup_common.py)
            *usb_camera_with_apriltag(),

		## sick_tim_571
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('sick_scan_xd'), '/launch/sick_tim_5xx.launch.py']),
		),
		
		# laser filter (shadow_filter)
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('laser_filters'), '/examples/shadow_filter_example.launch.py']),
		),
		
		## cygbot 2D lidar
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('cyglidar_d2_ros2'), '/launch/cyglidar.launch.py']),
		),
		
		# realsense D455
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
		),

		# Livox MID-360 (3D LiDAR)
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('livox_ros_driver2'), '/launch_ROS2/MID360_launch.py']),
		),
        
        ]
    )
