#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
    realsense_apriltag_node,
    robot_pose_publisher_node,
    robot_state_publisher_node,
    rosbridge_nodes,
    use_sim_time_argument,
)


# this is the function launch  system will look for
def generate_launch_description():
    front_lidar_hostname = DeclareLaunchArgument('front_lidar_hostname', default_value='192.168.0.1')
    rear_lidar_hostname = DeclareLaunchArgument('rear_lidar_hostname', default_value='192.168.0.2')
    front_lidar_frame = DeclareLaunchArgument('front_lidar_frame', default_value='laser')
    rear_lidar_frame = DeclareLaunchArgument('rear_lidar_frame', default_value='laser_link2')

    # tetra_wheel_driver Board
    tetra_wheel_driver_node = Node(
        package='tetra_wheel_driver', 
        executable='tetra_wheel_driver',
        output='screen',
        parameters=[
            {
                # 바퀴 반지름·간격을 모델별로 고른다. 빠지면 M 값으로 주행해 회전 속도와 오도메트리가 틀어진다
                "robot_model": "h",
                "port": LaunchConfiguration('wheel_port'),
                "ekf_option": True,  # default: False
            }
        ]
    )
    
    # EKF Localization
    ekf_localization_node = ekf_node('ekf_h.yaml')
    
    # tetra_power_driver Board
    tetra_power_driver_node = Node(
        package='tetra_power_driver', 
        executable='tetra_power_driver',
        output='screen',
        parameters=[
            {"port": LaunchConfiguration('power_port')},
            # H 리프트의 적재 센서(비었음·걸쳐 있음·다 실림)는 전원 보드의 컨베이어 센서(CP)로 읽는다 (예전 tetra_h_interface 와 같음).
            # 꺼 두면 /conveyor_sensor 가 나오지 않아 관제의 싣기·내리기와 태스크 LOAD·UNLOAD 가 적재 상태를 모른다
            {"conveyor_option": True},
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
    rsp_node = robot_state_publisher_node(xacro_folder='tetraDS')
    
    # rosbridge_server + rosapi
    rosbridge_server, rosapi_node = rosbridge_nodes()


    # sick_tim_5xx.launch.py는 DeclareLaunchArgument 대신 sys.argv만 읽으므로
    # include로 넘긴 인자가 무시된다. 두 대를 구분하려면 드라이버를 직접 실행해야 한다.
    sick_launch_xml = os.path.join(
        get_package_share_directory('sick_scan_xd'), 'launch', 'sick_tim_5xx.launch')
    shadow_filter_params = os.path.join(
        get_package_share_directory('tetra_bringup'), 'params', 'laser_shadow_filter.yaml')

    def sick_lidar(position, scan_topic):
        return Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name=f'sick_tim_5xx_{position}',
            output='screen',
            arguments=[
                sick_launch_xml,
                ['hostname:=', LaunchConfiguration(f'{position}_lidar_hostname')],
                ['frame_id:=', LaunchConfiguration(f'{position}_lidar_frame')],
                f'nodename:=sick_tim_5xx_{position}',
                f'laserscan_topic:={scan_topic}',
                f'cloud_topic:=cloud_{position}',
            ],
        )

    def shadow_filter(position, scan_topic, filtered_topic):
        return Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name=f'{position}_laser_filter',
            output='screen',
            remappings=[('scan', scan_topic), ('scan_filtered', filtered_topic)],
            parameters=[shadow_filter_params],
        )

    front_lidar = sick_lidar('front', '/scan_front')
    rear_lidar = sick_lidar('rear', '/scan_rear')
    front_filter = shadow_filter('front', '/scan_front', '/scan_filtered_1')
    rear_filter = shadow_filter('rear', '/scan_rear', '/scan_filtered_2')
    
    # create and return launch description object
    return LaunchDescription(
        [
            front_lidar_hostname,
            rear_lidar_hostname,
            front_lidar_frame,
            rear_lidar_frame,
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
			
            front_lidar,
            rear_lidar,
            front_filter,
            rear_filter,

            # laser merge
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('dual_laser_merger'), '/demo_laser_merger.launch.py']),
            ),
            
            # realsense D455
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
            ),
            
            # 충전 도킹 태그는 전방 RealSense 컬러 영상에서 찾는다 (navigation_h.yaml docking_server).
            # H 에는 usb_cam 이 없어 예전 usb_camera_apriltag_include()는 발행자 없는 /camera/image_raw 만 구독했다
            realsense_apriltag_node(),
        ]
    )
