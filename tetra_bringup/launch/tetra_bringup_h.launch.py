#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction


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
                "m_bEKF_option": True,  # default: False
            }
        ]
    )
    
    # EKF Localization
    ekf_localization_node= Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("tetra_bringup"), 'params', 'ekf_h.yaml')],
            arguments=['--ros-args', '--log-level', 'error']
            #remappings=[('/odometry/filtered', '/odom')],
    )
    
    # tetra_power_driver Board
    tetra_power_driver_node = Node(
        package='tetra_power_driver', 
        executable='tetra_power_driver',
        output='screen',
        parameters=[
            {"m_bConveyor_option": False},
            {"m_bUltrasonic_option": False}
        ]
    )
    
    # IMU Sensor
    iahrs_driver_node = Node(
        package='iahrs_driver', 
        executable='iahrs_driver',
        output='screen',
        parameters=[
            {"m_bSingle_TF_option": False} # default: True
        ]
    )
    
    # Joystick
    joy_node = Node(
        package='joy', 
        executable='joy_node',
        output='screen',
        parameters=[
            {"deadzone": 0.05}
        ]
    )
    
    # tetra_URDF
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="false")
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('tetra_description'),
                        'urdf',
                        'tetraDS',
                        'tetra.xacro',
                    ]),
                ]),
        }]
    )
    
    # rosbridge_server 
    rosbridge_server = Node(
        package='rosbridge_server', 
        executable='rosbridge_websocket',
        output='screen',
        parameters=[
            {"port": 9090}
        ]
    )

    # rosapi_node 
    rosapi_node = Node(
        package='rosapi', 
        executable='rosapi_node',
        output='screen',
    )

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
            tetra_wheel_driver_node,
            ekf_localization_node,
            tetra_power_driver_node, 
            iahrs_driver_node,
            joy_node,
            use_sim_time,
            rsp_node,
            # 웹 관제가 로봇 위치로 쓰는 /footprint 를 TF(map/odom → base_footprint)에서 발행
            Node(package='tetra_bringup', executable='robot_pose_publisher.py', name='robot_pose_publisher', output='screen'),
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
            
            # apriltag_ros
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('apriltag_ros'), '/launch/apriltag_detection.launch.py']),
            ),
        ]
    )
