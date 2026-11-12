#!/usr/bin/env python3

import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# 같은 디렉터리의 공용 모듈: ttyS 포트를 스캔해 구동·전원보드와 IMU 위치를 찾는다
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tetra_serial_ports import serial_port_actions  # noqa: E402
from launch_ros.parameter_descriptions import ParameterValue


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
                "robot_model": "edu",
                # 포트는 serial_port_actions 가 ttyS0~5 를 스캔해 구동보드를 찾아 넣는다.
                # 못 찾으면 EDU 배선 기본값 ttyS2 로 돌아간다 (드라이버 코드 기본값 ttyS0 은 다른 모델 배선)
                "port": LaunchConfiguration('wheel_port'),
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
            parameters=[os.path.join(get_package_share_directory("tetra_bringup"), 'params', 'ekf_edu.yaml')],
            arguments=['--ros-args', '--log-level', 'error']
            #remappings=[('/odometry/filtered', '/odom')],
    )
    
    # tetra_power_driver Board
    tetra_power_driver_node = Node(
        package='tetra_power_driver', 
        executable='tetra_power_driver',
        output='screen',
        parameters=[
            {"port": LaunchConfiguration('power_port')},
            {"m_bConveyor_option": False},
            {"m_bUltrasonic_option": False}
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
    # 바퀴·캐스터는 돌아가는 관절(continuous)이라 관절 값이 없으면 robot_state_publisher 가 그 링크의 TF 를 내지 못한다.
    # /joint_states 를 내는 노드가 없어 RViz RobotModel 이 "No transform from [base_l_wheel_link] ..." 오류를 냈다.
    # 휠 드라이버는 바퀴 각도를 내지 않으므로 0 으로 고정해 발행한다 (모델을 그리기 위한 값이고 주행 계산에는 쓰지 않는다)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # xacro 결과를 문자열로 못박는다. 그냥 두면 launch 가 YAML 로 읽으려 해서, URDF 주석에 ': ' 가 들어가면
            # (예: camera_joint 보정 설명) 'Unable to parse ... as yaml' 로 브링업 전체가 뜨지 않는다
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('tetra_description'),
                        'urdf',
                        'tetra_edu',
                        'tetra.xacro',
                    ]),
                ]),
                value_type=str,
            ),
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
    
    # Kanavi R270
    kanavi_lidar_node = Node(
        package='kanavi_vl', 
        executable='R270',
        name='r270_node',
        output='screen',
        arguments=[
            '-i', '192.168.123.100', '5000',
            '-m', '224.0.0.5',
            '-fix', 'lidar_link',
            '-topic', 'points2'
        ]
    )
    
    # pointcloud to scan
    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/points2'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'lidar_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )
    
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
            # 드라이버 노드보다 먼저 와야 노드가 스캔한 포트를 받는다
            *serial_port_actions({'wheel': '/dev/ttyS2', 'power': '/dev/ttyS3', 'imu': '/dev/IMU'}),
            tetra_wheel_driver_node,
            ekf_localization_node,
            tetra_power_driver_node, 
            joy_node,
            use_sim_time,
            rsp_node,
            jsp_node,
            # 웹 관제가 로봇 위치로 쓰는 /footprint 를 TF(map/odom → base_footprint)에서 발행
            Node(package='tetra_bringup', executable='robot_pose_publisher.py', name='robot_pose_publisher', output='screen'),
            rosbridge_server,
            rosapi_node,
            kanavi_lidar_node,
            pointcloud_to_scan_node,
            web_video_server_node,

        # apriltag_ros
		# 공용 apriltag_detection.launch.py 는 /camera/image_raw 를 구독해 EDU 의 RealSense 컬러 영상을 받지 못한다.
		# 예전 EDU 와 같이 컬러 영상(36h11, 0.1 m 태그)을 보도록 여기서 직접 띄운다
		Node(
			package='apriltag_ros',
			executable='apriltag_node',
			name='apriltag_node',
			output='screen',
			# decimate 기본값 2.0 은 640x480 을 320x240 으로 줄여 찾는다. staging 거리(약 1 m)에서 태그가 20~30 px 로 작아져
			# 'only 0~2 usable detections', 'detections are unstable' 로 도킹이 실패했다. 원본 해상도로 찾고 스레드를 하나 더 준다
			parameters=[{'family': '36h11', 'size': 0.1, 'detector.decimate': 1.0, 'detector.threads': 2}],
			remappings=[
				('/image_rect', '/camera1/color/image_raw'),
				('/camera_info', '/camera1/color/camera_info'),
			],
		),

		# realsense D435
		# 카메라 이름은 기본값(camera1)을 둔다: 관제 기본 영상 토픽이 /camera1/color/image_raw 이고 URDF 의 camera1_link 에 TF 가 붙는다.
		# 지금 EDU 에서 RealSense 는 AprilTag 도킹용 컬러 영상에만 쓴다. Nav2 costmap·collision_monitor 는 R270 의 /points2 만 본다.
		# 포인트클라우드는 구독하는 곳이 없어 끈다 (라이다 도킹으로 바꾸면 RealSense 자체를 뺄 수 있다).
		# 컬러·depth 모두 640x480@15 로 두어 드라이버 기본값(depth 30fps)보다 USB 대역폭·CPU 를 줄인다
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
		launch_arguments={
			'rgb_camera.color_profile': '640x480x15',
			'enable_depth': 'true',
			'depth_module.depth_profile': '640x480x15',
			'pointcloud.enable': 'false',
		}.items(),
		),

        ]
    )
