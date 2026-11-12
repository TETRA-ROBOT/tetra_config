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
                "robot_model": "sm",
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
            parameters=[os.path.join(get_package_share_directory("tetra_bringup"), 'params', 'ekf_sm.yaml')],
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
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('tetra_description'),
                        'urdf',
                        'tetra_sm',
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
            '-topic', 'points2',
        ],
    )
    
    # pointcloud to scan
    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/points2'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'lidar_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -1.8415926,   # -135
            'angle_max':  2.8415926,   # 135
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    line_extraction_node = Node(
        package='laser_line_extraction_ros2',
        executable='line_extraction_node',
        name='line_extractor_node',
        output='screen',
        # 튜닝값은 laser_line_extraction_ros2 example.launch.py(도킹용)에서 옮겨왔다.
        parameters=[{
            'frame_id': 'lidar_link',
            'scan_topic': '/scan',
            'publish_markers': True,
            'bearing_std_dev': 1e-5,
            'range_std_dev': 0.012,
            'least_sq_angle_thresh': 0.0001,
            'least_sq_radius_thresh': 0.0001,
            'max_line_gap': 0.2,
            'min_line_length': 0.15,
            'min_range': 0.05,
            'max_range': 1.0,
            'min_split_dist': 0.04,
            'outlier_dist': 0.06,
            'min_line_points': 5,
        }],
    )
    
    # create and return launch description object
    return LaunchDescription(
        [
            # 드라이버 노드보다 먼저 와야 노드가 스캔한 포트를 받는다
            *serial_port_actions({'wheel': '/dev/ttyS0', 'power': '/dev/ttyS1', 'imu': '/dev/IMU'}),
            tetra_wheel_driver_node,
            ekf_localization_node,
            tetra_power_driver_node, 
            iahrs_driver_node,
            #joy_node,
            use_sim_time,
            rsp_node,
            jsp_node,
            # 웹 관제가 로봇 위치로 쓰는 /footprint 를 TF(map/odom → base_footprint)에서 발행
            Node(package='tetra_bringup', executable='robot_pose_publisher.py', name='robot_pose_publisher', output='screen'),
            rosbridge_server,
            rosapi_node,
            kanavi_lidar_node,
            pointcloud_to_scan_node,
            line_extraction_node,

            # 충전 도킹은 전방 RealSense 컬러 영상의 AprilTag(36h11, 0.1 m)로 한다 (navigation_sm.yaml docking_server).
            # 공용 apriltag_detection.launch.py 는 /camera/image_raw 를 구독해 RealSense 영상을 받지 못하므로 EDU 와 같이 직접 띄운다
            Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='apriltag_node',
                output='screen',
                parameters=[{'family': '36h11', 'size': 0.1}],
                remappings=[
                    ('/image_rect', '/camera1/color/image_raw'),
                    ('/camera_info', '/camera1/color/camera_info'),
                ],
            ),
            # 카메라 이름 기본값(camera1)이어야 URDF camera1_link 아래로 컬러 광학 프레임 TF 가 붙는다.
            # 드라이버가 camera1_link -> camera1_color_frame 으로 RGB 센서의 옆 오프셋을 낸다
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
                launch_arguments={
                    'rgb_camera.color_profile': '640x480x15',
                    'enable_depth': 'false',
                }.items(),
            ),
            
		# USB Camera
		# IncludeLaunchDescription(
		# PythonLaunchDescriptionSource(
		# 	[get_package_share_directory('usb_cam'), '/launch/camera.launch.py']),
		# ),


		# IncludeLaunchDescription(
		# PythonLaunchDescriptionSource(
		# 	[get_package_share_directory('line_tf_publisher'), '/launch/line_tf_publisher.launch.py']),
		# ),
        
        # apriltag_ros 
		# IncludeLaunchDescription(
		# PythonLaunchDescriptionSource(
		# 	[get_package_share_directory('apriltag_ros'), '/launch/apriltag_detection.launch.py']),
		# ),
			
		# sick_tim_571
		#IncludeLaunchDescription(
		#PythonLaunchDescriptionSource(
		#	[get_package_share_directory('sick_scan_xd'), '/launch/sick_tim_5xx.launch.py']),
		#),
		
		# cygbot 2D lidar
		#IncludeLaunchDescription(
		#PythonLaunchDescriptionSource(
		#	[get_package_share_directory('cyglidar_d2_ros2'), '/launch/cyglidar.launch.py']),
		#),
		
		# realsense D455
		#IncludeLaunchDescription(
		#PythonLaunchDescriptionSource(
		#	[get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
		#),

		# Livox MID-360 (3D LiDAR)
		#IncludeLaunchDescription(
		#PythonLaunchDescriptionSource(
		#	[get_package_share_directory('livox_ros_driver2'), '/launch_ROS2/MID360_launch.py']),
		#),

        # laser filter
		# IncludeLaunchDescription(
		# PythonLaunchDescriptionSource(
		# 	[get_package_share_directory('laser_filters'), '/examples/angular_filter_example.launch.py']),
		# ),

        
        ]
    )
