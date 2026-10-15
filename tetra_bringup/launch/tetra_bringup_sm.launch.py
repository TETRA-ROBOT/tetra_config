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


# this is the function launch  system will look for
def generate_launch_description():

    # tetra_wheel_driver Board
    tetra_wheel_driver_node = Node(
        package='tetra_wheel_driver', 
        executable='tetra_wheel_driver',
        output='screen',
        parameters=[
            {"m_bEKF_option": True} #default: False
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
            tetra_wheel_driver_node,
            ekf_localization_node,
            tetra_power_driver_node, 
            iahrs_driver_node,
            #joy_node,
            use_sim_time,
            rsp_node,
            rosbridge_server,
            rosapi_node,
            kanavi_lidar_node,
            pointcloud_to_scan_node,
            line_extraction_node,
            
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
