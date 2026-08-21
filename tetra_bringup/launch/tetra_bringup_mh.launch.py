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

    # tetra Motor Driver Board
    tetra_node = Node(
        package='tetra', 
        executable='tetra',
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
            parameters=[os.path.join(get_package_share_directory("tetra_bringup"), 'params', 'ekf.yaml')],
            arguments=['--ros-args', '--log-level', 'error']
            #remappings=[('/odometry/filtered', '/odom')],
    )
    
    # tetra_interface Board
    tetra_interface_node = Node(
        package='tetra_interface', 
        executable='tetra_interface',
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
    
    # tetra_service
    tetra_service_node = Node(
        package='tetra_service', 
        executable='tetra_service',
        respawn= True,
        output='screen',
        parameters=[
            {"robot_model": "mh"},
            {"m_dHome_ID": 0}
        ]
    )

    # rosbridge_server 
    rosbridge_server = Node(
        package='rosbridge_server', 
        executable='rosbridge_websocket',
        output='screen',
        parameters=[
            {"port": 9090, 'max_message_size': 10000000}
        ]
    )

    # rosapi_node 
    rosapi_node = Node(
        package='rosapi', 
        executable='rosapi_node',
        output='screen',
    )
       
    keepalive_relay = Node(
        package='topic_tools',
        executable='relay',
        name='image_keepalive_relay',
        output='screen',
        arguments=['/camera1/color/image_raw', '/camera1/color/image_raw_passthrough']
    )

    # web_video_server_node 
    web_video_server_node = Node(
        package='web_video_server', 
        executable='web_video_server',
        output='screen',
        parameters=[
            {"quality": 1,
             "port ": 8081}
        ]
    )

    gscam_config = "rtspsrc location=rtsp://admin:Thermal1594%21@192.168.0.100:554/Primary latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert"
    # GStreamer 
    gscam_node = Node(
        package='gscam', 
        executable='gscam_node',
        output='screen',
        parameters=[
            {
                "sync_sink": False,
                "gscam_config": gscam_config,
                "frame_id": "camera",
                "use_gst_timestamps": False,
            }
        ]
    )


    # tetra_gpio_controller
    tetra_gpio_controller_node = Node(
        package='tetra_gpio_controller', 
        executable='gpio_controller_node',
        output='screen',
    )
    
    # create and return launch description object
    return LaunchDescription(
        [
            tetra_node,
            ekf_localization_node,
            tetra_interface_node, 
            iahrs_driver_node,
            tetra_gpio_controller_node, 
            joy_node,
            use_sim_time,
            # rsp_node,
            tetra_service_node,
            rosbridge_server,
            rosapi_node,
            keepalive_relay,
            web_video_server_node,
            # gscam_node,
			
            # sick_`tim_571_forward
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('sick_scan_xd'), '/launch/sick_tim_5xx.launch.py']),
            ),

            # laser filter (shadow_filter)
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('laser_filters'), '/examples/shadow_filter_example.launch.py']),
            ),

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
            
            # HerkuleX add...
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('herkulex'), '/launch/herkulex_pantilt_launch.py']),
            ),
            
            # autonics ai camera add...
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('autonics_rgbd_ros'), '/launch/rgbd_cpp.launch.py']),
                launch_arguments={
                    "nic": "eno1",
                    "listen_ip": "192.168.0.200",
                    "uart": "/dev/ai_camera",
                }.items()
            ),

            # rb10 add...
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_package_share_directory('rbpodo_moveit_config'), '/launch/test.launch.py']
                ),
                launch_arguments={
                    "model_id": "rb10_1300e",
                    "use_fake_hardware": "false",
                    "cb_simulation": "Real",
                    "robot_ip": "10.0.2.7"
                }.items()
            ),
        ]
    )
