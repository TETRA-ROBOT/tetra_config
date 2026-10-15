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
from launch.conditions import IfCondition


# this is the function launch  system will look for
def generate_launch_description():
    front_lidar_hostname = DeclareLaunchArgument('front_lidar_hostname', default_value='192.168.0.1')
    rear_lidar_hostname = DeclareLaunchArgument('rear_lidar_hostname', default_value='192.168.0.2')
    front_lidar_frame = DeclareLaunchArgument('front_lidar_frame', default_value='laser')
    rear_lidar_frame = DeclareLaunchArgument('rear_lidar_frame', default_value='laser_link2')
    use_keepalive_relay = DeclareLaunchArgument('use_keepalive_relay', default_value='true')
    use_thermal_camera = DeclareLaunchArgument('use_thermal_camera', default_value='false')
    use_gpio_controller = DeclareLaunchArgument('use_gpio_controller', default_value='true')
    use_realsense = DeclareLaunchArgument('use_realsense', default_value='true')
    use_herkulex = DeclareLaunchArgument('use_herkulex', default_value='true')
    use_autonics_rgbd = DeclareLaunchArgument('use_autonics_rgbd', default_value='true')
    use_rb10 = DeclareLaunchArgument('use_rb10', default_value='false')
    thermal_pipeline = DeclareLaunchArgument(
        'thermal_pipeline',
        default_value='rtspsrc location=rtsp://admin:Thermal1594%21@192.168.0.100:554/Primary latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert',
    )
    autonics_interface = DeclareLaunchArgument('autonics_interface', default_value='eno1')
    autonics_ip = DeclareLaunchArgument('autonics_ip', default_value='192.168.0.200')
    autonics_uart = DeclareLaunchArgument('autonics_uart', default_value='/dev/ai_camera')
    rb10_ip = DeclareLaunchArgument('rb10_ip', default_value='10.0.2.7')

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
            parameters=[os.path.join(get_package_share_directory("tetra_bringup"), 'params', 'ekf_mh.yaml')],
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
                        'tetra_mh',
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
        arguments=['/camera1/color/image_raw', '/camera1/color/image_raw_passthrough'],
        condition=IfCondition(LaunchConfiguration('use_keepalive_relay')),
    )

    # web_video_server_node 
    web_video_server_node = Node(
        package='web_video_server', 
        executable='web_video_server',
        output='screen',
        parameters=[
            {"quality": 1,
             "port": 8081}
        ]
    )

    gscam_config = LaunchConfiguration('thermal_pipeline')
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
        ],
        condition=IfCondition(LaunchConfiguration('use_thermal_camera')),
    )


    # tetra_gpio_controller
    tetra_gpio_controller_node = Node(
        package='tetra_gpio_controller', 
        executable='gpio_controller_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gpio_controller')),
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
            use_keepalive_relay,
            use_thermal_camera,
            use_gpio_controller,
            use_realsense,
            use_herkulex,
            use_autonics_rgbd,
            use_rb10,
            thermal_pipeline,
            autonics_interface,
            autonics_ip,
            autonics_uart,
            rb10_ip,
            tetra_wheel_driver_node,
            ekf_localization_node,
            tetra_power_driver_node, 
            iahrs_driver_node,
            tetra_gpio_controller_node, 
            joy_node,
            use_sim_time,
            rsp_node,
            rosbridge_server,
            rosapi_node,
            keepalive_relay,
            web_video_server_node,
            gscam_node,
			
            front_lidar,
            rear_lidar,
            front_filter,
            rear_filter,

            # laser merge
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_package_share_directory('dual_laser_merger'), '/demo_laser_merger.launch.py']),
            ),

            # realsense: use 플래그가 false일 때 패키지가 없어도 파싱되도록 FindPackageShare 사용
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
                condition=IfCondition(LaunchConfiguration('use_realsense')),
            ),

            # apriltag_ros
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_package_share_directory('apriltag_ros'), '/launch/apriltag_detection.launch.py']),
            ),

            # HerkuleX pan-tilt
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('herkulex'), 'launch', 'herkulex_pantilt_launch.py'])),
                condition=IfCondition(LaunchConfiguration('use_herkulex')),
            ),

            # Autonics AI camera
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('autonics_rgbd_ros'), 'launch', 'rgbd_cpp.launch.py'])),
                launch_arguments={
                    "nic": LaunchConfiguration('autonics_interface'),
                    "listen_ip": LaunchConfiguration('autonics_ip'),
                    "uart": LaunchConfiguration('autonics_uart'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_autonics_rgbd')),
            ),

            # RB10 manipulator
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('rbpodo_moveit_config'), 'launch', 'test.launch.py'])),
                launch_arguments={
                    "model_id": "rb10_1300e",
                    "use_fake_hardware": "false",
                    "cb_simulation": "Real",
                    "robot_ip": LaunchConfiguration('rb10_ip'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_rb10')),
            ),
        ]
    )
