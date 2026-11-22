#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
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
    herkulex_port = DeclareLaunchArgument('herkulex_port', default_value='/dev/ttyS2')

    # tetra_wheel_driver Board
    tetra_wheel_driver_node = Node(
        package='tetra_wheel_driver', 
        executable='tetra_wheel_driver',
        output='screen',
        parameters=[
            {
                # 바퀴 반지름·간격을 모델별로 고른다. 빠지면 M 값으로 주행해 회전 속도와 오도메트리가 틀어진다
                "robot_model": "mh",
                "port": LaunchConfiguration('wheel_port'),
                "ekf_option": True,  # default: False
            }
        ]
    )
    
    # EKF Localization
    ekf_localization_node = ekf_node('ekf_mh.yaml')
    
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
    rsp_node = robot_state_publisher_node(xacro_folder='tetra_mh')
    
    # rosbridge_server + rosapi
    rosbridge_server, rosapi_node = rosbridge_nodes(max_message_size=10000000)

       
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

    laser_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        output='screen',
        parameters=[{
            'laser_1_topic': '/scan_filtered_1',
            'laser_2_topic': '/scan_filtered_2',
            'merged_scan_topic': '/scan_merge',
            'target_frame': 'laser_merge',
            # 앞·뒤 스캔은 최대 33 ms 차이로 짝지어진다. 도는 중(0.8 rad/s)에는 1.5° 어긋나므로 odom 으로 같은 시각에 맞춘다
            # (dual_laser_merger fixed_frame, 0002 패치 필요)
            'fixed_frame': 'odom',
            'laser_1_x_offset': 0.0,
            'laser_1_y_offset': 0.0,
            'laser_1_yaw_offset': 0.0,
            # 뒤 라이다를 laser_link2 x 로 -4 cm 옮기던 값은 이 로봇에서 앞 라이다와 3.4 cm 어긋나게 만들었다
            # (2026-11-20 정지 스캔 ICP: URDF 만 쓰면 겹치는 벽 차이 6 mm, -0.04 를 넣으면 34 mm). 외부 보정은 URDF 에 둔다
            'laser_2_x_offset': 0.0,
            'laser_2_y_offset': 0.0,
            'laser_2_yaw_offset': 0.0,
            'tolerance': 0.01,
            'queue_size': 1000,
            'angle_increment': 0.005818,  # 0.333 deg = SICK TiM5xx 해상도. 0.001 이면 6284칸 중 85% 가 inf 라 AMCL max_beams 60 이 유효 빔 7개만 쓴다
            'scan_time': 0.067,
            'range_min': 0.01,
            'range_max': 20.0,
            'min_height': -1.0,
            'max_height': 1.0,
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'inf_epsilon': 1.0,
            'use_inf': True,
            'allowed_radius': 0.45,
            'enable_shadow_filter': False,
            'enable_average_filter': False,
        }],
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
            herkulex_port,
            # 드라이버 노드보다 먼저 와야 노드가 스캔한 포트를 받는다
            *serial_port_actions({'wheel': '/dev/ttyS0', 'power': '/dev/ttyS1', 'imu': '/dev/IMU'}),
            tetra_wheel_driver_node,
            ekf_localization_node,
            tetra_power_driver_node, 
            iahrs_driver_node,
            tetra_gpio_controller_node, 
            joy_node,
            use_sim_time,
            rsp_node,
            jsp_node,
            # 웹 관제가 로봇 위치로 쓰는 /footprint 를 TF(map/odom → base_footprint)에서 발행
            robot_pose_publisher_node(),
            rosbridge_server,
            rosapi_node,
            keepalive_relay,
            web_video_server_node,
            gscam_node,
			
            front_lidar,
            rear_lidar,
            front_filter,
            rear_filter,

            # laser merge: demo_laser_merger.launch.py 의 컴포넌트 컨테이너는 런치가 보내는 load_node 요청이
            # 서비스 매칭 전에 나가 유실되면 merger 가 영영 안 뜬다(/scan_merge 없음, 재시작마다 랜덤). 단독 노드로 같은 파라미터를 쓴다
            laser_merger_node,

            # realsense: use 플래그가 false일 때 패키지가 없어도 파싱되도록 FindPackageShare 사용
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
                condition=IfCondition(LaunchConfiguration('use_realsense')),
            ),

            # apriltag_ros: 공용 apriltag_detection.launch.py 는 M 의 USB 카메라(/camera/image_raw)를 구독해 MH 의 RealSense 영상을 받지 못한다.
            # 예전 MH(ros2_ws)와 같이 RealSense 컬러 영상에서 36h11, 0.1 m 태그를 찾는다 (도킹 플러그인이 이 태그 TF 를 쓴다)
            realsense_apriltag_node(),

            # HerkuleX pan-tilt: 웹 관제의 허큘렉스 캘리브레이션 화면이 쓰는 서비스(herkulex/get_gain 등)를 제공하는 herkulex_driver 로 교체
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('herkulex_driver'), 'launch', 'herkulex_driver.launch.py'])),
                launch_arguments={
                    'serial_port': LaunchConfiguration('herkulex_port'),
                    'baud_rate': '115200',
                    'model': '0602',
                    'params_file': PathJoinSubstitution([FindPackageShare('tetra_bringup'), 'params', 'herkulex_mh.yaml']),
                }.items(),
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
