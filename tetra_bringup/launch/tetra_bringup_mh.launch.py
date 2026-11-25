#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import subprocess

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# 같은 디렉터리의 공용 모듈: 직렬 포트 스캔(tetra_serial_ports), 모델 공통 노드(bringup_common)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tetra_serial_ports import serial_port_actions  # noqa: E402
from bringup_common import (  # noqa: E402
    ekf_node,
    depth_point_cloud_xyz_node,
    joint_state_publisher_node,
    joystick_node,
    realsense_apriltag_node,
    realsense_pointcloud_arguments,
    robot_pose_publisher_node,
    robot_state_publisher_node,
    rosbridge_nodes,
    use_sim_time_argument,
    web_video_server_node,
)
from launch.conditions import IfCondition, UnlessCondition


# this is the function launch  system will look for
# 팔이 있을 때 joint_state_publisher 에 줄 몸체 URDF (팔 관절이 없어 리프트·팬틸트만 0 으로 낸다).
# joint_state_publisher 는 URDF 파일 경로를 인자로 받으면 /robot_description(몸체+팔) 대신 그 파일을 쓴다
BODY_URDF_PATH = '/tmp/tetra_mh_body.urdf'
# 리프트·팬틸트 실제 관절 값. 관제 API 가 /lift_status·서보 상태로 10 Hz 로 낸다 (TetraSingleManagement module-joint-states.ts).
# 받은 관절은 0 대신 이 값을 낸다. 늘 0 이면 TF 몸통·머리가 실제와 달라 MoveIt 카메라 장애물이 로봇 자기 몸을 장애물로 남겼다 (2026-11-25 실기)
MODULE_JOINT_SOURCES = ['/module_joint_states']


def _body_only_joint_state_publisher(context, *args, **kwargs):
    xacro_file = os.path.join(get_package_share_directory('tetra_description'), 'urdf', 'tetra_mh', 'tetra.xacro')
    with open(BODY_URDF_PATH, 'w') as urdf_file:
        urdf_file.write(subprocess.run(['xacro', xacro_file], check=True, capture_output=True, text=True).stdout)
    return [Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        arguments=[BODY_URDF_PATH],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'source_list': MODULE_JOINT_SOURCES}],
        # Jazzy joint_state_publisher 는 파일을 받아도 /robot_description 을 늘 구독해, robot_state_publisher 가 낸 몸체+팔 URDF 로
        # 갈아타고 팔 관절을 0 으로 냈다. joint_state_broadcaster 의 실제 값과 번갈아 들어가 팔이 접혔다 폈다 했다 (2026-11-24 실기).
        # 아무도 내지 않는 토픽으로 돌려 몸체 파일만 쓰게 한다
        remappings=[('robot_description', 'joint_state_publisher_body_description_unused')],
    )]


def generate_launch_description():
    front_lidar_hostname = DeclareLaunchArgument('front_lidar_hostname', default_value='192.168.0.1')
    rear_lidar_hostname = DeclareLaunchArgument('rear_lidar_hostname', default_value='192.168.0.2')
    front_lidar_frame = DeclareLaunchArgument('front_lidar_frame', default_value='laser')
    rear_lidar_frame = DeclareLaunchArgument('rear_lidar_frame', default_value='laser_link2')
    use_keepalive_relay = DeclareLaunchArgument('use_keepalive_relay', default_value='true')
    use_thermal_camera = DeclareLaunchArgument('use_thermal_camera', default_value='false')
    # 예전 관제용 GPIO 컨트롤러(tetra_lab tetra_gpio_controller): 출력 0~7 의 0→1 을 리프트 L0·L8, 팬 90°·0°, 틸트 80°·0°, 팔 원점·접기 명령으로 쓰고
    # 출력을 바로 0 으로 되돌린다. 그래서 웹 GPIO 창의 출력이 켜자마자 꺼졌고, 팬틸트 범위(±80°/±30°)와 팔 몸체 충돌 검사를 거치지 않았다.
    # 2026-11-24 부터 MH 는 끈다: 팔·리프트·팬틸트는 웹 창과 플랜 모듈 액션으로 움직이고, GPIO 출력은 켠 상태가 유지되는 일반 출력이다
    use_gpio_controller = DeclareLaunchArgument('use_gpio_controller', default_value='false')
    use_realsense = DeclareLaunchArgument('use_realsense', default_value='true')
    # RealSense 점구름 /camera1/depth/color/points: Nav2 지역·전역 costmap 의 STVL 장애물(navigation_mh.yaml)과 관제 지도 3D 점구름이 쓴다
    realsense_pointcloud = DeclareLaunchArgument('realsense_pointcloud', default_value='true')
    use_herkulex = DeclareLaunchArgument('use_herkulex', default_value='true')
    use_autonics_rgbd = DeclareLaunchArgument('use_autonics_rgbd', default_value='true')
    # Autonics 깊이 영상 → /camera/depth/points (MoveIt 장애물 인식용). Autonics 를 띄울 때만 뜻이 있다.
    # Nav2 에는 넣지 않는다: 팔 끝에 달려 팔과 같이 움직이고 로봇 몸체·팔을 볼 수 있다 (tetra_config CONTEXT.md 13)
    autonics_pointcloud = DeclareLaunchArgument('autonics_pointcloud', default_value='true')
    use_rb10 = DeclareLaunchArgument('use_rb10', default_value='false')
    thermal_pipeline = DeclareLaunchArgument(
        'thermal_pipeline',
        default_value='rtspsrc location=rtsp://admin:Thermal1594%21@192.168.0.100:554/Primary latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert',
    )
    autonics_interface = DeclareLaunchArgument('autonics_interface', default_value='eno1')
    autonics_ip = DeclareLaunchArgument('autonics_ip', default_value='192.168.0.200')
    autonics_uart = DeclareLaunchArgument('autonics_uart', default_value='/dev/ai_camera')
    rb10_ip = DeclareLaunchArgument('rb10_ip', default_value='10.0.2.7')
    # Autonics 카메라 장착 위치 (로봇팔이 있을 때만 뜻이 있다. 팔이 없으면 팬틸트 위): gripper | pantilt | none
    autonics_camera_mount = DeclareLaunchArgument('autonics_camera_mount', default_value='gripper')
    # 팔 끝단 원(rbpodo tcp) 기준 카메라 메시 원점·자세: 카메라 중심이 위쪽 17.5 cm, 위아래가 실제와 맞는 값 (2026-11-24 실측, tetra_rb10.xacro 설명)
    autonics_gripper_xyz = DeclareLaunchArgument('autonics_gripper_xyz', default_value='-0.0091 -0.0287 -0.1733')
    autonics_gripper_rpy = DeclareLaunchArgument('autonics_gripper_rpy', default_value='3.1416 0 -1.5708')
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
            {"ultrasonic_option": False},
            # 리프트(허리) 모터 위치를 /lift_status 로 낸다. 웹 관제 리프트 창의 높이·단계별 높이가 이 값이다
            {"lift_position_option": True}
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
    # 로봇팔이 있으면(use_rb10) rb10_arm.launch.py 가 몸체+팔 URDF 로 robot_state_publisher 를 띄운다. 여기서도 띄우면 같은 이름 노드가 둘이 된다
    rsp_node = GroupAction([robot_state_publisher_node(xacro_folder='tetra_mh')], condition=UnlessCondition(LaunchConfiguration('use_rb10')))
    # joint_state_publisher 는 URDF 의 움직이는 관절을 모두 0 으로 낸다. 리프트·팬틸트는 관절 값을 내는 드라이버가 없어 이 값으로 TF 를 잇는다.
    # 팔이 있으면 팔 관절은 joint_state_broadcaster 가 내므로, 몸체 URDF 만 넘겨 팔 관절을 0 으로 덮어쓰지 않게 한다
    jsp_node = GroupAction([joint_state_publisher_node(MODULE_JOINT_SOURCES)], condition=UnlessCondition(LaunchConfiguration('use_rb10')))
    jsp_body_node = OpaqueFunction(function=_body_only_joint_state_publisher, condition=IfCondition(LaunchConfiguration('use_rb10')))
    
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
            # odom 움직임 보정은 스캔 시각의 odom TF 를 이 시간만큼 기다린다. 0.01 이면 EKF TF 가 10~20 ms 늦을 때마다
            # 'extrapolation into the future' 로 합친 스캔을 버려 /scan_merge 가 15 Hz → 12 Hz 로 줄었다 (2026-11-24 맵핑 중 초당 1.6 번).
            # 같은 값이 앞·뒤 스캔 짝짓기 벌점(ApproximateTime age penalty)이라 표준 기본값 0.1 로 둔다
            'tolerance': 0.1,
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

    # Autonics 보정 깊이(16UC1 mm, camera_depth_frame = 광학 프레임, 드라이버가 camera_link 아래에 static TF 로 낸다)
    # camera_info 는 영상 옆의 /camera/depth/camera_info(보정 전 내부 파라미터)를 받는다. 보정 뒤 값(camera_info_rect)과 fx·cx 차이가 0.2 % 안이다
    autonics_points_node = depth_point_cloud_xyz_node(
        '/camera/depth/image_rect_raw', '/camera/depth/points', 'autonics_point_cloud_xyz',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_autonics_rgbd'), "' == 'true' and '",
            LaunchConfiguration('autonics_pointcloud'), "' == 'true'"])),
    )
    # 팔 카메라(ToF)는 렌즈 앞 0.2 m 안을 제대로 재지 못해 허공에 점을 낸다. 접은 팔에서 장애물로 들어간 점의 58 % 가 카메라 0.2 m 안이었고
    # octomap 은 한 번 맞으면 칸을 막아 가만히 서 있어도 1분 만에 접기 → 홈 길이 막혔다 (실기 2026-11-25). MoveIt 에는 0.25 m 안을 뺀 점군을 준다.
    # camera_depth_frame 이 광학 프레임이라 z = 카메라 앞 거리다. keep_organized 로 640x480 모양을 남겨 sensors_3d 의 point_subsample(행·열)이 그대로 맞는다
    autonics_near_filter_node = Node(
        package='pcl_ros',
        executable='filter_passthrough_node',
        name='autonics_points_near_filter',
        output='screen',
        parameters=[{
            'filter_field_name': 'z',
            'filter_limit_min': 0.25,
            'filter_limit_max': 2.0,
            'keep_organized': True,
        }],
        remappings=[('input', '/camera/depth/points'), ('output', '/camera/depth/points_filtered')],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_autonics_rgbd'), "' == 'true' and '",
            LaunchConfiguration('autonics_pointcloud'), "' == 'true'"])),
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
            realsense_pointcloud,
            use_herkulex,
            use_autonics_rgbd,
            autonics_pointcloud,
            use_rb10,
            thermal_pipeline,
            autonics_interface,
            autonics_ip,
            autonics_uart,
            rb10_ip,
            autonics_camera_mount,
            autonics_gripper_xyz,
            autonics_gripper_rpy,
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
            jsp_body_node,
            # 웹 관제가 로봇 위치로 쓰는 /footprint 를 TF(map/odom → base_footprint)에서 발행
            robot_pose_publisher_node(),
            rosbridge_server,
            rosapi_node,
            keepalive_relay,
            web_video_server_node(),
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
                launch_arguments=realsense_pointcloud_arguments(LaunchConfiguration('realsense_pointcloud')).items(),
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
                    # 예전 MH 브링업(~/ros2_ws)과 같은 값. 드라이버 기본 best_effort 면 web_video_server·관제(reliable 구독)가 영상을 못 받는다
                    "qos_reliability": "reliable",
                    "rgb_exposure_us": "30000",
                    # 관제는 카메라 상태·사고 기록 사진을 압축 토픽으로 받는다 (api blackbox/camera-topic.ts: 같은 카메라 폴더의 */compressed).
                    # 이 드라이버는 보정 영상만 압축해 낸다 → /camera/color/image_rect/compressed
                    "enable_undistortion": "true",
                    "use_compressed": "true",
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_autonics_rgbd')),
            ),
            autonics_points_node,
            autonics_near_filter_node,

            # RB10 manipulator: 몸체+팔 URDF(tetra_description tetra_mh/tetra_rb10.xacro)로 robot_state_publisher·ros2_control·MoveIt.
            # 팔 패키지는 프로젝트 밖(~/rainbow)이라 robot.env TETRA_EXTRA_ROS_SETUP 으로 불러와야 한다
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('tetra_bringup'), 'launch', 'rb10_arm.launch.py'])),
                launch_arguments={
                    "model_id": "rb10_1300e",
                    "use_fake_hardware": "false",
                    "cb_simulation": "Real",
                    "robot_ip": LaunchConfiguration('rb10_ip'),
                    "autonics_camera_mount": LaunchConfiguration('autonics_camera_mount'),
                    "autonics_gripper_xyz": LaunchConfiguration('autonics_gripper_xyz'),
                    "autonics_gripper_rpy": LaunchConfiguration('autonics_gripper_rpy'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_rb10')),
            ),
        ]
    )
