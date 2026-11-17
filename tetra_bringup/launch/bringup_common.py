"""TETRA 브링업 공용: 모델과 상관없이 같은 노드(URDF 발행, 조이스틱, rosbridge, 위치 발행, EKF, 후면 카메라 + AprilTag)를 만든다.

모델별 launch 파일 10개가 이 블록들을 각자 복사해 갖고 있었다. 한 곳만 고치고 나머지를 빠뜨리는 일이 실제로 있었다:
apriltag 의 undistort_corners 가 tetra_bringup_m 에만 들어가고, 같은 카메라를 쓰는 tetra_3d_bringup 에는 빠졌다 (2026-11-16 고침).

모델마다 다른 것(센서, 직렬 포트, xacro 폴더·인자, EKF 파라미터 파일)은 각 launch 파일이 인자로 넘긴다.
쓰는 법은 tetra_serial_ports.py 와 같다: launch 파일에서 sys.path 에 이 폴더를 넣고 import 한다.
"""

import os
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ROSBRIDGE_PORT = 9090


def use_sim_time_argument() -> DeclareLaunchArgument:
    """robot_state_publisher·joint_state_publisher 가 읽는 use_sim_time 인자. 두 노드보다 앞에 넣는다."""
    return DeclareLaunchArgument('use_sim_time', default_value='false')


def joint_state_publisher_node() -> Node:
    """바퀴·캐스터 관절 값을 0 으로 발행한다.

    바퀴·캐스터는 돌아가는 관절(continuous)이라 관절 값이 없으면 robot_state_publisher 가 그 링크의 TF 를 내지 못한다.
    /joint_states 를 내는 노드가 없어 RViz RobotModel 이 "No transform from [base_l_wheel_link] ..." 오류를 냈다.
    휠 드라이버는 바퀴 각도를 내지 않으므로 0 으로 고정해 발행한다 (모델을 그리기 위한 값이고 주행 계산에는 쓰지 않는다)
    """
    return Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )


def robot_state_publisher_node(
    xacro_folder: Optional[str] = None,
    xacro_arguments: Optional[Dict[str, SomeSubstitutionsType]] = None,
    description_as_string: bool = False,
) -> Node:
    """URDF 를 xacro 로 만들어 TF 를 발행한다.

    xacro_folder: tetra_description/urdf 아래 모델 폴더 ('tetraDS', 'tetra_mh' 등). None 이면 urdf/tetra.xacro (M·S)
    xacro_arguments: xacro 인자 {이름: 값}. 값은 문자열이나 LaunchConfiguration (예: {'m_rear_sonar': LaunchConfiguration('use_ultrasonic')})
    description_as_string: xacro 결과를 문자열로 못박는다. 그냥 두면 launch 가 YAML 로 읽으려 해서, URDF 주석에 ': ' 가 들어가면
        'Unable to parse ... as yaml' 로 브링업 전체가 뜨지 않는다 (EDU)
    """
    xacro_path = ['urdf', xacro_folder, 'tetra.xacro'] if xacro_folder else ['urdf', 'tetra.xacro']
    xacro_command = ['xacro ', PathJoinSubstitution([FindPackageShare('tetra_description'), *xacro_path])]
    for argument_name, argument_value in (xacro_arguments or {}).items():
        xacro_command += [f' {argument_name}:=', argument_value]
    robot_description = Command(xacro_command)
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(robot_description, value_type=str) if description_as_string else robot_description,
        }],
    )


def joystick_node() -> Node:
    """조이스틱 입력."""
    return Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[{'deadzone': 0.05}],
    )


def rosbridge_nodes(max_message_size: Optional[int] = None) -> List[Node]:
    """rosbridge 웹소켓(9090)과 rosapi.

    max_message_size: 한 메시지의 최대 크기(바이트). None 이면 rosbridge 기본값을 쓴다 (M·MH 만 10 MB 로 올려 쓴다)
    """
    rosbridge_parameters = {'port': ROSBRIDGE_PORT}
    if max_message_size is not None:
        rosbridge_parameters['max_message_size'] = max_message_size
    return [
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            output='screen',
            parameters=[rosbridge_parameters],
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            output='screen',
        ),
    ]


def robot_pose_publisher_node() -> Node:
    """웹 관제가 로봇 위치로 쓰는 /footprint 를 TF(map/odom → base_footprint)에서 발행한다."""
    return Node(package='tetra_bringup', executable='robot_pose_publisher.py', name='robot_pose_publisher', output='screen')


def ekf_node(params_file_name: str) -> Node:
    """휠 오도메트리 + IMU 융합.

    params_file_name: tetra_bringup/params 안의 모델별 파일 이름 (예: 'ekf_m.yaml')
    """
    return Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('tetra_bringup'), 'params', params_file_name)],
        arguments=['--ros-args', '--log-level', 'error'],
        # 예전 launch 파일마다 주석으로 남아 있던 줄: remappings=[('/odometry/filtered', '/odom')]
    )


def usb_camera_include() -> IncludeLaunchDescription:
    """후면 usb_cam 을 로봇별 카메라 보정 파일로 띄운다 (usb_camera.launch.py)."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('tetra_bringup'), '/launch/usb_camera.launch.py']),
    )


def usb_camera_apriltag_include(undistort_corners: bool = False) -> IncludeLaunchDescription:
    """usb_cam 영상(/camera/image_raw)에서 AprilTag 를 찾는다 (apriltag_detection.launch.py).

    undistort_corners: 태그 모서리의 렌즈 왜곡을 보정한 뒤 자세를 구한다. M 후면 usb_cam 은 왜곡(k1 0.17)이 커서,
        보정 전 영상 그대로 자세를 구하면 태그가 중앙을 벗어날수록 yaw 가 휘어 후진 도킹 중 목표가 로봇 회전을 따라 돌았다 (2026-11-10 실기)
    """
    source = PythonLaunchDescriptionSource(
        [get_package_share_directory('apriltag_ros'), '/launch/apriltag_detection.launch.py'])
    if not undistort_corners:
        return IncludeLaunchDescription(source)
    return IncludeLaunchDescription(source, launch_arguments={'undistort_corners': 'true'}.items())


def usb_camera_with_apriltag(undistort_corners: bool = False) -> List[IncludeLaunchDescription]:
    """후면 usb_cam 과 그 영상의 AprilTag 검출. 같은 카메라를 쓰는 launch 가 왜곡 보정 여부를 한 인자로 정하게 묶었다."""
    return [usb_camera_include(), usb_camera_apriltag_include(undistort_corners)]


def realsense_apriltag_node(detector_parameters: Optional[Dict[str, object]] = None) -> Node:
    """RealSense 컬러 영상(/camera1/color/image_raw)에서 36h11, 0.1 m 태그를 찾는다.

    공용 apriltag_detection.launch.py 는 usb_cam(/camera/image_raw)을 구독해 RealSense 영상을 받지 못한다.
    전방 RealSense 로 도킹하는 모델(MH, SM, EDU)이 이 노드를 직접 띄운다.

    detector_parameters: 검출기 추가 파라미터 (예: {'detector.decimate': 1.0})
    """
    return Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[{'family': '36h11', 'size': 0.1, **(detector_parameters or {})}],
        remappings=[
            ('/image_rect', '/camera1/color/image_raw'),
            ('/camera_info', '/camera1/color/camera_info'),
        ],
    )
