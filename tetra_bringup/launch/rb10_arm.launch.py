"""TETRA-MH 의 Rainbow RB10 로봇팔: robot_state_publisher(몸체+팔 URDF), ros2_control, MoveIt.

예전에는 ~/rainbow 의 rbpodo_moveit_config test.launch.py 를 그대로 include 했다. 그 런치는 ~/rainbow 안의 URDF(팔 + 예전 ~/ros2_ws
몸체)로 robot_state_publisher 를 하나 더 띄워, MH 브링업의 robot_state_publisher 와 같은 이름 노드가 둘 생겼고 카메라 장착 위치도
프로젝트에서 바꿀 수 없었다. 같은 노드를 이 프로젝트의 URDF(tetra_description tetra_mh/tetra_rb10.xacro)와 SRDF 로 띄운다.

팔 패키지(rbpodo_description·rbpodo_hardware·rbpodo_moveit_config·rbpodo_bringup)는 프로젝트 밖 워크스페이스에 있어
robot.env 의 TETRA_EXTRA_ROS_SETUP 으로 불러온다. 노드 구성·지연·재시작 값은 test.launch.py 와 같다.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# 컨트롤 박스가 팔을 깨우는 동안 ros2_control 하드웨어가 먼저 붙으면 실패해서 예전 런치도 10초 늦게 띄웠다
ROS2_CONTROL_START_DELAY_S = 10.0


def _arguments() -> list:
    return [
        DeclareLaunchArgument('robot_ip', default_value='10.0.2.7', description='RB 컨트롤 박스 IP'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false'),
        DeclareLaunchArgument('cb_simulation', default_value='Real', description='컨트롤 박스 모드 Real | Simulation'),
        DeclareLaunchArgument('model_id', default_value='rb10_1300e'),
        DeclareLaunchArgument('autonics_camera_mount', default_value='gripper', description='gripper | pantilt | none'),
        DeclareLaunchArgument('autonics_gripper_xyz', default_value='-0.0091 -0.0287 -0.1733', description='팔 끝단 원(tcp) 기준 카메라 메시 원점 (m). 카메라 중심이 위쪽 17.5 cm 에 온다'),
        DeclareLaunchArgument('autonics_gripper_rpy', default_value='3.1416 0 -1.5708', description='tcp 기준 카메라 자세 (rad)'),
        DeclareLaunchArgument('use_arm_perception', default_value='true',
                              description='깊이 카메라 점군으로 MoveIt octomap 을 만들어 팔 충돌 검사에 장애물을 넣는다 (params/rb10/sensors_3d.yaml). CPU 가 모자라면 false'),
    ]


def _perception_sensors(enabled: bool) -> tuple:
    """(sensors_3d yaml 경로 또는 None, 로그 문구). 플러그인 패키지가 없으면 move_group 이 플러그인 오류만 내고 장애물 없이 돌아서 미리 알린다"""
    if not enabled:
        return None, 'rb10_arm: use_arm_perception:=false, MoveIt 카메라 장애물(octomap)을 쓰지 않습니다'
    try:
        get_package_share_directory('moveit_ros_perception')
    except PackageNotFoundError:
        return None, ('rb10_arm: moveit_ros_perception 이 없어 카메라 장애물(octomap)을 쓰지 않습니다. '
                      'sudo apt install ros-jazzy-moveit-ros-perception 후 팔 런치를 다시 띄우세요')
    path = os.path.join(get_package_share_directory('tetra_bringup'), 'params', 'rb10', 'sensors_3d.yaml')
    return path, f'rb10_arm: MoveIt 카메라 장애물(octomap) 사용: {path}'


def _launch_setup(context, *args, **kwargs) -> list:
    mapping_names = ('robot_ip', 'use_fake_hardware', 'fake_sensor_commands', 'model_id', 'cb_simulation',
                     'autonics_camera_mount', 'autonics_gripper_xyz', 'autonics_gripper_rpy')
    mappings = {name: LaunchConfiguration(name).perform(context) for name in mapping_names}
    urdf_path = os.path.join(get_package_share_directory('tetra_description'), 'urdf', 'tetra_mh', 'tetra_rb10.xacro')
    srdf_path = os.path.join(get_package_share_directory('tetra_bringup'), 'params', 'rb10', 'tetra_mh_rb10.srdf')
    sensors_path, perception_message = _perception_sensors(LaunchConfiguration('use_arm_perception').perform(context).lower() in ('true', '1'))

    # 절대 경로를 주면 MoveItConfigsBuilder 가 패키지 경로와 합치지 않고 그대로 쓴다
    # sensors_3d 를 안 부르면 to_moveit_configs 가 rbpodo_moveit_config/config/sensors_3d.yaml 을 찾는데 없어서 장애물 없이 뜬다
    builder = (
        MoveItConfigsBuilder('rbpodo', package_name='rbpodo_moveit_config')
        .robot_description(file_path=urdf_path, mappings=mappings)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .robot_description_semantic(file_path=srdf_path)
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'])
    )
    if sensors_path:
        builder = builder.sensors_3d(file_path=sensors_path)
    moveit_config = builder.to_moveit_configs()
    controllers_path = os.path.join(get_package_share_directory('rbpodo_bringup'), 'config', 'test.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
        respawn=True,
        respawn_delay=2.0,
    )
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
        respawn=True,
        respawn_delay=2.0,
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, controllers_path],
        output='both',
        respawn=True,
        respawn_delay=5.0,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '3000', '--controller-manager', '/controller_manager'],
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
    )
    return [
        LogInfo(msg=perception_message),
        robot_state_publisher,
        move_group,
        TimerAction(period=ROS2_CONTROL_START_DELAY_S, actions=[ros2_control_node]),
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(_arguments() + [OpaqueFunction(function=_launch_setup)])
