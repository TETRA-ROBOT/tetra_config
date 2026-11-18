import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_navigation = get_package_share_directory('tetra_navigation2')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Flag to enable use_sim_time'
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='office.yaml',
        description='Name of the map file to use'
    )


    # Path to the Slam Toolbox launch file
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'launch',
        'navigation_launch.py'
    )

    localization_params_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'params',
        'amcl_localization_m.yaml'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'params',
        'navigation_m.yaml'
    )

    # 가상벽은 API 가 발행하는 /Local_virtual_wall, /Global_virtual_wall 을 costmap 장애물 레이어가 직접 읽는다.
    # 없는 virtual_wall 패키지를 찾으면 런치 생성 단계에서 예외가 나서 내비게이션이 뜨지 않았다.

    #map_file_path = os.path.join(
    #    get_package_share_directory('tetra_navigation2'),
    #    'maps',
    #    'office.yaml'
    #)
    
    persistent_maps_dir = os.environ.get('TETRA_MAPS_DIR', '/home/tetra/tetra_integration/persistent/maps')
    # Dynamically constructed paths
    map_file_path = PathJoinSubstitution([
        persistent_maps_dir, LaunchConfiguration('map_name')
    ])

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
                'map': map_file_path,
        }.items()
    )


    auto_relocalize_arg = DeclareLaunchArgument(
        'auto_relocalize', default_value='true',
        description='AMCL 이 위치를 잃으면 스캔을 지도에 맞춰 찾은 자세로 바로잡는다 (tetra_util/tetra_relocalization)'
    )

    # AMCL 의 무작위 파티클 주입(recovery_alpha)은 위치를 튀게 해서 껐다 (amcl_localization_m.yaml).
    # 대신 이 노드가 스캔이 지도와 맞는 비율을 재다가, 확실히 더 잘 맞고 헷갈리는 후보가 없는 자세일 때만 /initialpose 를 보낸다
    auto_relocalizer = Node(
        package='tetra_relocalization',
        executable='auto_relocalizer',
        name='auto_relocalizer',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('auto_relocalize')),
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(auto_relocalize_arg)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)
    launchDescriptionObject.add_action(auto_relocalizer)

    return launchDescriptionObject
