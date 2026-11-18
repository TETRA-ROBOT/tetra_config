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
    pkg_rviz_config = get_package_share_directory('tetra_navigation2')

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
        'amcl_localization_mh.yaml'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'params',
        'navigation_mh.yaml'
    )

    # 가상벽은 API 가 발행하는 /Local_virtual_wall, /Global_virtual_wall 을 costmap 장애물 레이어가 직접 읽는다.
    # 없는 virtual_wall 패키지를 찾으면 런치 생성 단계에서 예외가 나서 내비게이션이 뜨지 않았다.

    #map_file_path = PathJoinSubstitution([
    #    pkg_bme_ros2_navigation, 'maps', LaunchConfiguration('map_name')
    #])
    
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

    
    rviz_config_file = os.path.join(pkg_rviz_config, 'rviz', 'tetra_navigation2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # AMCL 이 위치를 잃으면 스캔을 지도에 맞춰 바로잡는다 (auto_relocalize:=false 로 끈다). scan_topic 은 AMCL 과 같게
    auto_relocalization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tetra_navigation2'), 'launch', 'auto_relocalization_launch.py')),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': 'scan_merge',
        }.items()
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)
    launchDescriptionObject.add_action(auto_relocalization_launch)
    #launchDescriptionObject.add_action(rviz_node)

    return launchDescriptionObject
