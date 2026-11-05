# 경로 지정 주행용 Nav2 Route Server.
#
# 관제 API 가 지도별 경로 그래프(GeoJSON)를 만들어 /route_server/set_route_graph 로 읽히고,
# compute_route(노드 ID) 로 받은 경로를 FollowPath 로 보낸다. 그래서 Route Server 는 그래프 없이 떠 있어야 한다.
#
# - 빈 그래프는 Route Server 가 거부해 lifecycle 전환이 실패하므로, 쓰이지 않는 먼 곳의 자리표시 그래프로 띄운다.
#   API 는 실제 그래프를 읽히기 전까지 경로 모드 이동을 거부한다.
# - Nav2 본체와 다른 lifecycle manager 를 쓴다. 그래프·Route Server 문제로 Nav2 전체 기동이 멈추지 않게 하려는 것이다.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('tetra_navigation2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_route_server = LaunchConfiguration('use_route_server')
    route_params_file = LaunchConfiguration('route_params_file')
    placeholder_graph = os.path.join(share_dir, 'graphs', 'route_placeholder.geojson')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_route_server', default_value='true',
            description='Start the Nav2 route server used for route-guided driving'),
        DeclareLaunchArgument(
            'route_params_file',
            default_value=os.path.join(share_dir, 'params', 'route_server.yaml')),
        GroupAction(
            condition=IfCondition(use_route_server),
            actions=[
                Node(
                    package='nav2_route',
                    executable='route_server',
                    name='route_server',
                    output='screen',
                    parameters=[
                        route_params_file,
                        {'graph_filepath': placeholder_graph, 'use_sim_time': use_sim_time},
                    ],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_route',
                    output='screen',
                    parameters=[{
                        'autostart': True,
                        'node_names': ['route_server'],
                        'use_sim_time': use_sim_time,
                    }],
                ),
            ],
        ),
    ])
