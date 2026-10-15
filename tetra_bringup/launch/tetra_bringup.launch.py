#!/usr/bin/env python3
"""
TETRA Unified Bringup Launch

robot_model 인자에 맞는 모델별 런치(tetra_bringup_<model>.launch.py)를 실행합니다.
모델별 런치가 실제 센서 구성의 단일 출처이므로 이 파일은 노드를 직접 정의하지 않습니다.
사용 예시:
  ros2 launch tetra_bringup tetra_bringup.launch.py robot_model:=m
  ros2 launch tetra_bringup tetra_bringup.launch.py robot_model:=mh use_rb10:=false
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SUPPORTED_MODELS = ('m', 'h', 'edu', 'mh', 's', 'sm')


def include_model_launch(context: LaunchContext) -> List[IncludeLaunchDescription]:
    robot_model = LaunchConfiguration('robot_model').perform(context).lower()
    # 잘못된 모델명이면 아무 노드도 뜨지 않은 채 정상 종료된 것처럼 보이므로 즉시 실패시킨다.
    if robot_model not in SUPPORTED_MODELS:
        raise RuntimeError(
            f"지원하지 않는 robot_model '{robot_model}' (지원: {', '.join(SUPPORTED_MODELS)})")

    model_launch_path = os.path.join(
        get_package_share_directory('tetra_bringup'), 'launch', f'tetra_bringup_{robot_model}.launch.py')
    # 모델별 추가 인자(use_rb10 등)는 명령줄에서 주면 하위 런치에도 그대로 전달된다.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(model_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='m',
            description=f"로봇 모델 선택 ({', '.join(SUPPORTED_MODELS)})",
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='시뮬레이션 시간 사용 여부',
        ),
        OpaqueFunction(function=include_model_launch),
    ])
