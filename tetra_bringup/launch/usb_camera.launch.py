#!/usr/bin/env python3
"""후면 usb_cam 을 로봇별 보정 파일로 띄운다 (usb_cam camera.launch.py 와 같은 노드·리매핑).

usb_cam 기본 설정은 소스에 든 공용 보정값(package://usb_cam/config/camera_info.yaml)을 읽는다. 웹관제
관리자 화면의 카메라 캘리브레이션이 set_camera_info 로 저장하면 그 파일이 덮여 모든 로봇에 같은 값이 들어가므로,
PERSISTENT_DIR/camera_info/rear_camera.yaml 을 쓰게 한다. 파일이 없으면 공용 기본값을 복사해 시작한다.
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

CAMERA_NAME = 'camera'
CALIBRATION_FILE_NAME = 'rear_camera.yaml'


def robot_calibration_url(default_file: str) -> str:
    persistent_dir = os.environ.get('PERSISTENT_DIR', '/home/tetra/tetra_integration/persistent')
    calibration_file = os.path.join(persistent_dir, 'camera_info', CALIBRATION_FILE_NAME)
    try:
        if not os.path.isfile(calibration_file):
            os.makedirs(os.path.dirname(calibration_file), exist_ok=True)
            shutil.copyfile(default_file, calibration_file)
        return 'file://' + calibration_file
    except OSError:
        # 저장소에 못 쓰면 카메라가 아예 안 뜨는 것보다 공용 기본값으로 뜨는 편이 낫다 (캘리브레이션 저장만 거부된다)
        return 'package://usb_cam/config/camera_info.yaml'


def generate_launch_description() -> LaunchDescription:
    usb_cam_dir = get_package_share_directory('usb_cam')
    params_file = os.path.join(usb_cam_dir, 'config', 'params_1.yaml')
    calibration_url = robot_calibration_url(os.path.join(usb_cam_dir, 'config', 'camera_info.yaml'))
    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=CAMERA_NAME,
        output='screen',
        parameters=[params_file, {'camera_info_url': calibration_url}],
        remappings=[
            ('image_raw', f'{CAMERA_NAME}/image_raw'),
            ('image_raw/compressed', f'{CAMERA_NAME}/image_compressed'),
            ('image_raw/compressedDepth', f'{CAMERA_NAME}/compressedDepth'),
            ('image_raw/theora', f'{CAMERA_NAME}/image_raw/theora'),
            ('camera_info', f'{CAMERA_NAME}/camera_info'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )
    return LaunchDescription([LogInfo(msg=f'usb_cam calibration: {calibration_url}'), camera])
