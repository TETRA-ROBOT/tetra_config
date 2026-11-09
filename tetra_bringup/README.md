# 🔧 Tetra Bringup

## 📖 개요
Tetra Bringup 패키지는 Tetra 로봇의 초기화 및 설정을 담당합니다. 이 패키지는 로봇의 기본 설정을 실행합니다.

## 의존성
- ament_cmake
- nav2_common
- navigation2
- launch_ros
- diff_drive_controller
- joint_state_broadcaster
- slam_toolbox
- xacro

## 설치 방법
1. ROS2 워크스페이스를 설정합니다.
2. `tetra_bringup` 디렉토리를 `src` 폴더에 복사합니다.
3. 워크스페이스 루트에서 다음 명령어를 실행합니다:
   ```bash
   colcon build --packages-select tetra_bringup
   ```

## 주요 파일 설명
### launch/
- **`tetra_bringup.launch.py`**: Tetra 로봇의 주요 노드들을 실행하는 launch 파일입니다. EKF 로컬라이제이션, 조이스틱, URDF, 서비스 노드 등을 포함합니다.

### params/
- **`ekf.yaml`**: EKF 로컬라이제이션을 위한 설정 파일로, 주로 로봇의 센서 데이터와 odometry 데이터를 통합하여 위치를 추정합니다.

## 주요 노드
- **`tetra_wheel_driver`**: TETRA 차륜 모터 드라이버 노드로, 로봇의 구동, 속도 명령 처리 및 오도메트리 데이터를 발행합니다.
- **`robot_localization`**: EKF 로컬라이제이션 노드로, odometry와 IMU 데이터를 통합하여 로봇의 위치를 추정합니다.
- **`tetra_power_driver`**: 전원 관리 보드 드라이버 노드로, 배터리 상태, 충전 제어, 범퍼, GPIO, LED 및 초음파/컨베이어 옵션을 제어합니다.
- **`iahrs_driver`**: IMU 센서 드라이버 노드로, IMU 데이터를 처리합니다.
- **`robot_state_publisher`**: 로봇의 URDF를 퍼블리시하여 TF 트리를 생성합니다.
- **`rosbridge_websocket`**: ROS와 웹 클라이언트 간의 통신을 위한 노드입니다.

### TETRA-M 센서 구성
- 전방 SICK TIM 5xx: `sick_scan_xd`로 전방 2D LiDAR를 실행합니다.
- 후방 Cyglidar D2: `cyglidar_d2_ros2`로 후방 2D LiDAR를 실행합니다.
- USB 카메라: `usb_cam`으로 카메라 영상을 발행합니다.
- AprilTag: `apriltag_ros`가 USB 카메라 영상에서 태그를 검출합니다.
- 가상벽: 별도 `virtual_wall` 패키지는 실행하지 않습니다. 관제 백엔드가 `/Global_virtual_wall` 및 `/Local_virtual_wall` PointCloud2 토픽을 직접 발행합니다.

### 모델별 런치 인자
- `tetra_bringup.launch.py`는 `robot_model`(m, h, edu, mh, s, sm)에 맞는 `tetra_bringup_<model>.launch.py`를 실행합니다. 지원하지 않는 값이면 오류로 종료합니다. 모델별 인자는 같은 명령줄에 함께 넘기면 됩니다.
- EKF는 모델별 `params/ekf_<model>.yaml`을 사용합니다.
- H/MH 듀얼 SICK

  | 인자 | 기본값 | 설명 |
  |---|---|---|
  | `front_lidar_hostname` | `192.168.0.1` | 전방 SICK IP |
  | `rear_lidar_hostname` | `192.168.0.2` | 후방 SICK IP |
  | `front_lidar_frame` | `laser` | 전방 SICK TF 프레임 (URDF의 전방 대각 장착 위치) |
  | `rear_lidar_frame` | `laser_link2` | 후방 SICK TF 프레임 (URDF의 후방 대각 장착 위치) |

  `/scan_front`, `/scan_rear`를 각각 `params/laser_shadow_filter.yaml`로 필터링해 `/scan_filtered_1`, `/scan_filtered_2`로 발행하고, `dual_laser_merger`가 `/scan_merge`로 병합합니다.
- SM은 EDU와 동일한 `kanavi_vl/R270`을 사용하며 `/points2 -> pointcloud_to_laserscan -> /scan -> line_extraction` 흐름입니다. URDF는 `urdf/tetra_sm/tetra.xacro`이고, 프레임은 이 URDF의 `lidar_link`(2D 라이다 장착 위치, -90도 회전)를 사용합니다. `camera1_link`는 사용하지 않습니다.
- MH는 `urdf/tetra_mh/tetra.xacro`(리프트·팬틸트 포함)로 `robot_state_publisher`를 실행합니다. H는 `urdf/tetraDS/tetra.xacro`를 사용합니다.
- MH 외부 장비

  | 사용 플래그 | 기본값 | 관련 인자 |
  |---|---|---|
  | `use_gpio_controller` | `true` | - |
  | `use_keepalive_relay` | `true` | - |
  | `use_realsense` | `true` | - |
  | `use_herkulex` | `true` | - |
  | `use_autonics_rgbd` | `true` | `autonics_interface`(`eno1`), `autonics_ip`(`192.168.0.200`), `autonics_uart`(`/dev/ai_camera`) |
  | `use_rb10` | `false` | `rb10_ip`(`10.0.2.7`) |
  | `use_thermal_camera` | `false` | `thermal_pipeline`(GStreamer RTSP 파이프라인, 기본 `192.168.0.100`) |

  예: `ros2 launch tetra_bringup tetra_bringup.launch.py robot_model:=mh use_herkulex:=false` (RB10은 기본 비활성, 필요 시 `use_rb10:=true`와 별도 설치한 `rbpodo_moveit_config` 필요)
- M, H, EDU, MH, SM은 현재 3D LiDAR를 사용하지 않습니다. Livox MID-360(`livox_ros_driver2`)은 `tetra_3d_bringup*.launch.py`에서만 사용하며, 현재 `tetra_3d_bringup.launch.py`는 S 모델 테스트용(`ekf_s.yaml`)입니다.
- Livox는 `launch_ROS2/MID360_launch.py`로 `/livox/lidar`에 PointCloud2를 발행합니다. 라이다/호스트 IP는 `livox_ros_driver2/config/MID360_config.json`에서 설정합니다.

## 사용 방법
1. Launch 파일 실행:
   ```bash
   ros2 launch tetra_bringup tetra_bringup.launch.py
   ```
2. EKF 로컬라이제이션 설정을 수정하려면 `params/ekf.yaml` 파일을 편집합니다.
3. 조이스틱 입력을 사용하려면 `joy_node`를 활성화합니다.

## EKF 설정 예시
`params/ekf.yaml` 파일의 주요 설정:
- **`frequency`**: EKF 업데이트 주기 (기본값: 50.0 Hz).
- **`map_frame`**: 맵 프레임 이름 (기본값: `map`).
- **`odom_frame`**: 오도메트리 프레임 이름 (기본값: `odom`).
- **`base_link_frame`**: 로봇의 기본 링크 프레임 이름 (기본값: `base_footprint`).
- **`imu0`**: IMU 데이터 토픽 이름 (기본값: `imu/data`).