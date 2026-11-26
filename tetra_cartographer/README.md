# 🗺️ Tetra Cartographer

## 📖 개요
`tetra_cartographer` 패키지는 Tetra 로봇의 SLAM(Simultaneous Localization and Mapping)을 위한 Cartographer 설정 및 실행 파일을 제공합니다. 이 패키지는 2D SLAM을 지원하며, 로봇의 라이다 데이터를 활용하여 실시간으로 지도를 생성합니다.

## 의존성
- ament_cmake
- cartographer_ros

## 설치 방법
1. ROS2 워크스페이스를 설정합니다.
2. `tetra_cartographer` 디렉토리를 `src` 폴더에 복사합니다.
3. 워크스페이스 루트에서 다음 명령어를 실행합니다:
   ```bash
   colcon build --packages-select tetra_cartographer
   ```

## 📂 주요 파일 설명
### config/
- **`tetra_lds_2d.lua`**: Cartographer의 2D SLAM 설정 파일로, 라이다 데이터 처리 및 지도 생성에 필요한 매개변수를 정의합니다.
- **`tetra_lds_2d(old).lua`**: 이전 버전의 Cartographer 설정 파일로, 참고용으로 제공됩니다.
- **`tetra_edu_2d.lua`**: TETRA-EDU 웹 관제 매핑용 (Kanavi R270 `/points2`).

### 웹 관제 매핑 (`launch/cartographer_mapping.launch.py`)
웹 관제는 관리자 옵션 `slam_engine` 이 `cartographer` 일 때 이 런치로 매핑합니다 (EDU 기본값). EDU 는 `tetra_edu_2d.lua`,
나머지 모델은 2D 스캔 하나를 받는 `tetra_lds_2d.lua` 를 씁니다 (H·MH 는 `scan_topic:=/scan_merge`). 오도메트리는 EKF 출력(`/odometry/filtered`)으로 바꿔 받습니다.
SLAM Toolbox 와 지도를 비교하려면 루트의 `scripts/slam_compare.sh` 를 쓰세요.

### launch/
- **`cartographer_mapping.launch.py`**: 웹 관제가 쓰는 Cartographer + Occupancy Grid (RViz·고정 TF 없음). 인자 `configuration_basename`, `scan_topic`.
- **`cartographer_<model>.launch.py`**: 손으로 띄울 때. 위 런치를 모델 설정으로 include 하고 `use_rviz:=true`(기본)면 `rviz/tetra_cartographer_<model>.rviz` 를 띄운다.
- **`occupancy_grid_<model>.launch.py`**: Occupancy Grid 노드만 (예전 방식, 위 런치에 이미 들어 있음).

### rviz/
- **`tetra_cartographer_<model>.rviz`**: 모델별 RViz 설정 파일로, SLAM 결과를 시각화하는 데 사용됩니다.

## 주요 노드
- **`cartographer_node`**: Cartographer의 메인 노드로, 라이다 데이터를 처리하여 지도를 생성합니다.
- **`cartographer_occupancy_grid_node`**: Occupancy Grid를 생성하고 퍼블리시하는 노드입니다.

## 🚀 실행 방법
1. SLAM + Occupancy Grid + RViz 실행 (모델 접미사):
   ```bash
   ros2 launch tetra_cartographer cartographer_m.launch.py              # 화면 없는 로봇 PC: use_rviz:=false
   ```
2. 지도 저장: 웹 관제의 '지도 저장' 또는 `ros2 run nav2_map_server map_saver_cli -f <이름>`.

## 주요 설정
### `tetra_lds_2d.lua`
- **`map_frame`**: 생성된 지도의 프레임 이름 (기본값: `map`).
- **`tracking_frame`**: 로봇의 추적 프레임 이름 (기본값: `base_footprint`).
- **`published_frame`**: 퍼블리시되는 프레임 이름 (기본값: `odom`).
- **`TRAJECTORY_BUILDER_2D.min_range`**: 라이다의 최소 범위 (기본값: 0.05m).
- **`TRAJECTORY_BUILDER_2D.max_range`**: 라이다의 최대 범위 (기본값: 20.0m).
- **`TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching`**: 온라인 스캔 매칭 사용 여부 (기본값: `true`).

## 참고
Cartographer에 대한 자세한 정보는 [Cartographer 공식 문서](https://google-cartographer-ros.readthedocs.io/)를 참조하세요.