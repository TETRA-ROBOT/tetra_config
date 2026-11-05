# tetra_config - AI Agent Context

## 개요
TETRA 로봇의 런치(Launch) 파일, 센서 융합(EKF), 2D/3D SLAM, 그리고 Nav2(내비게이션2) 파라미터를 집중 관리하는 설정 패키지 그룹입니다.

## 서브 패키지 맵

| 서브 패키지 | 역할 | 주요 파일 |
|---|---|---|
| `tetra_bringup` | 6종 로봇 모델별 메인 브링업 런치 및 EKF 필터 설정 | `launch/tetra_bringup_*.launch.py`, `params/ekf.yaml` |
| `tetra_navigation2` | Nav2 자율주행 스택 (BT Navigator, Costmap, Controller, Planner) | `launch/navigation_launch.py`, `params/nav2_params.yaml` |
| `tetra_cartographer` | Cartographer 기반 2D SLAM 런치 및 Lua 설정 | `launch/cartographer.launch.py`, `configuration_files/*.lua` |
| `tetra_rtabmap` | RTAB-Map 기반 3D Visual/LiDAR SLAM 및 메모리 관리 | `launch/tetra_rtabmap.launch.py` |

## 실행 흐름 다이어그램

```
  [ tetra_bringup ]
         │
         ├──> [ tetra_wheel_driver ] + [ tetra_power_driver ] (하드웨어 구동)
         ├──> [ robot_state_publisher ] (URDF TF 발행)
         ├──> [ ekf_filter_node ] (Odom + IMU 센서 융합)
         └──> [ rosbridge_websocket ] (웹 백엔드 통신 포트 9090)
```

## AI 에이전트 주의사항
1. **모델별 브링업**:
   - `tetra_bringup_m.launch.py`: 기본 TETRA-M 모델 (2D LiDAR)
   - `tetra_bringup_h.launch.py`: TETRA-H 대형 모델 (듀얼 LiDAR + Realsense)
   - `tetra_bringup_s.launch.py`: TETRA-S 3D 매핑 모델
2. **Nav2 수명주기(Lifecycle)**: `nav2_lifecycle_manager` 노드가 AMCL, Controller, Planner의 상태를 `unconfigured` -> `inactive` -> `active` 순으로 관리합니다.
3. **경로 지정 주행 Route Server**: `tetra_navigation2/launch/route_server_launch.py` 를 `navigation_launch.py` 가 포함한다 (설치돼 있을 때만).
   빈 그래프는 거부되므로 `graphs/route_placeholder.geojson` 으로 뜨고, 실제 그래프는 관제 API 가 `set_route_graph` 로 읽힌다.
   Nav2 본체와 다른 lifecycle manager(`lifecycle_manager_route`)라 Route Server 문제로 Nav2 기동이 멈추지 않는다. 파라미터: `params/route_server.yaml` (모든 모델 공통).
