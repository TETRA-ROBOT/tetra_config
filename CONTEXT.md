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
   - 모델과 상관없이 같은 노드(URDF 발행, joy, rosbridge·rosapi, 위치 발행, EKF, 후면 usb_cam + apriltag)는
     `tetra_bringup/launch/bringup_common.py` 의 함수로 만든다. 모델별 launch 파일에는 차이만 남긴다
     (센서, 직렬 포트 기본값, xacro 폴더·인자, EKF 파일, rosbridge 메시지 크기, 태그 모서리 왜곡 보정 여부).
     공통 블록을 launch 파일에 다시 복사하지 않는다: 한 곳만 고치고 나머지를 빠뜨린 적이 있다 (apriltag undistort_corners).
     launch 폴더의 모듈은 `tetra_serial_ports.py` 처럼 `sys.path` 에 그 폴더를 넣고 import 한다.
2. **Nav2 수명주기(Lifecycle)**: `nav2_lifecycle_manager` 노드가 AMCL, Controller, Planner의 상태를 `unconfigured` -> `inactive` -> `active` 순으로 관리합니다.
3. **경로 지정 주행 Route Server**: `tetra_navigation2/launch/route_server_launch.py` 를 `navigation_launch.py` 가 포함한다 (설치돼 있을 때만).
   빈 그래프는 거부되므로 `graphs/route_placeholder.geojson` 으로 뜨고, 실제 그래프는 관제 API 가 `set_route_graph` 로 읽힌다.
   Nav2 본체와 다른 lifecycle manager(`lifecycle_manager_route`)라 Route Server 문제로 Nav2 기동이 멈추지 않는다. 파라미터: `params/route_server.yaml` (모든 모델 공통).
4. **구간 비용 필터(PENALTY 구간)**: 실기 `params/navigation_{m,h,mh,s,sm,edu}.yaml` 의 global_costmap 에 `filters: ["zone_keepout_filter"]`(nav2_costmap_2d::KeepoutFilter,
   `filter_info_topic: /costmap_filter_info`)가 있다. 마스크(`/zone_filter_mask`)와 필터 정보는 관제 API 가 transient_local 로 보낸다 (TetraSingleManagement `lib/robot/zones/zone-filter-publisher.ts`).
   마스크 값(1~90%)이 비용이 되어 전역 경로가 그 구간을 되도록 피한다. 구간이 없으면 값이 모두 0 이라 동작이 예전과 같다.
   local_costmap 에는 넣지 않았다 (제어기까지 피하게 하면 좁은 곳에서 멈출 수 있다). 시뮬레이션 `tetra_lab/robot_gazebo_sim/params/navigation_sim.yaml` 에는 아직 없다.
5. **도킹 접근 속도**: `params/navigation_{m,h,mh,edu,sm}.yaml` docking_server `controller:` 의 `v_linear_max`·`v_linear_min`·`slowdown_radius` 가
   staging 이후 충전기까지의 속도를 정한다 (graceful: 남은 거리 + 0.25 m 에 비례해 줄인다). 기본 0.10 / 0.0385 / 0.65 = 도크 0.4 m 앞부터 줄여 약 0.04 m/s 로 닿는다.
   관제 API 가 도킹 목표마다 관리자 설정 '도킹 속도'·'감속 시작 거리'로 세 값을 덮어쓰므로, YAML 은 관제 없이 도킹할 때와 재시작 직후 첫 언도킹에만 쓰인다.
   언도킹 후진과 리프트 도킹도 같은 제어기 값을 쓴다. 도킹 중에는 docking_server 파라미터를 바꿀 수 없다 (서버 mutex, tetra_util CONTEXT.md).
6. **AMCL 과 자동 위치 재추정**: 모든 모델 `params/amcl_localization_*.yaml` 은 `likelihood_field_prob` 을 쓰고 무작위 파티클 주입(`recovery_alpha_*`)을 끈다
   (주입이 위치를 튀게 했다). 위치를 잃으면 `launch/auto_relocalization_launch.py`(모델별 `tetra_navigation_*.launch.py` 가 include)의
   `auto_relocalizer`(tetra_util/tetra_relocalization)가 스캔을 지도에 맞춰 바로잡는다. 그 `scan_topic` 은 AMCL 과 같게 준다 (H·MH `scan_merge`).
   AMCL YAML 에 `use_sim_time` 을 쓰지 않는다: params 파일 값이 런치의 `-p use_sim_time` 보다 우선해서 실로봇 AMCL 이 sim time 으로 뜬다.
7. **정지 중 EKF 방향 흐름 (M)**: `ekf_m.yaml` 의 imu0 는 `imu/data_gated` 다. 브링업이 띄우는 `imu_standstill_gate`(tetra_util/tetra_imu_gate, `bringup_common.imu_standstill_gate_node`)가
   바퀴가 서 있으면 IMU 를 막는다. 바퀴 정지 분산(1e-9)만으로는 process noise 때문에 자이로 영점 오차가 섞여 정지 중 -6°/h 흘렀다 (2026-11-20).
   다른 모델 EKF 도 IMU 회전 속도를 쓰지만 아직 막지 않는다. 넣을 때는 런치에 노드를 추가하고 imu0 를 바꾼다.
