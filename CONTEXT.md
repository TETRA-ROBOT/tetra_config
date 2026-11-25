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
8. **주행 BT 와 복구 동작**: Jazzy bt_navigator 는 `navigate_to_pose.bt_xml_filename` 을 읽지 않는다. BT 는 반드시 `default_nav_to_pose_bt_xml`·`default_nav_through_poses_bt_xml` 로 준다
   (예전 `~/ros2_ws/...` 경로는 무시돼 Nav2 기본 BT 의 Spin 1.57 rad·BackUp 0.3 m 복구가 돌았고, H 가 후진하다 뒤쪽 안테나가 부서졌다).
   EDU 를 뺀 모든 모델은 `behavior_trees/navigate_*_w_replanning_and_recovery.xml`(기본 BT 에서 Spin·BackUp 제거: costmap 초기화·대기만)을 쓴다. EDU 는 `_edu.xml`(BackUp 만 제거).
   H·MH·M·S·SM collision_monitor 는 정지(stop) 다각형과 감속 다각형을 따로 둔다 (H 만 라이다 자기검출 없음을 실기 확인, M·S·SM 은 미검증).
   H footprint 는 뒤쪽 안테나(7 cm)까지 넣어 뒤 끝이 -0.504 m 다. 장착물이 차체 밖으로 나오면 footprint 와 정지 다각형을 같이 늘린다.
9. **MH EKF·라이다 합치기** (2026-11-24 맵핑 품질 분석): `ekf_mh.yaml` 은 H 와 같게 바퀴 x·y 변화량(differential)만 받고 회전은 IMU 자이로가 맡는다(60 Hz).
   예전에는 바퀴 yaw 를 절대값으로 받고 `/odom` 공분산이 0 이라 EKF 가 바퀴 방향을 그대로 따라, 회전 때 미끄럼만큼 지도 바깥 벽이 0.3~0.5 m 어긋나고 이중으로 그려졌다.
   `dual_laser_merger` 는 MH 만 odom 움직임 보정(`fixed_frame: odom`)을 쓰는데 `tolerance` 가 TF 대기 시간이기도 해서 0.01 이면 늦은 EKF TF 마다 합친 스캔을 버렸다 → 0.1(짝짓기 벌점 표준값).
   바퀴 배율 보정(`~/.config/tetra/wheel_odom_scale.txt`, 관리자 "바퀴 오도메트리 보정")은 MH 에서 아직 하지 않았다.
10. **MH GPIO 컨트롤러는 기본으로 끈다** (`use_gpio_controller` 기본 false, 2026-11-24). 켜면 `tetra_gpio_controller` 가 출력 0~7 의 0→1 을
    리프트 L0·L8, 팬 90°·0°, 틸트 80°·0°, 팔 원점·접기 명령으로 쓰고 출력을 0 으로 되돌린다. 웹의 팬틸트 범위·팔 충돌 검사를 거치지 않으니 켤 때 주의.
11. **MH + RB10 로봇팔** (`use_rb10:=true`): `tetra_bringup/launch/rb10_arm.launch.py` 가 몸체+팔 URDF(`tetra_description` `tetra_mh/tetra_rb10.xacro`)로
   robot_state_publisher·ros2_control·MoveIt(SRDF `params/rb10/tetra_mh_rb10.srdf`)을 띄운다. 이때 MH 브링업은 자기 robot_state_publisher 를 띄우지 않고,
   joint_state_publisher 에는 몸체 URDF 만 넘긴다 (팔 관절을 0 으로 덮어써 TF 가 튀지 않게).
   Jazzy joint_state_publisher 는 파일을 받아도 `/robot_description` 을 늘 구독해 몸체+팔 URDF 로 갈아타므로 그 구독을 쓰지 않는 토픽으로 돌린다
   (안 하면 팔 관절 0 이 실제 값과 번갈아 나가 관제 3D 의 팔이 접혔다 폈다 했다). 예전에는 ~/rainbow test.launch.py 를 include 해 robot_state_publisher 가 둘이었다.
   팔 패키지(rbpodo_*)는 이 레포 밖이라 robot.env `TETRA_EXTRA_ROS_SETUP` 으로 불러온다 (README 3.4).
   Autonics 카메라 장착 위치는 `autonics_camera_mount`(gripper|pantilt|none). 리프트·팬틸트 관절은 값을 내는 드라이버가 없어 joint_state_publisher 가 낸다: `source_list: ['/module_joint_states']` 로 관제 API 가 `/lift_status`·서보 상태로 10 Hz 로 보내는 실제 값을 받고(TetraSingleManagement `module-joint-states.ts`), 받기 전·API 가 없을 때는 0 이다.
12. **RB10 팔 카메라 장애물 (MoveIt octomap)** (2026-11-25): `rb10_arm.launch.py` 의 `use_arm_perception`(기본 true)이 `params/rb10/sensors_3d.yaml` 을
    MoveItConfigsBuilder `.sensors_3d()` 로 move_group 에 넣는다. PointCloudOctomapUpdater 두 개: 팔 끝 Autonics `/camera/depth/points_filtered`(depth_image_proc → pcl_ros PassThrough `autonics_points_near_filter` 로 카메라 앞 0.25~2 m 만, 정렬 점군, subsample 4),
    RealSense `/camera1/depth/color/points`(rs_launch `pointcloud.enable:=true` 필요, 비정렬, subsample 16).
    팔 카메라(ToF)는 렌즈 앞 0.2 m 안에 거짓 점을 낸다: 접은 팔에서 장애물로 들어간 점의 58 % 가 그 안이었고, octomap 은 한 번 맞으면 칸을 막아 가만히 서 있어도 1분 만에 팔 길이 막혔다 (2026-11-25).
    드라이버 ToF 필터(`conf_filter_enable` 등 README 권장값)는 켜 보니 점이 프레임당 1만 개 넘게 늘어 더 나빠져 끈 채로 둔다. `octomap_frame: odom`(로봇이 움직여도 장애물이 끌려가지 않음), 해상도 0.05 m, max_range 2 m.
    자기 몸 거르기: 업데이터가 move_group 이 아는 URDF(몸체+팔) 충돌 모양마다 `padding_scale`·`padding_offset`(팔 카메라 0.06, RealSense 0.05 m)만큼 키워 그 안의 점과 그림자를 버린다.
    걸러진 점군은 `/move_group/{arm,body}_camera/filtered_points` 로 RViz 에서 본다. 예전에는 리프트·팬틸트 TF 가 늘 0 이라 실제 높이·각도와 달라 몸통·머리를 자기 몸으로 걸러 내지 못했고, 리프트 3 cm 차이로 팔 접기·홈 길이 막혔다 (2026-11-25). 지금은 11번의 실제 값으로 TF 를 낸다. API 가 내려가 있으면 다시 0 이 될 수 있다.
    octomap 은 planning scene 의 world 에 `<octomap>` 으로 들어가 `/check_state_validity`(관제 API 팔 충돌 검사)가 같이 본다 → API 409 `obstacle`.
    한 번 본 장애물은 나중 점군이 그 칸을 비워 볼 때까지 남는다. move_group `/clear_octomap`(std_srvs/Empty, 관제 로봇팔 창 "장애물 지우기")으로 비운다.
    **플러그인은 `ros-jazzy-moveit-ros-perception` 에 있다** (`moveit_ros_occupancy_map_monitor` 만으로는 없음). 없으면 런치가 경고만 남기고 sensors_3d 를 빼고 띄운다.
    CPU 가 모자라면 `use_arm_perception:=false` 또는 yaml 의 `max_update_rate`·`point_subsample` 을 조정한다.
13. **깊이 카메라 점구름** (2026-11-25):
   - **RealSense** (`/camera1/depth/color/points`, 프레임 `camera1_depth_optical_frame`): MH·H 브링업 인자 `realsense_pointcloud`(기본 true). rs_launch 인자는 `bringup_common.realsense_pointcloud_arguments`,
     rs_launch 가 받지 않는 decimation 배율은 `params/realsense_pointcloud.yaml`(config_file). 깊이 640x480@15 → decimation 4 = 160x120(최대 19,200 점), 색 없음(xyz), 4 m 넘는 깊이 버림.
     decimation 은 발행되는 깊이 영상도 160x120 으로 줄인다. 컬러(AprilTag 도킹)는 그대로. 메시지는 구독자가 있을 때만 만든다.
     S 는 켜지 않았다: `navigation_s.yaml` STVL 이 이미 realsense_cloud 를 구독해 켜는 순간 S 주행이 바뀐다 (실기 확인 뒤 켠다). EDU 는 일부러 끔, SM 은 깊이 자체가 꺼짐, M 은 RealSense 기본 꺼짐, 3D 브링업은 MID360 을 쓴다.
   - **MH Nav2** (`navigation_mh.yaml`): 지역·전역 costmap 에 `stvl_layer`(SpatioTemporalVoxelLayer, 관측 realsense_cloud).
     obstacle_layer(2D)에 넣지 않은 이유: 라이다 광선이 같은 칸을 지워 라이다 평면 밖 장애물(낮은 물건, 책상 상판)이 깜박인다.
     높이 0.1~2.0 m(odom 기준, 그 아래는 바닥), 표시 3 m, 지우는 절두체 0.4~3 m(D455 87°x58°), 복셀 5 cm·점 2 개 이상, decay 3 s(시야 안에서는 가속 5 로 약 1 s).
     전역에도 둔 이유: RPP 는 장애물 앞에서 멈추기만 하고 계획기는 전역 costmap 만 보므로, 전역에 없으면 같은 길을 다시 준다.
     collision_monitor 에는 넣지 않았다: 시간 필터가 없어 깊이 잡음 한 번에 급정지하고, 낮은 장애물은 RPP 충돌 검사(1 s 앞)가 costmap 으로 막는다.
     도킹: 카메라가 충전기 몸체를 장애물로 볼 수 있으나 도크 0.45 m 안은 충돌 검사에서 빠진다 (`dock_collision_threshold`). 실기에서 도킹을 다시 확인한다.
   - **Autonics 팔 카메라** (`/camera/depth/points`, 프레임 `camera_depth_frame`): MH 브링업 인자 `autonics_pointcloud`(기본 true, `use_autonics_rgbd` 일 때만).
     depth_image_proc `point_cloud_xyz_node` 가 `/camera/depth/image_rect_raw`(16UC1 mm)와 옆의 `/camera/depth/camera_info` 를 받는다 (image_transport 규칙이라 camera_info_rect 로 못 바꾼다. fx·cx 차이 0.2 % 안).
     `camera_depth_frame` 은 드라이버가 `camera_link` 아래 광학 프레임으로 static TF 를 낸다. 구독자가 없으면 깊이도 구독하지 않는다. MoveIt 장애물 인식이 쓴다.
     **Nav2 에는 넣지 않는다**: 팔 끝에 달려 팔과 같이 움직이고(팔 TF 지연만큼 점이 흔들림) 로봇 몸체·팔·짐을 장애물로 본다.
