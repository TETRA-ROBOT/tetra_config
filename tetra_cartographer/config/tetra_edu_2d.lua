-- TETRA-EDU 2D 매핑용 Cartographer 설정 (웹 관제 매핑: launch/cartographer_mapping.launch.py)
--
-- 예전 EDU 작업공간(~/ros2_ws/src/tetra_cartographer/config/tetra_lds_2d.lua)에서 실기로 쓰던 값을 그대로 옮겼다.
-- 공용 slam_toolbox 로 바꾼 뒤 EDU 지도가 회전 구간에서 기울고 매핑 끝에 시작점(home)이 옆으로 밀려서 되돌렸다 (2026-11-10).
--  - 입력: Kanavi R270 의 /points2 를 그대로 받는다 (num_point_clouds = 1, 토픽 이름 points2).
--    R270 클라우드는 lidar_link 기준 z 가 모두 0 인 평면이라(실측) 바닥이 장애물로 들어오지 않는다.
--  - odom: EKF 출력 /odometry/filtered (런치에서 /odom 을 바꿔 연결). IMU 는 EDU 에 없다.
--  - 공용 tetra_lds_2d.lua 는 /scan 1개를 받도록 바뀌어 있어 다른 모델용으로 두고, EDU 는 이 파일을 쓴다.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 0.2,
  fixed_frame_pose_sampling_ratio = 0.5,
  imu_sampling_ratio = 0.1,
  landmarks_sampling_ratio = 0.5,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.53
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.493
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 웹 관제의 매핑 정확도 표시는 이 실시간 상관 스캔 매칭의 점수(read_metrics)를 쓴다. 끄면 정확도가 나오지 않는다
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2

POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options
