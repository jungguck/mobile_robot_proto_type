include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",    -- 선생님 로봇 몸통 이름
  published_frame = "odom",        -- 카토그래퍼가 맞추려는 위치
  odom_frame = "odom",             -- 로봇이 주는 오돔 이름
  provide_odom_frame = false,      -- 오돔은 로봇(diff_drive)이 주니까 넌 만들지 마
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,             -- 선생님 로봇은 오돔이 있으니까 true!
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,             -- 라이다 1개
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 라이다 설정 (2D)
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0   -- 라이다 최대 거리 (적당히 조절)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 2D SLAM에서는 IMU 없어도 됨
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- IMU 가 없으므로 자세의 초기 추정값이 "바퀴 오도메트리 yaw" 하나뿐이다.
-- 그런데 diff drive 는 회전할 때 바퀴가 옆으로 긁혀서 그 yaw 가 잘 틀어진다.
-- rotation_weight 는 "스캔 정합 결과가 초기 추정 각도에서 벗어나는 것"에 대한
-- 벌점이다. 기본 40 은 초기 추정을 꽤 믿는 값이라, 바퀴 yaw 가 튀면
-- 스캔이 맞다고 말해도 잘 안 따라간다. 낮춰서 스캔 쪽 손을 들어준다.
-- (되돌리려면 40. 으로. 반대로 스캔이 헛도는 환경이면 다시 올릴 것)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.

return options