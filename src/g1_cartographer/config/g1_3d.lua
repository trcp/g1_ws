include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "livox_frame",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  
  -- 【重要】ロボットの推定姿勢から Pitch, Roll, Z を除去し、平面上の動きのみを出力する
  publish_frame_projected_to_2d = true, 
  
  use_pose_extrapolator = true,
  use_odometry = false, 
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
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

-- 3D モードを有効化
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4 

-- 3D トラジェクトリビルダーの設定
TRAJECTORY_BUILDER_3D.min_range = 0.3
TRAJECTORY_BUILDER_3D.max_range = 30.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true

-- 【重要】IMUの重力推定の時定数を大きくし、歩行時の高周波な衝撃（加速度）を無視する
-- デフォルト(10.0)よりも大きくすることで、長期的な平均重力（真下）を強く信用します
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 15.0

-- 【重要】ヒューマノイドの歩行時の揺れに対応するための最適化
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.
-- 回転（特にPitch/Roll）の変更に対するペナルティをさらに極端に高くし、傾きを防ぐ
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1000. 

TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 30.

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 50.

return options
