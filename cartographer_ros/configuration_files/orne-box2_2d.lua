-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-----------------------------------------------------------------------------------------------
--GLOBAL SLAM SETTING
-----------------------------------------------------------------------------------------------

POSE_GRAPH.optimize_every_n_nodes = 90 --0にすると大域SLAMをオフにできる
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 --潜在的な制約に対する追加された制約の割合が、この設定値より低くなると、制約が追加されることになります。
POSE_GRAPH.constraint_builder.max_constraint_distance = 15 --サブマップと近い poses について、考慮する距離の閾値。
POSE_GRAPH.constraint_builder.min_score = 0.55 --スキャンマッチスコアの閾値で、これ以下ではマッチングが考慮されない。低いスコアは、スキャンとマップが似ていないことを示す。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6 --global localization を信頼しないようにするスコア閾値。
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4 --ループクロージャー制約の"並進"成分に対する最適化問題で使用される重み。
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5 --ループクロージャー制約の"回転"成分に対する最適化問題で使用される重み。
POSE_GRAPH.constraint_builder.log_matches = true --有効にすると、ループクロージャー制約の情報をデバッグログに出力します。

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7 --最適なスキャンアライメントを見つけるための最小の線形探索 Window。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.) --最適なスキャンアライメントが得られる最小の角度探索 Window。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7 --事前計算に使用するグリッドの数。

POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20 --各コスト functor のスケーリング値
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10 
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

POSE_GRAPH.matcher_translation_weight = 5e2 -- 5e2 非ループクロージャー制約の"並進"成分に対する最適化問題で使用される重み。
POSE_GRAPH.matcher_rotation_weight = 1.6e3 --  1.6e3 非ループクロージャー制約の"回転"成分に対する最適化問題で使用される重み。

POSE_GRAPH.optimization_problem.huber_scale = 1e1 --Huber loss function のスケーリング値
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2 --1.1e2 IMU acceleration 期間のスケーリング値
POSE_GRAPH.optimization_problem.rotation_weight = 1.4e3  --1.6e4 IMU 回転の期間のスケーリング値
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  --1e5 ローカル SLAM の pose に基づく連続したノード間の"並進"のためのスケーリング値。
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5 --1e5 ローカル SLAM の pose に基づく連続したノード間の"回転"のためのスケーリング値。
POSE_GRAPH.optimization_problem.odometry_translation_weight = 4.0e4 --2e4 オドメトリに基づく連続したノード間の"並進"のためのスケーリング値。
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 3.0e4  --3e4 オドメトリに基づく連続したノード間の"回転"のためのスケーリング値。
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1 --FixedFramePose の"並進"のためのスケーリング値。
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2 --FixedFramePose の"回転"のためのスケーリング値。
POSE_GRAPH.optimization_problem.fixed_frame_pose_use_tolerant_loss = false 
POSE_GRAPH.optimization_problem.fixed_frame_pose_tolerant_loss_param_a = 1
POSE_GRAPH.optimization_problem.fixed_frame_pose_tolerant_loss_param_b = 1
POSE_GRAPH.optimization_problem.log_solver_summary = false --有効なら、Ceres ソルバーのサマリーはすべての最適化についてログに記録します。
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true
POSE_GRAPH.optimization_problem.fix_z_in_3d = false

POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 7

POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.log_residual_histograms = true
POSE_GRAPH.global_constraint_search_after_n_seconds = 10

--POSE_GRAPH.overlapping_submaps_trimmer_2d.fresh_submaps_count = 1
--POSE_GRAPH.overlapping_submaps_trimmer_2d.min_covered_area = 2
--POSE_GRAPH.overlapping_submaps_trimmer_2d.min_added_submaps_count = 5

------------------------------------------------------------------------------------------------
--LOCAL SLAM SETTING
------------------------------------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 40
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 2
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) --20
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1 --1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10 --10 オドメトリとスキャンに関する直線移動の重み
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 140 --40 オドメトリとスキャンに関する回転移動の重み
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10

TRAJECTORY_BUILDER_2D.pose_extrapolator.use_imu_based = false
TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 10
TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.001

TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.pose_queue_duration = 5
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.gravity_constant = 9.806
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.pose_translation_weight = 1
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.pose_rotation_weight = 1
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.imu_acceleration_weight = 1
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.imu_rotation_weight = 1
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.odometry_translation_weight = 1
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.odometry_rotation_weight = 1
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.max_num_iterations = 10
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.num_threads = 1

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90 --90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.truncation_distance = 0.3
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.maximum_weight = 10
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_free_space = false
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.num_normal_samples = 4
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.sample_radius = 0.5

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.project_sdf_distance_to_scan_normal = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_range_exponent = 0
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 8

return options
