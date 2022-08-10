include "turtlebot_sim.lua"

TRAJECTORY_BUILDER.pure_localization = true
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--  max_submaps_to_keep = 3,
-- }
POSE_GRAPH.optimize_every_n_nodes = 2
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2

MAP_BUILDER.num_background_threads=4

-- from cartographer_magazino localization.lua
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 
  2 * TRAJECTORY_BUILDER_2D.submaps.num_range_data

POSE_GRAPH.global_constraint_search_after_n_seconds = 60

POSE_GRAPH.constraint_builder.min_score = .4
POSE_GRAPH.constraint_builder.global_localization_min_score = .35
POSE_GRAPH.constraint_builder.max_constraint_distance = 2.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(5.0)


return options
