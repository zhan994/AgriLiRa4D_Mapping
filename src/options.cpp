#include "options.h"

namespace mapping {

Options LoadOptionsFromFile(const std::string &config_file) {
  auto config = YAML::LoadFile(config_file);

  Options options;

  options.verbose = config["verbose"].as<bool>();

  options.pose_gt_file = config["pose_gt_file"].as<std::string>();
  options.lidar_topic = config["lidar_topic"].as<std::string>();
  options.radar_topic = config["radar_topic"].as<std::string>();

  options.t_bl_vec = config["extrinsics"]["t_bl"].as<std::vector<double>>();
  options.R_bl_vec = config["extrinsics"]["R_bl"].as<std::vector<double>>();
  options.t_lr_vec = config["extrinsics"]["t_lr"].as<std::vector<double>>();
  options.R_lr_vec = config["extrinsics"]["R_lr"].as<std::vector<double>>();

  options.t_bl =
      V3D(options.t_bl_vec[0], options.t_bl_vec[1], options.t_bl_vec[2]);
  options.R_bl << options.R_bl_vec[0], options.R_bl_vec[1], options.R_bl_vec[2],
      options.R_bl_vec[3], options.R_bl_vec[4], options.R_bl_vec[5],
      options.R_bl_vec[6], options.R_bl_vec[7], options.R_bl_vec[8];
  options.t_lr =
      V3D(options.t_lr_vec[0], options.t_lr_vec[1], options.t_lr_vec[2]);
  options.R_lr << options.R_lr_vec[0], options.R_lr_vec[1], options.R_lr_vec[2],
      options.R_lr_vec[3], options.R_lr_vec[4], options.R_lr_vec[5],
      options.R_lr_vec[6], options.R_lr_vec[7], options.R_lr_vec[8];

  options.R_br = options.R_bl * options.R_lr;
  options.t_br = options.R_bl * options.t_lr + options.t_bl;

  options.lidar_filter_num = config["preprocess"]["lidar_filter_num"].as<int>();
  options.lidar_blind = config["preprocess"]["lidar_blind"].as<double>();
  options.pose_num_threshold =
      config["preprocess"]["pose_num_threshold"].as<int>();

  options.mapper_type = config["mapper"]["type"].as<int>();
  if (config["mapper"]["publish_period"]) {
    options.map_publish_period =
        config["mapper"]["publish_period"].as<double>();
  }
  if (options.mapper_type == 0) {
    options.octomap_options.resolution =
        config["mapper"]["octomap"]["resolution"].as<double>();
    options.octomap_options.max_range =
        config["mapper"]["octomap"]["max_range"].as<double>();
    options.octomap_options.hit_prob =
        config["mapper"]["octomap"]["hit_prob"].as<double>();
    options.octomap_options.miss_prob =
        config["mapper"]["octomap"]["miss_prob"].as<double>();
    options.octomap_options.occupancy_threshold =
        config["mapper"]["octomap"]["occupancy_threshold"].as<double>();
  }

  return options;
}
} // namespace mapping
