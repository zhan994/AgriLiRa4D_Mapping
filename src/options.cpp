#include "options.h"

#include <stdexcept>

namespace mapping {

Options LoadOptionsFromFile(const std::string &config_file) {
  const auto config = YAML::LoadFile(config_file);

  Options options;

  options.verbose = config["verbose"].as<bool>();

  options.pose_gt_file = config["pose_gt_file"].as<std::string>();
  options.lidar_topic = config["lidar_topic"].as<std::string>();
  options.radar_topic = config["radar_topic"].as<std::string>();

  const auto extrinsics = config["extrinsics"];
  options.t_bl_vec = extrinsics["t_bl"].as<std::vector<double>>();
  options.R_bl_vec = extrinsics["R_bl"].as<std::vector<double>>();
  options.t_lr_vec = extrinsics["t_lr"].as<std::vector<double>>();
  options.R_lr_vec = extrinsics["R_lr"].as<std::vector<double>>();

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

  const auto preprocess = config["preprocess"];
  options.lidar_filter_num = preprocess["lidar_filter_num"].as<int>();
  options.lidar_blind = preprocess["lidar_blind"].as<double>();

  const auto sync = config["sync"];
  if (const auto pose_num_threshold = sync["pose_num_threshold"]) {
    const int value = pose_num_threshold.as<int>();
    if (value <= 0)
      throw std::invalid_argument("sync.pose_num_threshold must be positive");
    options.sync_options.pose_num_threshold = value;
  }
  if (const auto output_queue_capacity = sync["output_queue_capacity"]) {
    const int value = output_queue_capacity.as<int>();
    if (value <= 0)
      throw std::invalid_argument(
          "sync.output_queue_capacity must be positive");
    options.sync_options.output_queue_capacity = value;
  }

  const auto mapper = config["mapper"];
  options.mapper_type = mapper["type"].as<int>();
  if (const auto publish_period = mapper["publish_period"]) {
    options.map_publish_period = publish_period.as<double>();
  }
  if (const auto timing_csv = mapper["timing_csv"]) {
    options.mapper_timing_csv = timing_csv.as<std::string>();
  }
  if (options.mapper_type != 0 && options.mapper_type != 1)
    throw std::invalid_argument("Unsupported mapper type");
  options.mapper_options.resolution = mapper["resolution"].as<double>();
  options.mapper_options.max_range = mapper["max_range"].as<double>();
  options.mapper_options.hit_prob = mapper["hit_prob"].as<double>();
  options.mapper_options.miss_prob = mapper["miss_prob"].as<double>();
  options.mapper_options.occupancy_threshold =
      mapper["occupancy_threshold"].as<double>();

  return options;
}
} // namespace mapping
