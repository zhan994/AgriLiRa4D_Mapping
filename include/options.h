/**
 * @file options.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief common options for AgriLiRa4D
 * @version 0.1
 * @date 2026-07-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef OPTIONS_H
#define OPTIONS_H

#include "comm.h"

namespace mapping {
struct Options {
  Options() {}

  std::string pose_gt_file = "pose_gt.txt";

  std::string lidar_topic = "/rslidar_points";
  std::string imu_topic = "/imu/data";
  std::string radar_topic = "/radar_points";
  std::string lidar_registered_topic = "/lidar_registered";
  std::string radar_registered_topic = "/radar_registered";
};

Options LoadOptionsFromFile(const std::string &config_file);
} // namespace mapping

#endif // OPTIONS_H