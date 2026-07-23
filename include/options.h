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

  std::vector<double> extrinsic_T = {0.00425, 0.00418, -0.00446};
  std::vector<double> extrinsic_R = {0.0, -1.0, 0.0, -1.0, 0.0,
                                     0.0, 0.0,  0.0, -1.0};
  V3D t_flu_imu = V3D::Zero();
  M3D R_flu_imu = M3D::Identity();
};

Options LoadOptionsFromFile(const std::string &config_file);
} // namespace mapping

#endif // OPTIONS_H