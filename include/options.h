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
  std::string radar_topic = "/radar_points";

  std::string lidar_registered_topic = "/lidar_registered";
  std::string radar_registered_topic = "/radar_registered";

  std::vector<double> t_bl_vec = {0.0, 0.0, 0.0};
  std::vector<double> R_bl_vec = {0, 0, 1, 0, -1, 0, 1, 0, 0};

  std::vector<double> t_lr_vec = {0.11216, 0, -0.03737};
  std::vector<double> R_lr_vec = {0.1564345, 0.9876884, 0,          0, 0,
                                  1,         0.9876884, -0.1564345, 0};

  V3D t_bl = V3D::Zero();
  M3D R_bl = M3D::Identity();
  V3D t_lr = V3D::Zero();
  M3D R_lr = M3D::Identity();
};

Options LoadOptionsFromFile(const std::string &config_file);
} // namespace mapping

#endif // OPTIONS_H