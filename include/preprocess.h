/**
 * @file preprocess.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief Preprocessing functions for AgriLiRa4D
 * @version 0.1
 * @date 2026-07-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef PREPROCESS_H
#define PREPROCESS_H

#include "comm.h"

#include <pcl/common/transforms.h>

namespace mapping {

class Preprocess {
public:
  // Convert ROS PointCloud2 message to PCL PointCloud for RS_Airy LIDAR
  static void RSAiry2PCL(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         CloudPtr &cloud, int filter_num, double blind);

  // Convert ROS PointCloud2 message to PCL PointCloud for TXG Radar
  static void Radar2PCL(const sensor_msgs::PointCloud2::ConstPtr &msg,
                        RadarCloudPtr &cloud);

  // Undistort LiDAR points based on synchronized pose data
  static void UndistortPcl(MeasureGroup &meas, const M3D &R_bl,
                           const V3D &t_bl);

  static void TransformPointCloud(const CloudPtr &cloud, const M3D &r,
                                  const V3D &t, CloudPtr &output);

  static void TransformRadarPointCloud(const RadarCloudPtr &cloud, const M3D &r,
                                       const V3D &t, RadarCloudPtr &output);
};
} // namespace mapping

#endif // PREPROCESS_H