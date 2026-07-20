/**
 * @file mapping.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief mapping for AgriLiRa4D
 * @version 0.1
 * @date 2026-07-18
 *
 * @copyright Copyright (c) 2026
 *
 */

 
#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include "comm.h"
#include "options.h"
#include "preprocess.h"

namespace mapping {
class Mapping {
public:
  Mapping(ros::NodeHandle &nh, const Options &options = Options());
  ~Mapping();

  void LidarCBK(const sensor_msgs::PointCloud2ConstPtr &msg);
  void RadarCBK(const sensor_msgs::PointCloud2ConstPtr &msg);
  void ImuCBK(const sensor_msgs::ImuConstPtr &msg);

private:
  ros::NodeHandle nh_;
  Options options_;

  ros::Subscriber sub_lidar_;
  ros::Subscriber sub_radar_;
  ros::Subscriber sub_imu_;
  ros::Publisher pub_lidar_registered_;
  ros::Publisher pub_radar_registered_;
};
} // namespace mapping

#endif // MAPPING_H