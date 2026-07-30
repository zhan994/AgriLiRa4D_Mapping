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

#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "comm.h"
#include "mapper/mapper.h"
#include "mapper/mapper_octomap.h"
#include "options.h"
#include "preprocess.h"

namespace mapping {

struct MapperUpdateStats {
  double wall_time_ms = 0.0;
  double thread_cpu_time_ms = 0.0;
  double thread_cpu_utilization_percent = 0.0;
  double process_cpu_utilization_percent = 0.0;
  std::int64_t rss_bytes = -1;
  std::size_t lidar_points = 0;
  std::size_t radar_points = 0;
};

struct DataCache {
  double last_lidar_time = -1.0;
  double last_radar_time = -1.0;

  // buffers
  std::mutex mtx; // mutex for data buffer

  // LiDAR
  std::deque<std::pair<double, CloudPtr>> lidar_buffer; // origin frame
  std::deque<PointStamped> lidar_points_stamped_buffer; // points with timestamp
  // RADAR
  std::deque<std::pair<double, RadarCloudPtr>> radar_buffer;
  // Pose
  std::deque<Pose6D> pose_buffer;

  nav_msgs::Path path;
};

class Mapping {
public:
  Mapping(ros::NodeHandle &nh, const Options &options = Options());
  ~Mapping();

private:
  void LoadPoseGT(const std::string &pose_gt_file);
  void LidarCBK(const sensor_msgs::PointCloud2ConstPtr &msg);
  void RadarCBK(const sensor_msgs::PointCloud2ConstPtr &msg);

  void Run();
  bool SyncGroup();
  void Process();
  void PublishOctomap(const ros::Time &stamp);
  void PublishUpdateStats(const MapperUpdateStats &stats);

  ros::NodeHandle nh_;
  Options options_;

  // cache for data
  DataCache data_;
  MeasureGroup sync_data_;

  // thread for mapping
  std::shared_ptr<std::thread> run_thread_;

  // mapping backend
  std::unique_ptr<Mapper> mapper_;
  ros::Time last_map_publish_stamp_;
  bool has_published_octomap_ = false;
  double last_process_cpu_time_ms_ = 0.0;
  double last_process_wall_time_ms_ = 0.0;

  // cloud_aft_mapped
  CloudPtr lidar_aft_mapped_;
  RadarCloudPtr radar_aft_mapped_;

  // subscribers
  ros::Subscriber sub_lidar_;
  ros::Subscriber sub_radar_;

  // publishers
  ros::Publisher pub_odom_;
  ros::Publisher pub_path_;
  ros::Publisher pub_lidar_aft_mapped_;
  ros::Publisher pub_radar_aft_mapped_;
  ros::Publisher pub_octomap_;
  ros::Publisher pub_update_stats_;
};
} // namespace mapping

#endif // MAPPING_H
