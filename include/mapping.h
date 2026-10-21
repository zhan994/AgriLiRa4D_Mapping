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
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include "comm.h"
#include "mapper/mapper.h"
#include "mapper/mapper_octomap.h"
#include "options.h"
#include "preprocess.h"
#include "sync.h"

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

class Mapping {
public:
  Mapping(ros::NodeHandle &nh, const Options &options = Options());
  ~Mapping();

  // Constructor initializes resources and starts workers before enabling subscriptions.
  // Shut down subscribers, stop synchronizer, drain queued groups and join Run().
  // Call after ROS callbacks have quiesced; destructor also calls Stop().
  // Repeated calls are allowed. Never called from Run() or a subscriber callback.
  void Stop();

private:
  void LoadPoseGT(const std::string &pose_gt_file);
  void LidarCBK(const sensor_msgs::PointCloud2ConstPtr &msg);
  void RadarCBK(const sensor_msgs::PointCloud2ConstPtr &msg);

  // Owns a local MeasureGroup and blocks in synchronizer_->WaitPop().
  void Run();
  void LogInvalidSync(const SyncDiagnostics &info) const;
  MapperInput Process(MeasureGroup &group);
  void UpdateMap(const MapperInput &input);
  void PublishResults(const MeasureGroup &group);
  // Returns extraction wall time in milliseconds, or NaN when publication is skipped.
  double PublishMapCloud(const ros::Time &stamp);
  void PublishUpdateStats(const MapperUpdateStats &stats);
  void WriteMapperTiming(double timestamp, std::size_t data_size,
                         double update_cost_ms, double get_map_cost_ms);

  ros::NodeHandle nh_;
  Options options_;

  // Mapping owns the synchronizer; destroy after joining workers.
  // ROS producers access it only through Synchronizer's synchronized API.
  std::unique_ptr<Synchronizer> synchronizer_;

  // Initialized before Run; used exclusively by Run until it has been joined.
  // Includes Update, Reset, Save and GetOccupiedCloud: no map mutex needed.
  std::unique_ptr<Mapper> mapper_;
  nav_msgs::Path path_;
  ros::Time last_map_publish_stamp_;
  bool has_published_map_ = false;
  double last_process_cpu_time_ms_ = 0.0;
  double last_process_wall_time_ms_ = 0.0;
  std::ofstream mapper_timing_csv_; // Written by Run(); closed after joining it.

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
  ros::Publisher pub_map_cloud_;
  ros::Publisher pub_update_stats_;

  std::thread run_thread_; // Stop() joins before any worker-owned state is destroyed.
};
} // namespace mapping

#endif // MAPPING_H
