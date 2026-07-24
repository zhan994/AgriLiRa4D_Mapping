#include "mapping.h"

namespace mapping {

Mapping::Mapping(ros::NodeHandle &nh, const Options &options)
    : nh_(nh), options_(options) {
  sub_lidar_ =
      nh_.subscribe(options_.lidar_topic, 10, &Mapping::LidarCBK, this);
  sub_radar_ =
      nh_.subscribe(options_.radar_topic, 10, &Mapping::RadarCBK, this);

  pub_lidar_aft_mapped_ = nh_.advertise<sensor_msgs::PointCloud2>(
      options_.lidar_registered_topic, 10);
  pub_radar_aft_mapped_ = nh_.advertise<sensor_msgs::PointCloud2>(
      options_.radar_registered_topic, 10);
}

void Mapping::LidarCBK(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // Handle LIDAR point cloud message
}

void Mapping::RadarCBK(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // Handle Radar point cloud message
}

} // namespace mapping