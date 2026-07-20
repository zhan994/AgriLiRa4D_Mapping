#include "preprocess.h"

namespace mapping {

void Preprocess::RSAiry2PCL(const sensor_msgs::PointCloud2::ConstPtr &msg,
                            CloudPtr &cloud, int filter_num, double blind) {
  // Implementation for converting RS_Airy LIDAR PointCloud2 to PCL PointCloud
}

void Preprocess::Radar2PCL(const sensor_msgs::PointCloud2::ConstPtr &msg,
                           RadarCloudPtr &cloud) {
  // Implementation for converting TXG Radar PointCloud2 to PCL PointCloud
}

} // namespace mapping