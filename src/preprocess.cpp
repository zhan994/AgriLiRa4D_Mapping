#include "preprocess.h"

namespace mapping {

void Preprocess::RSAiry2PCL(const sensor_msgs::PointCloud2::ConstPtr &msg,
                            CloudPtr &cloud, int filter_num, double blind) {
  cloud->clear();

  pcl::PointCloud<robosense_ros::Point> pl_full;
  pcl::fromROSMsg(*msg, pl_full);

  int plsize = pl_full.points.size();
  for (int i = 0; i < plsize; i++) {
    if (!(std::isfinite(pl_full.points[i].x) &&
          std::isfinite(pl_full.points[i].y) &&
          std::isfinite(pl_full.points[i].z)))
      continue;

    // std::cout << pl_full.points[i].timestamp - pl_full.points[0].timestamp <<
    // std::endl;
    PointType added_pt;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_full.points[i].x;
    added_pt.y = pl_full.points[i].y;
    added_pt.z = pl_full.points[i].z;
    added_pt.intensity = pl_full.points[i].intensity;
    added_pt.curvature =
        (pl_full.points[i].timestamp - pl_full.points[0].timestamp) *
        float(1000); // convert to `ms` relative time

    if (i % filter_num == 0) {
      double sq_range = added_pt.x * added_pt.x + added_pt.y * added_pt.y +
                        added_pt.z * added_pt.z;
      if (sq_range > (blind * blind)) {
        cloud->points.push_back(added_pt);
      }
    }
  }
}

void Preprocess::Radar2PCL(const sensor_msgs::PointCloud2::ConstPtr &msg,
                           RadarCloudPtr &cloud) {
  cloud->clear();
  pcl::fromROSMsg(*msg, *cloud);
}

void Preprocess::Undistort(MeasureGroup &meas) {
  // Undistort LiDAR points based on synchronized pose data
  if (meas.pose_data.empty() || meas.lidar_points_stamped.empty()) {
    ROS_WARN("No pose or LiDAR data to undistort.");
    return;
  }

  ROS_INFO("Undistorting LiDAR points using %lu pose data points.",
           meas.pose_data.size());
}

} // namespace mapping