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

void Preprocess::UndistortPcl(MeasureGroup &meas, const M3D &R_bl,
                              const V3D &t_bl) {
  // Undistort LiDAR points based on synchronized pose data
  if (meas.pose_data.empty() || meas.lidar_points_stamped.empty()) {
    ROS_WARN("No pose or LiDAR data to undistort.");
    return;
  }

  ROS_INFO("Undistorting LiDAR points using %lu pose data points.",
           meas.pose_data.size());

  // target pose is the last pose in the synchronized data
  M3D R_wb = meas.pose_data.back().rot;
  V3D t_wb = meas.pose_data.back().trans;

  // Iterate through poses and points in reverse chronological order.
  auto it_pcl = meas.lidar_cloud->points.end() - 1;
  for (auto it_kp = meas.pose_data.end(); it_kp != meas.pose_data.begin();) {
    --it_kp;
    double offset = it_kp->offset - meas.lidar_beg_time;
    M3D R_wb_kp = it_kp->rot;
    V3D t_wb_kp = it_kp->trans;

    for (; it_pcl->curvature / double(1000) > offset; --it_pcl) {
      V3D pt(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D pt_compensated =
          R_bl.transpose() *
          (R_wb.transpose() * (R_wb_kp * (R_bl * pt + t_bl) + t_wb_kp - t_wb) -
           t_bl);

      it_pcl->x = pt_compensated.x();
      it_pcl->y = pt_compensated.y();
      it_pcl->z = pt_compensated.z();

      // All points have been processed; do not revisit the first point
      // when the outer loop advances to an earlier pose.
      if (it_pcl == meas.lidar_cloud->points.begin())
        return;
    }
  }
}

void Preprocess::TransformPointCloud(const CloudPtr &cloud, const M3D &r,
                                     const V3D &t, CloudPtr &output) {
  if (cloud->empty()) {
    output->clear();
    std::cout << "Warning: Transforming empty point cloud." << std::endl;
    return;
  }

  M4F transform = M4F::Identity();
  transform.block<3, 3>(0, 0) = r.cast<float>();
  transform.block<3, 1>(0, 3) = t.cast<float>();

  pcl::transformPointCloud(*cloud, *output, transform);
}

void Preprocess::TransformRadarPointCloud(const RadarCloudPtr &cloud,
                                          const M3D &r, const V3D &t,
                                          RadarCloudPtr &output) {
  if (cloud->empty()) {
    output->clear();
    std::cout << "Warning: Transforming empty point cloud." << std::endl;
    return;
  }

  M4F transform = M4F::Identity();
  transform.block<3, 3>(0, 0) = r.cast<float>();
  transform.block<3, 1>(0, 3) = t.cast<float>();

  pcl::transformPointCloud(*cloud, *output, transform);
}

} // namespace mapping
