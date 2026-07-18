/**
 * @file comm.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief common functions for AgriLiRa4D
 * @version 0.1
 * @date 2026-07-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef COMM_H
#define COMM_H

#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using RadarPointType = txg_radar::Point;
using RadarPointCloudType = pcl::PointCloud<RadarPointType>;
using RadarCloudPtr = RadarPointCloudType::Ptr;
using RadarPointVector =
    std::vector<RadarPointType, Eigen::aligned_allocator<RadarPointType>>;

using V2D = Eigen::Vector2d;
using V3D = Eigen::Vector3d;
using V4D = Eigen::Vector4d;
using V5D = Eigen::Matrix<double, 5, 1>;
using M2D = Eigen::Matrix2d;
using M3D = Eigen::Matrix3d;
using M4D = Eigen::Matrix4d;
using V2F = Eigen::Vector2f;
using V3F = Eigen::Vector3f;
using V4F = Eigen::Vector4f;
using V5F = Eigen::Matrix<float, 5, 1>;
using M3F = Eigen::Matrix3f;
using M4F = Eigen::Matrix4f;
using VXD = Eigen::VectorXd;
using VXF = Eigen::VectorXf;
using MXD = Eigen::MatrixXd;
using MXF = Eigen::MatrixXf;
using V12D = Eigen::Matrix<double, 12, 1>;

using M24D = Eigen::Matrix<double, 24, 24>;
using M12D = Eigen::Matrix<double, 12, 12>;
using V24D = Eigen::Matrix<double, 24, 1>;
using V12D = Eigen::Matrix<double, 12, 1>;
using M24X12D = Eigen::Matrix<double, 24, 12>;
using M6D = Eigen::Matrix<double, 6, 6>;
using M24X3D = Eigen::Matrix<double, 24, 3>;

using VV3D = std::vector<V3D, Eigen::aligned_allocator<V3D>>;
using VV3F = std::vector<V3F, Eigen::aligned_allocator<V3F>>;
using VV4F = std::vector<V4F, Eigen::aligned_allocator<V4F>>;
using VV4D = std::vector<V4D, Eigen::aligned_allocator<V4D>>;
using VV5F = std::vector<V5F, Eigen::aligned_allocator<V5F>>;
using VV5D = std::vector<V5D, Eigen::aligned_allocator<V5D>>;
using VM3D = std::vector<M3D, Eigen::aligned_allocator<M3D>>;

using QD = Eigen::Quaterniond;
using QF = Eigen::Quaternionf;

namespace robosense_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace robosense_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

// clang-format on
namespace txg_radar {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;     // preferred way of adding a XYZ+padding
  float v_doppler_mps; // Doppler velocity in m/s
  float snr_db;        // Signal-to-noise ratio in dB
  float rcs;           // Radar cross-section in m^2
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace txg_radar
  // clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(txg_radar::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, v_doppler_mps, v_doppler_mps)
    (float, snr_db, snr_db)
    (float, rcs, rcs)
)

// clang-format on
namespace mapping {

class Utils {
public:
  // Convert ROS PointCloud2 message to PCL PointCloud for RS_Airy LIDAR
  static void RSAiry2PCL(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         CloudPtr &cloud, int filter_num, double blind);

  // Convert ROS PointCloud2 message to PCL PointCloud for TXG Radar
  static void Radar2PCL(const sensor_msgs::PointCloud2::ConstPtr &msg,
                        RadarCloudPtr &cloud);
};
} // namespace mapping

#endif // COMM_H