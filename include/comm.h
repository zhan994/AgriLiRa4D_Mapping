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
struct EIGEN_ALIGN16 PointStamped {
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  float intensity;  // Doppler velocity in m/s
  double timestamp; // timestamp

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(PointStamped,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
)

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using RadarPointType = txg_radar::Point;
using RadarPointCloudType = pcl::PointCloud<RadarPointType>;
using RadarCloudPtr = RadarPointCloudType::Ptr;
using RadarPointVector =
    std::vector<RadarPointType, Eigen::aligned_allocator<RadarPointType>>;

using PointStampedCloudType = pcl::PointCloud<PointStamped>;
using PointStampedCloudPtr = PointStampedCloudType::Ptr;
using PointStampedVector = std::vector<PointStamped, Eigen::aligned_allocator<PointStamped>>;

// clang-format on
namespace mapping {
/// @brief  6D pose struct
struct Pose6D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double offset;
  V3D acc;
  V3D gyro;
  V3D vel;
  V3D trans;
  M3D rot;

  Pose6D() = default;
  Pose6D(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p,
         const M3D &r)
      : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

/// @brief  IMU data struct
struct IMUData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  V3D acc;
  V3D gyro;
  double timestamp;

  IMUData() = default;
  IMUData(const V3D &a, const V3D &g, double &d)
      : acc(a), gyro(g), timestamp(d) {}
};

/// @brief  Measurement group struct
struct MeasureGroup {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MeasureGroup() {
    lidar_cloud.reset(new PointCloudType);
    lidar_points_stamped.clear();
    radar_cloud.reset(new RadarPointCloudType);
    pose_data.clear();
  }

  double lidar_beg_time = -1.0;
  double lidar_end_time = -1.0;
  double radar_time = -1.0;

  CloudPtr lidar_cloud;
  std::deque<PointStamped> lidar_points_stamped;
  RadarCloudPtr radar_cloud;
  std::deque<Pose6D> pose_data;
};
} // namespace mapping

/// @brief  Compute time increment in milliseconds
inline double
TimeInc(const std::chrono::high_resolution_clock::time_point &t_end,
        const std::chrono::high_resolution_clock::time_point &t_begin) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                   t_begin)
             .count() *
         1000.0;
}

/// @brief  Compute squared distance between two points
inline double SquareDist(const PointType &p1, const PointType &p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
         (p1.z - p2.z) * (p1.z - p2.z);
}

#endif // COMM_H