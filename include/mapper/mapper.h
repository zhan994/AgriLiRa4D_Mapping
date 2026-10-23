/**
 * @file mapper.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief mapper for AgriLiRa4D
 * @version 0.1
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef MAPPER_H
#define MAPPER_H

#include "comm.h"

#include <yaml-cpp/yaml.h>

namespace mapping {

/// @brief  Check if a point is valid (finite and within a reasonable range)
bool IsValidPoint(const double &x, const double &y, const double &z);

/**
 * @brief One synchronized observation expressed in the world frame.
 *
 * Synchronizer owns time synchronization; Mapping owns motion compensation
 * and extrinsic transforms.
 * A mapper backend only consumes the resulting observation and maintains its
 * own map representation.
 */
struct MapperInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = -1.0;
  Pose6D body_pose;
  V3D lidar_origin = V3D::Zero();
  V3D radar_origin = V3D::Zero();
  CloudPtr lidar_cloud;
  RadarCloudPtr radar_cloud;
};

class Mapper {
public:
  struct Options {
    Options() {}
    double resolution = 0.5; // meters
    double max_range = 50.0;
    double hit_prob = 0.7;
    double miss_prob = 0.4;
    double occupancy_threshold = 0.5;
    // Approximation: trace once per endpoint voxel, toward its center.
    bool deduplicate_rays = false;
  };

  struct UpdateDiagnostics {
    std::size_t valid_points = 0;
    std::size_t candidate_rays = 0; // Valid endpoints outside the origin voxel.
    std::size_t ray_count = 0;      // Actual traversal calls after deduplication.
    std::size_t touched_voxels = 0; // Unique free/hit voxels updated this input.
  };

  explicit Mapper(const Options &options = Options());

  // No internal worker or locking. All methods run on the mapping thread,
  // or on the owner thread before startup / after the mapping thread is joined.
  virtual ~Mapper() = default;
  virtual void Update(const MapperInput &input) = 0;
  virtual void Reset() = 0;
  // Save occupied voxel centers as a binary PCD file; false on write failure.
  virtual bool Save(const std::string &path) const = 0;
  // Replace output with occupied voxel centers in the world frame at the
  // configured base resolution, including voxels represented by merged leaves.
  virtual void GetOccupiedCloud(PointCloudType &output) const = 0;
  const UpdateDiagnostics &GetUpdateDiagnostics() const { return diagnostics_; }

protected:
  const Options options_;
  UpdateDiagnostics diagnostics_;
};

} // namespace mapping

#endif // MAPPER_H
