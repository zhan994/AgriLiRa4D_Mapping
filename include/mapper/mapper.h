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

/**
 * @brief One synchronized observation expressed in the world frame.
 *
 * Mapping owns synchronization, motion compensation and extrinsic transforms.
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
  virtual ~Mapper() = default;
  virtual void Update(const MapperInput &input) = 0;
  virtual void Reset() = 0;
  virtual bool Save(const std::string &path) const = 0;
};

} // namespace mapping

#endif // MAPPER_H
