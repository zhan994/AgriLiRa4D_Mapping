/**
 * @file octomap_mapper.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief octomap mapper for AgriLiRa4D
 * @version 0.1
 * @date 2026-07-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef MAPPER_OCTOMAP_H
#define MAPPER_OCTOMAP_H

#include <octomap/octomap.h>

#include "comm.h"
#include "mapper/mapper.h"

namespace mapping {

class OctoMapper : public Mapper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OctoMapper(const Options &options = Options());

  void Update(const MapperInput &input) override;
  void Reset() override;
  bool Save(const std::string &path) const override;

  void GetOccupiedCloud(PointCloudType &output) const override;

private:
  std::shared_ptr<octomap::OcTree> octree_;
  octomap::KeyRay key_ray_; // temp storage for ray casting
};

} // namespace mapping
#endif // MAPPER_OCTOMAP_H
