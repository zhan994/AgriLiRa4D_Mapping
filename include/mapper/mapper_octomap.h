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

#ifndef OCTOMAP_MAPPER_H
#define OCTOMAP_MAPPER_H

#include <octomap/octomap.h>

#include "comm.h"
#include "mapper/mapper.h"

namespace mapping {

class OctoMapper : public Mapper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options {
    Options() {}
    double resolution = 0.5; // default resolution in meters
    double max_range = 50.0;
    double hit_prob = 0.7;
    double miss_prob = 0.4;
    double occupancy_threshold = 0.5;
  };

  explicit OctoMapper(const Options &options = Options());

  void Update(const MapperInput &input) override;
  void Reset() override;
  bool Save(const std::string &path) const override;

  const octomap::OcTree *GetOctree() const override { return octree_.get(); }

private:
  Options options_;
  std::shared_ptr<octomap::OcTree> octree_;
  octomap::KeyRay key_ray_; // temp storage for ray casting
};

} // namespace mapping
#endif // OCTOMAP_MAPPER_H
