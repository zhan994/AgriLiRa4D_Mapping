/**
 * @file mapper_ioctree.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief i-octree for AgriLiRa4D
 * @version 0.1
 * @date 2026-07-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef IOCTREE_MAPPER_H
#define IOCTREE_MAPPER_H

#include "mapper/mapper.h"

namespace mapping {

class I_OctreeMapper : public Mapper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options {
    Options() {}
  };

  explicit I_OctreeMapper(const Options &options = Options());

  void Update(const MapperInput &input) override;
  void Reset() override;
  bool Save(const std::string &path) const override;
  const octomap::OcTree *GetOctree() const override;

private:
  Options options_;
};
} // namespace mapping

#endif // IOCTREE_MAPPER_H