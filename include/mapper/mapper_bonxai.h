/**
 * @file mapper_bonxai.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef MAPPER_BONXAI_H
#define MAPPER_BONXAI_H

#include "bonxai/bonxai.hpp"

#include "comm.h"
#include "mapper/mapper.h"

namespace mapping {

class BonxaiMapper : public Mapper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit BonxaiMapper(const Options &options = Options());

  void Update(const MapperInput &input) override;
  void Reset() override;
  bool Save(const std::string &path) const override;

  void GetOccupiedCloud(PointCloudType &output) const override;

private:
  struct Cell {
    enum : uint8_t { Unseen, Free, Hit };
    float log_odds = 0.0f;
    uint8_t flag = Unseen; // Transient; cleared after each Update().
  };

  Bonxai::VoxelGrid<Cell> grid_;
  float hit_log_;
  float miss_log_;
  float occupancy_log_;
  float clamp_min_log_;
  float clamp_max_log_;
};

} // namespace mapping

#endif // MAPPER_BONXAI_H
