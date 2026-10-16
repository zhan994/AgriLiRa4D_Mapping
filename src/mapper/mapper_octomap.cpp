#include "mapper/mapper_octomap.h"

#include <stdexcept>

namespace mapping {

OctoMapper::OctoMapper(const Options &options) : options_(options) {
  if (!std::isfinite(options_.resolution) || options_.resolution <= 0.0 ||
      !std::isfinite(options_.max_range) || options_.max_range <= 0.0 ||
      !std::isfinite(options_.hit_prob) || options_.hit_prob < 0.5 ||
      options_.hit_prob >= 1.0 || !std::isfinite(options_.miss_prob) ||
      options_.miss_prob <= 0.0 || options_.miss_prob > 0.5 ||
      !std::isfinite(options_.occupancy_threshold) ||
      options_.occupancy_threshold <= 0.0 || options_.occupancy_threshold >= 1.0)
    throw std::invalid_argument("Invalid OctoMap resolution, range or probabilities");
  octree_ = std::make_shared<octomap::OcTree>(options_.resolution);
  octree_->setProbHit(options_.hit_prob);
  octree_->setProbMiss(options_.miss_prob);
  octree_->setOccupancyThres(options_.occupancy_threshold);
}

void OctoMapper::Update(const MapperInput &input) {
  if (!octree_) {
    ROS_ERROR("OctoMapper: octree is not initialized.");
    return;
  }

  if (!input.lidar_cloud || input.lidar_cloud->empty()) {
    ROS_WARN("MapperInput has no LiDAR data.");
    return;
  }

  octomap::point3d lidar_origin(input.lidar_origin.x(), input.lidar_origin.y(),
                                input.lidar_origin.z());
  if (!IsValidPoint(lidar_origin.x(), lidar_origin.y(), lidar_origin.z())) {
    ROS_WARN("OctoMapper: Invalid LiDAR origin point.");
    return;
  }

  octomap::KeySet hit_cells, free_cells;
  for (const auto &point : input.lidar_cloud->points) {
    if (!IsValidPoint(point.x, point.y, point.z))
      continue;

    octomap::point3d tgt_pt(point.x, point.y, point.z);

    // 方向向量
    octomap::point3d dir_pt = tgt_pt - lidar_origin;
    float len = dir_pt.norm();
    if (len > options_.max_range) {
      octomap::point3d dir_unit(dir_pt.x() / len, dir_pt.y() / len,
                                dir_pt.z() / len);
      tgt_pt = lidar_origin + dir_unit * options_.max_range;
    }

    // 更新free
    if (octree_->computeRayKeys(lidar_origin, tgt_pt, key_ray_)) {
      free_cells.insert(key_ray_.begin(), key_ray_.end());
    }

    // 严格小于最大测距范围时更新hit
    if (len < options_.max_range) {
      // hit
      octomap::OcTreeKey tgt_key;
      if (octree_->coordToKeyChecked(tgt_pt, tgt_key)) {
        hit_cells.insert(tgt_key);
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for (auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    if (hit_cells.find(*it) == hit_cells.end()) {
      octree_->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (auto it = hit_cells.begin(), end = hit_cells.end(); it != end; it++) {
    octree_->updateNode(*it, true);
  }
}

void OctoMapper::Reset() {
  if (octree_)
    octree_->clear();
}

bool OctoMapper::Save(const std::string &path) const {
  if (!octree_) {
    ROS_ERROR("OctoMapper: octree is not initialized.");
    return false;
  }
  return octree_->writeBinary(path);
}

} // namespace mapping
