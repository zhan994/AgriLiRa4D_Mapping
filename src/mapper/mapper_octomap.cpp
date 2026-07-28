#include "mapper/mapper_octomap.h"

namespace mapping {

OctoMapper::OctoMapper(const Options &options) : options_(options) {
  octree_ = std::make_shared<octomap::OcTree>(options_.resolution);
  octree_->setProbHit(options_.hit_probability);
  octree_->setProbMiss(options_.miss_probability);
  octree_->setClampingThresMin(options_.clamping_min);
  octree_->setClampingThresMax(options_.clamping_max);
  octree_->setOccupancyThres(options_.occupancy_threshold);
}

void OctoMapper::Update(const MapperInput &input) {
  if (!octree_) {
    ROS_ERROR("OctoMapper: octree is not initialized.");
    return;
  }

  // Insert LiDAR points into the octree
  for (const auto &point : input.lidar_cloud->points) {
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