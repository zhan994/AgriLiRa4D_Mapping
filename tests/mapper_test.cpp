#include "mapper/mapper_octomap.h"

#include <octomap_msgs/conversions.h>
#include <iostream>
#include <stdexcept>

namespace {
void Check(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
}

int main() {
  try {
    mapping::OctoMapper::Options options;
    options.resolution = 0.5;
    options.max_range = 5.0;
    mapping::OctoMapper mapper(options);
    mapping::MapperInput input;
    input.lidar_origin = V3D(11.1, 5.1, 2.1);
    input.lidar_cloud.reset(new PointCloudType);
    PointType endpoint{};
    endpoint.x = 15.1;
    endpoint.y = 5.1;
    endpoint.z = 2.1;
    input.lidar_cloud->push_back(endpoint);
    input.lidar_cloud->push_back(endpoint); // One hit per voxel per update.
    mapper.Update(input);

    const auto *tree = mapper.GetOctree();
    const auto *occupied = tree->search(15.1, 5.1, 2.1);
    const auto *free = tree->search(13.1, 5.1, 2.1);
    Check(occupied && tree->isNodeOccupied(occupied), "Missing world-frame hit voxel");
    Check(std::abs(occupied->getOccupancy() - options.hit_prob) < 1e-6,
          "Duplicate points updated the same hit more than once");
    Check(free && !tree->isNodeOccupied(free), "Missing free ray from sensor origin");
    Check(!tree->search(10.1, 5.1, 2.1), "Ray used world origin instead of sensor origin");

    octomap_msgs::Octomap message;
    Check(octomap_msgs::binaryMapToMsg(*tree, message), "Map serialization failed");
    std::unique_ptr<octomap::AbstractOcTree> decoded(octomap_msgs::binaryMsgToMap(message));
    const auto *decoded_tree = dynamic_cast<octomap::OcTree *>(decoded.get());
    Check(decoded_tree && decoded_tree->isNodeOccupied(decoded_tree->search(15.1, 5.1, 2.1)),
          "Published binary map lost occupied endpoint");

    mapper.Reset();
    input.lidar_cloud->points[0].x = 50.1; // Out of range: free ray only, no hit.
    input.lidar_cloud->resize(1);
    mapper.Update(input);
    free = tree->search(15.1, 5.1, 2.1);
    Check(free && !tree->isNodeOccupied(free), "Range-limited ray did not update free space");
    Check(!tree->search(50.1, 5.1, 2.1), "Out-of-range endpoint marked occupied");
    std::cout << "Mapper tests passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
