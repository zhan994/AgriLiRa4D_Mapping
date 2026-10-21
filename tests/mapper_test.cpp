#include "mapper/mapper_octomap.h"

#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#include <tuple>

namespace {
void Check(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

PointType Point(float x, float y, float z) {
  PointType point{};
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

using Centers = std::set<std::tuple<float, float, float>>;
Centers Coordinates(const PointCloudType &cloud) {
  Centers centers;
  for (const auto &point : cloud)
    centers.emplace(point.x, point.y, point.z);
  return centers;
}
}

int main() {
  char directory[] = "/tmp/agrilira-mapper-XXXXXX";
  if (!mkdtemp(directory))
    return 1;
  const auto cleanup = [&] { std::filesystem::remove_all(directory); };
  try {
    mapping::OctoMapper::Options options;
    options.resolution = 0.5;
    options.max_range = 5.0;
    mapping::OctoMapper mapper(options);
    mapping::Mapper &backend = mapper;
    mapping::MapperInput input;
    input.lidar_origin = V3D(11.1, 5.1, 2.1);
    input.lidar_cloud.reset(new PointCloudType);
    input.lidar_cloud->push_back(Point(15.1, 5.1, 2.1));
    input.lidar_cloud->push_back(input.lidar_cloud->front());
    // A farther ray crosses the first endpoint: hit must win over free.
    input.lidar_cloud->push_back(Point(15.6, 5.1, 2.1));
    backend.Update(input);

    PointCloudType cloud;
    backend.GetOccupiedCloud(cloud);
    Check(cloud.size() == 2 &&
              Coordinates(cloud) == Centers{{15.25f, 5.25f, 2.25f},
                                            {15.75f, 5.25f, 2.25f}},
          "Wrong occupied centers, or free space included in output");
    Check(cloud.header.frame_id == "world", "Wrong output frame");

    // Three misses clear a single 0.7 hit, but would not clear two hits.
    input.lidar_cloud->clear();
    input.lidar_cloud->push_back(Point(50.1, 5.1, 2.1));
    for (int i = 0; i < 3; ++i)
      backend.Update(input);
    backend.GetOccupiedCloud(cloud);
    Check(cloud.empty(), "Duplicate hits, missing free updates or out-of-range hit");

    backend.Reset();
    backend.GetOccupiedCloud(cloud);
    Check(cloud.empty(), "Reset did not clear output");
    backend.Update(input);
    backend.GetOccupiedCloud(cloud);
    Check(cloud.empty(), "Range-limited ray produced occupied points");

    // Each observation starts and ends in one voxel, avoiding free updates.
    // A 4x4x4 block of identical hits merges across two levels. Check both
    // positive and negative coordinates against the same OctoMap operations.
    for (const float offset : {0.0f, -2.0f}) {
      backend.Reset();
      octomap::OcTree reference(options.resolution);
      reference.setProbHit(options.hit_prob);
      Centers expected;
      for (int x = 0; x < 4; ++x) {
        for (int y = 0; y < 4; ++y) {
          for (int z = 0; z < 4; ++z) {
            const auto point = Point(offset + x * 0.5f + 0.25f,
                                     offset + y * 0.5f + 0.25f,
                                     offset + z * 0.5f + 0.25f);
            input.lidar_origin = V3D(point.x, point.y, point.z);
            input.lidar_cloud->clear();
            input.lidar_cloud->push_back(point);
            backend.Update(input);
            reference.updateNode(point.x, point.y, point.z, true);
            expected.emplace(point.x, point.y, point.z);
          }
        }
      }
      auto leaf = reference.begin_leafs();
      Check(leaf != reference.end_leafs() && leaf.getSize() == 2.0,
            "Fixture did not create a merged leaf");
      ++leaf;
      Check(leaf == reference.end_leafs(), "Fixture should merge to one leaf");
      backend.GetOccupiedCloud(cloud);
      Check(cloud.size() == 64 && Coordinates(cloud) == expected,
            "Merged leaf did not export all base-resolution voxel centers");
      backend.GetOccupiedCloud(cloud);
      Check(cloud.size() == 64, "Extraction appended instead of replacing output");

      const std::string path = std::string(directory) + "/occupied.pcd";
      Check(backend.Save(path), "Failed to save occupied PCD");
      PointCloudType restored;
      Check(pcl::io::loadPCDFile(path, restored) == 0 &&
                restored.size() == 64 && Coordinates(restored) == expected,
            "Saved PCD differs from exported occupied centers");
      backend.GetOccupiedCloud(cloud);
      Check(Coordinates(cloud) == expected, "Save changed the map");
    }
    Check(!backend.Save(std::string(directory) + "/missing/map.pcd"),
          "Save did not report a write failure");
    backend.Reset();
    backend.GetOccupiedCloud(cloud);
    Check(cloud.empty(), "Reset left stale occupied points");
    cleanup();
    std::cout << "Mapper tests passed\n";
  } catch (const std::exception &error) {
    cleanup();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
