#include "mapper/mapper_octomap.h"
#include "mapper/mapper_bonxai.h"

#include <limits>
#include <random>

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

template <typename Backend>
void TestBackend(const std::string &directory) {
  mapping::Mapper::Options options;
  options.resolution = 0.5;
  options.max_range = 5.0;
  Backend mapper(options);
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
}

void TestParity() {
  for (bool deduplicate : {false, true}) {
    for (const double threshold : {0.5, 0.65, 0.7}) {
      mapping::Mapper::Options options;
      options.resolution = 0.5;
      options.max_range = 5.0;
      options.occupancy_threshold = threshold;
      options.deduplicate_rays = deduplicate;
      mapping::OctoMapper octo(options);
      mapping::BonxaiMapper bonxai(options);
      mapping::MapperInput input;
      input.lidar_cloud.reset(new PointCloudType);
      auto compare = [&] {
        octo.Update(input);
        bonxai.Update(input);
        PointCloudType expected, actual;
        octo.GetOccupiedCloud(expected);
        bonxai.GetOccupiedCloud(actual);
        Check(expected.size() == actual.size() &&
                  Coordinates(expected) == Coordinates(actual),
              "Bonxai occupied voxels differ from OctoMap");
      };

      // Pre-fill the scene so that free-space traversal differences become visible
      // immediately at threshold 0.65: one miss clears a voxel with one 0.7 hit.
      for (int x = -10; x <= 10; ++x) {
        for (int y = -10; y <= 10; ++y) {
          for (int z = -2; z <= 2; ++z) {
            auto point = Point(x * 0.5f + 0.25f, y * 0.5f + 0.25f,
                               z * 0.5f + 0.25f);
            input.lidar_origin = V3D(point.x, point.y, point.z);
            input.lidar_cloud->clear();
            input.lidar_cloud->push_back(point);
            octo.Update(input);
            bonxai.Update(input);
          }
        }
      }
      input.lidar_origin = V3D(0.1, 0.1, 0.1);
      input.lidar_cloud->clear();
      // Same endpoint voxel, different actual rays; compare both backends in each mode.
      input.lidar_cloud->push_back(Point(4.01, 0.51, 0.1));
      input.lidar_cloud->push_back(Point(4.49, 0.99, 0.1));
      input.lidar_cloud->push_back(input.lidar_cloud->front());
      compare();

      // Axis ties, negative directions, boundary origins, exact maximum range,
      // invalid returns and a return in the origin voxel.
      input.lidar_origin = V3D::Zero();
      input.lidar_cloud->clear();
      for (auto point : {Point(2, 2, 2), Point(-2, -2, -2), Point(5, 0, 0),
                         Point(20, 0, 0), Point(0.1, 0.1, 0.1),
                         Point(std::numeric_limits<float>::quiet_NaN(), 0, 0),
                         Point(0, std::numeric_limits<float>::infinity(), 0)})
        input.lidar_cloud->push_back(point);
      compare();
      input.lidar_origin.x() = std::numeric_limits<double>::quiet_NaN();
      compare();
      input.lidar_origin = V3D(-0.5, 0.5, -0.5);

      std::mt19937 random(42);
      std::uniform_real_distribution<float> coordinate(-7.0f, 7.0f);
      for (int frame = 0; frame < 50; ++frame) {
        input.lidar_cloud->clear();
        for (int i = 0; i < 50; ++i)
          input.lidar_cloud->push_back(Point(coordinate(random), coordinate(random),
                                            coordinate(random)));
        input.lidar_cloud->push_back(input.lidar_cloud->front());
        compare();
      }

      // Saturate a hit voxel, then reverse it with repeated free rays. Revisit
      // after several scans elsewhere to catch stale transient flags.
      octo.Reset();
      bonxai.Reset();
      input.lidar_origin = V3D(0.25, 0.25, 0.25);
      input.lidar_cloud->clear();
      input.lidar_cloud->push_back(Point(2.25, 0.25, 0.25));
      for (int i = 0; i < 30; ++i)
        compare();
      input.lidar_cloud->front().x = 20.25f;
      for (int i = 0; i < 30; ++i)
        compare();
      input.lidar_cloud->front().x = 2.25f;
      for (int i = 0; i < 10; ++i)
        compare();
    }
  }
  mapping::Mapper::Options invalid;
  invalid.resolution = 0;
  bool rejected = false;
  try {
    mapping::BonxaiMapper mapper(invalid);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  Check(rejected, "Invalid Bonxai options were accepted");
}

template <typename Backend>
void TestRayDeduplication() {
  mapping::Mapper::Options options;
  options.max_range = 5.0;
  options.deduplicate_rays = true;
  Backend dedup(options);
  options.deduplicate_rays = false;
  Backend baseline(options);
  mapping::MapperInput input;
  input.lidar_origin = V3D(0.25, 0.25, 0.25);
  input.lidar_cloud.reset(new PointCloudType);
  for (int i = 0; i < 100; ++i)
    input.lidar_cloud->push_back(Point(2.25, 0.25, 0.25));
  // A clipped miss and a hit share an endpoint voxel. The hit must still win.
  input.lidar_cloud->push_back(Point(5.1, 0.25, 0.25));
  input.lidar_cloud->push_back(Point(20.25, 0.25, 0.25));
  dedup.Update(input);
  baseline.Update(input);
  const auto &stats = dedup.GetUpdateDiagnostics();
  Check(stats.valid_points == 102 && stats.candidate_rays == 102 && stats.ray_count == 2,
        "Endpoint voxel deduplication did not reduce traversal calls");
  Check(baseline.GetUpdateDiagnostics().ray_count == 102,
        "Per-point baseline unexpectedly deduplicated rays");
  PointCloudType expected, actual;
  dedup.GetOccupiedCloud(actual);
  baseline.GetOccupiedCloud(expected);
  Check(Coordinates(actual) == Coordinates(expected) && actual.size() == 2,
        "Deduplication lost hits or included a clipped endpoint as free");
  Check(stats.touched_voxels == baseline.GetUpdateDiagnostics().touched_voxels,
        "Identical center-aligned rays should touch the same voxels");
  // Flags and endpoint sets must not persist across updates or Reset().
  dedup.Update(input);
  Check(dedup.GetUpdateDiagnostics().ray_count == 2, "Stale ray endpoint state");
  dedup.Reset();
  Check(dedup.GetUpdateDiagnostics().ray_count == 0, "Reset left stale diagnostics");
  input.lidar_cloud->clear();
  dedup.Update(input);
  Check(dedup.GetUpdateDiagnostics().valid_points == 0, "Empty input left stale diagnostics");
}

void TestApproximationChangesFreeSpace() {
  mapping::Mapper::Options options;
  options.occupancy_threshold = 0.65; // One miss clears a single hit.
  mapping::BonxaiMapper baseline(options);
  options.deduplicate_rays = true;
  mapping::BonxaiMapper dedup(options);
  mapping::MapperInput input;
  input.lidar_cloud.reset(new PointCloudType);
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 3; ++y) {
      auto point = Point(x * 0.5f + 0.25f, y * 0.5f + 0.25f, 0.25f);
      input.lidar_origin = V3D(point.x, point.y, point.z);
      input.lidar_cloud->clear();
      input.lidar_cloud->push_back(point);
      baseline.Update(input);
      dedup.Update(input);
    }
  }
  input.lidar_origin = V3D(0.1, 0.1, 0.25);
  input.lidar_cloud->clear();
  input.lidar_cloud->push_back(Point(4.01, 0.51, 0.25));
  input.lidar_cloud->push_back(Point(4.49, 0.99, 0.25));
  baseline.Update(input);
  dedup.Update(input);
  PointCloudType base_cloud, dedup_cloud;
  baseline.GetOccupiedCloud(base_cloud);
  dedup.GetOccupiedCloud(dedup_cloud);
  Check(Coordinates(base_cloud) != Coordinates(dedup_cloud),
        "Fixture should demonstrate changed free-space coverage");
}

int main() {
  char directory[] = "/tmp/agrilira-mapper-XXXXXX";
  if (!mkdtemp(directory))
    return 1;
  const auto cleanup = [&] { std::filesystem::remove_all(directory); };
  try {
    TestBackend<mapping::OctoMapper>(directory);
    TestBackend<mapping::BonxaiMapper>(directory);
    TestParity();
    TestRayDeduplication<mapping::OctoMapper>();
    TestRayDeduplication<mapping::BonxaiMapper>();
    TestApproximationChangesFreeSpace();
    cleanup();
    std::cout << "Mapper tests passed (OctoMap, Bonxai and parity)\n";
  } catch (const std::exception &error) {
    cleanup();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
