// Synthetic workload only: isolates endpoint multiplicity, not real sensor performance.
#include "mapper/mapper_bonxai.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using Clock = std::chrono::steady_clock;
double Milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
void Record(std::ofstream &stream, double timestamp, std::size_t size,
            double update_ms, double get_ms, const mapping::Mapper &mapper, bool dedup) {
  const auto &stats = mapper.GetUpdateDiagnostics();
  stream << std::fixed << std::setprecision(9) << timestamp << ',' << size << ','
         << std::setprecision(6) << update_ms << ',' << get_ms << ','
         << (dedup ? "endpoint_voxel" : "per_point") << ',' << stats.valid_points << ','
         << stats.candidate_rays << ',' << stats.ray_count << ',' << stats.touched_voxels << '\n';
}
}

int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: mapper_ray_benchmark OUTPUT_DIRECTORY [repeats=20]\n";
    return 1;
  }
  try {
    const int repeats = argc == 3 ? std::stoi(argv[2]) : 20;
    if (repeats < 1)
      throw std::invalid_argument("repeats must be positive");
    const std::filesystem::path directory(argv[1]);
    std::filesystem::create_directories(directory);
    for (int multiplicity : {1, 4, 16}) {
      mapping::Mapper::Options options;
      options.max_range = 50;
      mapping::BonxaiMapper baseline(options);
      options.deduplicate_rays = true;
      mapping::BonxaiMapper dedup(options);
      mapping::MapperInput input;
      input.lidar_origin = V3D(0.25, 0.25, 0.25);
      input.lidar_cloud.reset(new PointCloudType);
      for (int y = -20; y < 20; ++y) {
        for (int z = -10; z < 10; ++z) {
          PointType point{};
          point.x = 20.25f;
          point.y = y * 0.5f + 0.25f;
          point.z = z * 0.5f + 0.25f;
          for (int i = 0; i < multiplicity; ++i)
            input.lidar_cloud->push_back(point);
        }
      }
      // Warm both maps identically; keep warm-up samples out of the CSV.
      for (int i = 0; i < 3; ++i) {
        baseline.Update(input);
        dedup.Update(input);
      }
      std::ofstream base_csv(directory / ("per_point_x" + std::to_string(multiplicity) + ".csv"));
      std::ofstream dedup_csv(directory / ("endpoint_x" + std::to_string(multiplicity) + ".csv"));
      const char *header = "timestamp,data_size,update_cost,get_map_cost,ray_mode,valid_points,"
                           "candidate_rays,ray_count,touched_voxels\n";
      if (!base_csv || !dedup_csv)
        throw std::runtime_error("Cannot open benchmark output CSV");
      base_csv << header;
      dedup_csv << header;
      double base_total = 0, dedup_total = 0;
      auto measure = [&](mapping::Mapper &mapper, std::ofstream &csv, int frame, bool mode) {
        auto start = Clock::now();
        mapper.Update(input);
        const double update_ms = Milliseconds(start);
        PointCloudType cloud;
        start = Clock::now();
        mapper.GetOccupiedCloud(cloud);
        const double get_ms = Milliseconds(start);
        Record(csv, frame * 0.1, input.lidar_cloud->size(), update_ms, get_ms, mapper, mode);
        return update_ms;
      };
      for (int i = 0; i < repeats; ++i) {
        // Alternate order to reduce systematic ordering bias.
        if (i % 2) {
          dedup_total += measure(dedup, dedup_csv, i, true);
          base_total += measure(baseline, base_csv, i, false);
        } else {
          base_total += measure(baseline, base_csv, i, false);
          dedup_total += measure(dedup, dedup_csv, i, true);
        }
      }
      base_csv.flush();
      dedup_csv.flush();
      if (!base_csv || !dedup_csv)
        throw std::runtime_error("Failed to write benchmark CSV");
      std::cout << "Synthetic multiplicity=" << multiplicity
                << ", points=" << input.lidar_cloud->size()
                << ", rays=" << baseline.GetUpdateDiagnostics().ray_count << " -> "
                << dedup.GetUpdateDiagnostics().ray_count
                << ", mean update=" << base_total / repeats << " -> "
                << dedup_total / repeats << " ms, speedup=" << base_total / dedup_total << "x\n";
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
