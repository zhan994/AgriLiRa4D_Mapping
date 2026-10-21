#include "mapper/mapper_bonxai.h"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <limits>

namespace mapping {
namespace {
float LogOdds(double probability) {
  return static_cast<float>(std::log(probability / (1.0 - probability)));
}

// Match the float coordinates and direction arithmetic used by OctoMapper.
float RayLength(const V3F &direction) {
  return static_cast<float>(std::sqrt(static_cast<double>(
      direction.x() * direction.x() + direction.y() * direction.y() +
      direction.z() * direction.z())));
}

bool ToCoord(const V3F &point, double resolution, Bonxai::CoordT &coord) {
  if (!IsValidPoint(point.x(), point.y(), point.z()))
    return false;
  const double inv_resolution = 1.0 / resolution;
  for (unsigned axis = 0; axis < 3; ++axis) {
    const double index = std::floor(point[axis] * inv_resolution);
    // Leave a one-cell margin for DDA steps at numerical boundaries.
    if (index <= std::numeric_limits<int32_t>::min() ||
        index >= std::numeric_limits<int32_t>::max())
      return false;
    coord[axis] = static_cast<int32_t>(index);
  }
  return true;
}

// Amanatides-Woo traversal to the actual endpoint, including the origin voxel
// and excluding the endpoint voxel. Axis ties and the distance guard follow
// OctoMap's computeRayKeys(), rather than aiming at a voxel center.
template <typename Visitor>
void TraceRay(const V3F &origin, const V3F &end,
              const Bonxai::CoordT &origin_coord,
              const Bonxai::CoordT &end_coord, double resolution,
              const Visitor &visit) {
  if (origin_coord == end_coord)
    return;
  visit(origin_coord);
  V3F direction = end - origin;
  const float length = RayLength(direction);
  if (length <= 0.0f)
    return;
  direction /= length;
  Bonxai::CoordT current = origin_coord;
  int step[3];
  double t_max[3], t_delta[3];
  for (unsigned axis = 0; axis < 3; ++axis) {
    step[axis] = direction[axis] > 0.0f ? 1 : direction[axis] < 0.0f ? -1 : 0;
    if (step[axis]) {
      double border = (static_cast<double>(current[axis]) + 0.5) * resolution;
      border += static_cast<float>(step[axis] * resolution * 0.5);
      t_max[axis] = (border - origin[axis]) / direction[axis];
      t_delta[axis] = resolution / std::fabs(direction[axis]);
    } else {
      t_max[axis] = t_delta[axis] = std::numeric_limits<double>::max();
    }
  }
  while (true) {
    unsigned axis;
    if (t_max[0] < t_max[1])
      axis = t_max[0] < t_max[2] ? 0 : 2;
    else
      axis = t_max[1] < t_max[2] ? 1 : 2;
    current[axis] += step[axis];
    t_max[axis] += t_delta[axis];
    if (current == end_coord ||
        std::min({t_max[0], t_max[1], t_max[2]}) > length)
      return;
    visit(current);
  }
}
} // namespace

BonxaiMapper::BonxaiMapper(const Options &options)
    : Mapper(options), grid_(options_.resolution),
      hit_log_(LogOdds(options.hit_prob)),
      miss_log_(LogOdds(options.miss_prob)),
      occupancy_log_(LogOdds(options.occupancy_threshold)),
      clamp_min_log_(LogOdds(0.1192)), clamp_max_log_(LogOdds(0.971)) {}

void BonxaiMapper::Update(const MapperInput &input) {
  if (!input.lidar_cloud || input.lidar_cloud->empty()) {
    ROS_WARN("MapperInput has no LiDAR data.");
    return;
  }
  const V3F origin = input.lidar_origin.cast<float>();
  Bonxai::CoordT origin_coord;
  if (!ToCoord(origin, options_.resolution, origin_coord)) {
    ROS_WARN("BonxaiMapper: Invalid LiDAR origin point.");
    return;
  }

  auto accessor = grid_.createAccessor();
  std::vector<Cell *> touched;
  auto mark = [&](const Bonxai::CoordT &coord, uint8_t flag) {
    Cell *cell = accessor.value(coord, true);
    if (cell->flag == Cell::Unseen)
      touched.push_back(cell);
    // Hit wins regardless of the order of endpoints and traversed cells.
    cell->flag = std::max(cell->flag, flag);
  };
  for (const auto &point : input.lidar_cloud->points) {
    V3F end(point.x, point.y, point.z);
    if (!IsValidPoint(point.x, point.y, point.z))
      continue;
    const V3F direction = end - origin;
    const float length = RayLength(direction);
    if (length > options_.max_range) {
      const V3F unit = direction / length;
      end = origin + unit * static_cast<float>(options_.max_range);
    }
    Bonxai::CoordT end_coord;
    if (!ToCoord(end, options_.resolution, end_coord))
      continue;
    TraceRay(origin, end, origin_coord, end_coord, options_.resolution,
             [&](const Bonxai::CoordT &coord) { mark(coord, Cell::Free); });
    if (length < options_.max_range)
      mark(end_coord, Cell::Hit);
  }

  // Leaf allocations keep their addresses across insertions. These pointers
  // avoid another grid lookup while applying one update per touched voxel.
  for (Cell *cell : touched) {
    cell->log_odds = std::clamp(
        cell->log_odds + (cell->flag == Cell::Hit ? hit_log_ : miss_log_),
        clamp_min_log_, clamp_max_log_);
    cell->flag = Cell::Unseen;
  }
}

void BonxaiMapper::Reset() { grid_.clear(Bonxai::CLEAR_MEMORY); }

void BonxaiMapper::GetOccupiedCloud(PointCloudType &output) const {
  output.clear();
  output.header.frame_id = "world";
  output.is_dense = true;
  grid_.forEachCell([&](const Cell &cell, const Bonxai::CoordT &coord) {
    if (cell.log_odds < occupancy_log_)
      return;
    PointType point{};
    point.x = (static_cast<double>(coord.x) + 0.5) * options_.resolution;
    point.y = (static_cast<double>(coord.y) + 0.5) * options_.resolution;
    point.z = (static_cast<double>(coord.z) + 0.5) * options_.resolution;
    output.push_back(point);
  });
}

bool BonxaiMapper::Save(const std::string &path) const {
  PointCloudType cloud;
  GetOccupiedCloud(cloud);
  try {
    return pcl::io::savePCDFileBinary(path, cloud) == 0;
  } catch (const pcl::IOException &error) {
    ROS_ERROR("BonxaiMapper: Failed to save occupied cloud: %s", error.what());
    return false;
  }
}
} // namespace mapping
