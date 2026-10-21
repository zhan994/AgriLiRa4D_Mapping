#include "mapper/mapper.h"

#include <stdexcept>

namespace mapping {

Mapper::Mapper(const Options &options) : options_(options) {
  if (!std::isfinite(options_.resolution) || options_.resolution <= 0.0 ||
      !std::isfinite(1.0 / options_.resolution) ||
      !std::isfinite(options_.max_range) || options_.max_range <= 0.0 ||
      !std::isfinite(options_.hit_prob) || options_.hit_prob < 0.5 ||
      options_.hit_prob >= 1.0 || !std::isfinite(options_.miss_prob) ||
      options_.miss_prob <= 0.0 || options_.miss_prob > 0.5 ||
      !std::isfinite(options_.occupancy_threshold) ||
      options_.occupancy_threshold <= 0.0 || options_.occupancy_threshold >= 1.0)
    throw std::invalid_argument("Invalid mapper resolution, range or probabilities");
}

bool IsValidPoint(const double &x, const double &y, const double &z) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  const float MAX_RANGE = 100000.0f;
  if (fabs(x) > MAX_RANGE || fabs(y) > MAX_RANGE || fabs(z) > MAX_RANGE)
    return false;

  return true;
}
} // namespace mapping
