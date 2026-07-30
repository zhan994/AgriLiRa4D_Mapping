#include "mapper/mapper.h"

namespace mapping {

bool IsValidPoint(const double &x, const double &y, const double &z) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  const float MAX_RANGE = 100000.0f;
  if (fabs(x) > MAX_RANGE || fabs(y) > MAX_RANGE || fabs(z) > MAX_RANGE)
    return false;

  return true;
}
} // namespace mapping