#include "pointwise_heuristics.h"

namespace wavemap {
FloatingPoint computeHScore(const PointwiseHeuristicType& heuristic_type,
                            const Point3D& node_center, const Point3D& goal) {
  if (heuristic_type == PointwiseHeuristicType::kOctile) {
    const Vector3D d = (node_center - goal).cwiseAbs();
    const FloatingPoint d_min = d.minCoeff();
    const FloatingPoint d_max = d.maxCoeff();
    const FloatingPoint d_mid = d.sum() - d_min - d_max;
    constexpr FloatingPoint D1 = 1.f;
    constexpr FloatingPoint D2 = constants<FloatingPoint>::kSqrt2;
    constexpr FloatingPoint D3 = constants<FloatingPoint>::kSqrt3;
    return (D3 - D2) * d_min + (D2 - D1) * d_mid + D1 * d_max;
  } else {
    return (node_center - goal).norm();
  }
}
}  // namespace wavemap
