#ifndef POINTWISE_HEURISTICS_H_
#define POINTWISE_HEURISTICS_H_

#include <wavemap/core/common.h>
#include <wavemap/core/config/type_selector.h>

namespace wavemap {
struct PointwiseHeuristicType : TypeSelector<PointwiseHeuristicType> {
  using TypeSelector::TypeSelector;

  enum Id : TypeId { kEuclidean, kOctile };

  static constexpr std::array names = {"euclidean", "octile"};
};

FloatingPoint computeHScore(const PointwiseHeuristicType& heuristic_type,
                            const Point3D& node_center, const Point3D& goal);

inline FloatingPoint computeFScore(const PointwiseHeuristicType& heuristic_type,
                                   const Point3D& node_center,
                                   FloatingPoint node_g_score,
                                   const Point3D& goal) {
  const FloatingPoint node_h_score =
      computeHScore(heuristic_type, node_center, goal);
  return node_g_score + node_h_score;
}
}  // namespace wavemap

#endif  // POINTWISE_HEURISTICS_H_
