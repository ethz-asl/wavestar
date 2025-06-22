#include "volumetric_heuristics.h"

#include <algorithm>

#include "wavemap/core/data_structure/aabb.h"
#include "wavemap/core/indexing/index_conversions.h"

namespace wavemap {
FloatingPoint computeFScore(const OctreeIndex& predecessor_index,
                            FloatingPoint predecessor_g_score,
                            const OctreeIndex& node_index, const Point3D& goal,
                            FloatingPoint min_cell_width,
                            FloatingPoint node_negative_padding) {
  const Point3D predecessor_center =
      convert::nodeIndexToCenterPoint(predecessor_index, min_cell_width);
  auto node_aabb = convert::nodeIndexToAABB(node_index, min_cell_width);
  if (node_aabb.containsPoint(goal)) {
    const FloatingPoint d_predecessor_goal = (goal - predecessor_center).norm();
    const FloatingPoint goal_g_score = predecessor_g_score + d_predecessor_goal;
    constexpr FloatingPoint kGoalHScore = 0.f;
    return goal_g_score + kGoalHScore;
  }

  FloatingPoint center_f_score{};
  const Point3D node_center =
      convert::nodeIndexToCenterPoint(node_index, min_cell_width);
  const FloatingPoint d_predecessor_center =
      (node_center - predecessor_center).norm();
  const FloatingPoint center_g_score =
      predecessor_g_score + d_predecessor_center;
  const FloatingPoint center_h_score = (goal - node_center).norm();
  center_f_score = center_g_score + center_h_score;

  constexpr int kNumAabbCorners = AABB<Point3D>::kNumCorners;
  node_aabb.min.array() += node_negative_padding;
  node_aabb.max.array() -= node_negative_padding;
  Eigen::Matrix<FloatingPoint, kNumAabbCorners, 1> corner_f_scores;
  for (int corner_idx = 0; corner_idx < kNumAabbCorners; ++corner_idx) {
    const Point3D corner = node_aabb.corner_point(corner_idx);
    const FloatingPoint d_predecessor_corner =
        (corner - predecessor_center).norm();
    const FloatingPoint corner_g_score =
        predecessor_g_score + d_predecessor_corner;
    const FloatingPoint corner_h_score = (goal - corner).norm();
    corner_f_scores[corner_idx] = corner_g_score + corner_h_score;
  }

  return std::min(corner_f_scores.minCoeff(), center_f_score);
}
}  // namespace wavemap
