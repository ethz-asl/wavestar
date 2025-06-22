#ifndef A_STAR_H_
#define A_STAR_H_

#include <limits>
#include <utility>
#include <vector>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/neighbors/grid_neighborhood.h>

#include "planner_base.h"
#include "pointwise_heuristics.h"
#include "utils/containers.h"

namespace wavemap {
/**
 * Config struct for A* planner.
 */
struct AStarConfig : ConfigBase<AStarConfig, 1, PointwiseHeuristicType> {
  PointwiseHeuristicType heuristic_type = PointwiseHeuristicType::kOctile;

  static MemberMap memberMap;

  // Constructors
  AStarConfig() = default;
  explicit AStarConfig(PointwiseHeuristicType heuristic_type)
      : heuristic_type(std::move(heuristic_type)) {}

  bool isValid(bool verbose) const override;
};

class AStar : public PlannerBase {
 public:
  AStar(const AStarConfig& config, ClassifiedMap::ConstPtr classified_map)
      : PlannerBase(std::move(classified_map)), config_(config.checkValid()) {}

  void clear() override;

  std::vector<Point3D> searchPath(const Point3D& start,
                                  const Point3D& goal) override;

  bool solve(const Point3D& start, const Point3D& goal);

  std::vector<Point3D> getPath() const;
  const CostField& getCostField() const { return cost_field_; }

 private:
  using PriorityQueue = min_priority_queue<FScoreAndIndex>;

  const AStarConfig config_;

  Index3D start_index_ = Index3D::Zero();
  Index3D goal_index_ = Index3D::Zero();

  CostField cost_field_{GScoreAndPredecessor{
      std::numeric_limits<FloatingPoint>::max(),
      Index3D::Constant(std::numeric_limits<IndexElement>::max()), false}};
  PriorityQueue open_queue_;

  const FloatingPoint min_cell_width_ = classified_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;

  inline static const auto kNeighborIndexOffsets =
      GridNeighborhood<3>::generateIndexOffsets<Adjacency::kAnyDisjoint>();
  const std::array<FloatingPoint, kNeighborIndexOffsets.size()>
      neighbor_distance_offsets_ = GridNeighborhood<3>::computeOffsetLengths(
          kNeighborIndexOffsets, min_cell_width_);
};
}  // namespace wavemap

#endif  // A_STAR_H_
