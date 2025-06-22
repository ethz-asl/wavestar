#ifndef MULTI_RES_THETA_STAR_H_
#define MULTI_RES_THETA_STAR_H_

#include <utility>
#include <vector>

#include <wavemap/core/common.h>
#include <wavemap/core/indexing/ndtree_index.h>
#include <wavemap/core/utils/query/query_accelerator.h>

#include "planner_base.h"
#include "utils/containers.h"
#include "utils/geometry.h"

namespace wavemap {
/**
 * Config struct for Multi-Resolution Theta* planner.
 */
struct MultiResThetaStarConfig : ConfigBase<MultiResThetaStarConfig, 2> {
  FloatingPoint refinement_epsilon = 0.01f;
  IndexElement initialization_min_height = 0;

  static MemberMap memberMap;

  // Constructors
  MultiResThetaStarConfig() = default;
  MultiResThetaStarConfig(FloatingPoint max_relative_error,
                          IndexElement initialization_height)
      : refinement_epsilon(max_relative_error),
        initialization_min_height(initialization_height) {}

  bool isValid(bool verbose) const override;
};

class MultiResThetaStar : public PlannerBase {
 public:
  MultiResThetaStar(const MultiResThetaStarConfig& config,
                    ClassifiedMap::ConstPtr classified_map)
      : PlannerBase(std::move(classified_map)), config_(config.checkValid()) {
    CHECK_LE(config_.initialization_min_height,
             classified_map_->getTreeHeight());
  }

  void clear() override;

  std::vector<Point3D> searchPath(const Point3D& start,
                                  const Point3D& goal) override;

  bool solve(const Point3D& start, const Point3D& goal);

  std::vector<Point3D> getPath() const;
  const MultiResCostField& getCostField() const { return cost_field_raw_; }

 private:
  using PriorityQueue = min_priority_queue<FScoreAndMultiResIndex>;

  const MultiResThetaStarConfig config_;

  OctreeIndex start_index_{};
  OctreeIndex goal_index_{};

  const IndexElement tree_height_ = classified_map_->getTreeHeight();
  const FloatingPoint min_cell_width_ = classified_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;
  const FloatingPoint half_cell_width_at_min_height_ = 0.49f * min_cell_width_;
  static constexpr FloatingPoint kAbsolutePrecision = 0.0001f;

  MultiResCostField cost_field_raw_{tree_height_};
  QueryAccelerator<MultiResCostField> cost_field_{cost_field_raw_};
  PriorityQueue open_queue_;

  void seedBlock(const OctreeIndex& block_index);
  void recursiveSeedBlock(const OctreeIndex& sink_index,
                          const OctreeIndex& neighbor_index,
                          const ClassifiedMap::Node& neighbor_node);

  bool propagate(const OctreeIndex& goal_index, const Point3D& goal);

  void push(const OctreeIndex& source_index,
            const MultiResCostField::Node::DataType& source_node,
            const Point3D& goal);
  void recursivePush(const OctreeIndex& source_index,
                     const MultiResCostField::Node::DataType& source_node,
                     const OctreeIndex& neighbor_index,
                     MultiResCostField::Node& neighbor_node,
                     Occupancy::Mask neighbor_occupancy_mask,
                     const ClassifiedMap::Node* neighbor_occupancy_node,
                     bool neighbor_not_yet_in_queue, const Point3D& goal);

  void addToOpenQueue(const OctreeIndex& node_index,
                      MultiResCostField::Node& node, const Point3D& goal);

  FloatingPoint computeGScore(FloatingPoint predecessor_g_score,
                              const OctreeIndex& predecessor_index,
                              const OctreeIndex& node_index) const {
    return predecessor_g_score + distanceBetweenCenters(predecessor_index,
                                                        node_index,
                                                        min_cell_width_);
  }

  bool hasLineOfSight(const OctreeIndex& node_index,
                      const OctreeIndex& predecessor_index);

  enum class UpdateAction { kOld, kSourcePredecessor, kSourceDirect, kRefine };
  UpdateAction determineUpdateAction(
      const OctreeIndex& source_index, FloatingPoint& source_g_score,
      const MultiResCostField::Node::DataType& source_node,
      const OctreeIndex& neighbor_index,
      const MultiResCostField::Node::DataType& neighbor_node,
      bool cannot_refine);
};
}  // namespace wavemap

#endif  // MULTI_RES_THETA_STAR_H_
