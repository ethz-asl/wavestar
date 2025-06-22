#include "multi_res_lazy_theta_star.h"

#include <limits>
#include <utility>
#include <vector>

#include <wavemap/core/utils/iterate/grid_iterator.h>
#include <wavemap/core/utils/iterate/ray_iterator.h>
#include <wavemap/core/utils/neighbors/ndtree_adjacency.h>
#include <wavemap/core/utils/profile/profiler_interface.h>

#include "utils/ndtree_grid.h"
#include "volumetric_heuristics.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(MultiResLazyThetaStarConfig,
                       (refinement_epsilon)
                       (initialization_min_height));

bool MultiResLazyThetaStarConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(refinement_epsilon, 0.f, verbose);
  is_valid &= IS_PARAM_GE(initialization_min_height, 0, verbose);

  return is_valid;
}

void MultiResLazyThetaStar::clear() {
  ProfilerZoneScoped;
  start_index_ = OctreeIndex{};
  goal_index_ = OctreeIndex{};
  cost_field_raw_.clear();
  cost_field_.reset();
  open_queue_ = PriorityQueue{};
}

std::vector<Point3D> MultiResLazyThetaStar::searchPath(const Point3D& start,
                                                       const Point3D& goal) {
  ProfilerZoneScoped;
  if (solve(start, goal)) {
    return getPath();
  }
  return {};
}

bool MultiResLazyThetaStar::solve(const Point3D& start, const Point3D& goal) {
  ProfilerZoneScoped;
  // Make sure the previous solution has been cleared
  clear();

  // Check that the start and end positions are collision free
  start_index_ =
      OctreeIndex{0, convert::pointToNearestIndex(start, min_cell_width_inv_)};
  goal_index_ =
      OctreeIndex{0, convert::pointToNearestIndex(goal, min_cell_width_inv_)};
  for (const auto& [name, index] :
       {std::tuple{"start", start_index_}, {"goal", goal_index_}}) {
    if (!classified_map_->isFully(index, Occupancy::kFree)) {
      LOG(WARNING) << "Collision free path can not be found: " << name
                   << " is not fully free";
      return false;
    }
  }

  // Pad all obstacles with high res cells (inflection points)
  seedBlock(start_index_.computeParentIndex(tree_height_));

  // Initialize the cost field and queue
  cost_field_raw_.getOrAllocateNode(goal_index_);
  auto& start_node = cost_field_raw_.getOrAllocateNode(start_index_);
  start_node.data().predecessor_index = start_index_;
  start_node.data().predecessor_g_score = 0.f;
  addToOpenQueue(start_index_, start_node, goal);

  // Propagate the wavefront
  return propagate(goal_index_, goal);
}

std::vector<Point3D> MultiResLazyThetaStar::getPath() const {
  ProfilerZoneScoped;
  std::vector<Point3D> path;
  OctreeIndex current_index = goal_index_;
  while (true) {
    if (current_index == cost_field_raw_.getDefaultValue().predecessor_index) {
      return {};
    }
    Point3D center =
        convert::nodeIndexToCenterPoint(current_index, min_cell_width_);
    path.emplace_back(std::move(center));
    if (current_index == start_index_) {
      std::reverse(path.begin(), path.end());
      return path;
    }
    const auto [current_node, current_node_height] =
        cost_field_raw_.getValueOrAncestor(current_index);
    if (!current_node) {
      return {};
    }
    if (!current_node->closed) {
      LOG(WARNING)
          << "Encountered non-closed node ("
          << current_index.computeParentIndex(current_node_height).toString()
          << ") while extracting shortest path";
    }
    current_index = current_node->predecessor_index;
  }
}

void MultiResLazyThetaStar::seedBlock(const OctreeIndex& block_index) {
  ProfilerZoneScoped;
  const auto [min_block_index, max_block_index] =
      minMaxNeighborsAtHeight(block_index, tree_height_);
  for (const OctreeIndex& neighbor_block_index :
       NdtreeGrid(min_block_index, max_block_index)) {
    if (const auto* neighbor_block =
            classified_map_->getBlock(neighbor_block_index.position);
        neighbor_block) {
      recursiveSeedBlock(block_index, neighbor_block_index,
                         neighbor_block->getRootNode());
    }
  }
}

void MultiResLazyThetaStar::recursiveSeedBlock(  // NOLINT
    const OctreeIndex& sink_index, const OctreeIndex& neighbor_index,
    const ClassifiedMap::Node& neighbor_node) {
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    // If the child does not contain any occupied regions, skip it
    const auto child_occupancy =
        neighbor_node.data().childOccupancyMask(child_idx);
    if (!OccupancyClassifier::has(child_occupancy, Occupancy::kOccupied)) {
      continue;
    }

    // If the child is not adjacent to the sink_index (block to seed), skip it
    const OctreeIndex child_index = neighbor_index.computeChildIndex(child_idx);
    if (!areAdjacent(sink_index, child_index)) {
      continue;
    }

    // If the child is fully occupied, process it at the current resolution
    if (OccupancyClassifier::isFully(child_occupancy, Occupancy::kOccupied) ||
        child_index.height <= config_.initialization_min_height) {
      // Iterate over all the neighbors of the occupied child
      const auto min_neighbor_index = child_index.position.array() - 1;
      const auto max_neighbor_index = child_index.position.array() + 1;
      for (const Index3D& neighbor_index_pos :
           Grid<3>(min_neighbor_index, max_neighbor_index)) {
        const OctreeIndex child_neighbor_index{child_index.height,
                                               neighbor_index_pos};
        // Don't seed the occupied child itself
        // or indices outside the block to seed
        const bool is_outside_block =
            child_neighbor_index.computeParentIndex(sink_index.height)
                .position != sink_index.position;
        if (is_outside_block || child_neighbor_index == child_index) {
          continue;
        }
        // If the neighbor is fully free, it is an inflection point candidate
        if (classified_map_->isFully(child_neighbor_index, Occupancy::kFree)) {
          // Allocate the node in the cost field to force the wavefront to pass
          // through it at the current (child_index) resolution to evaluate it
          // at this resolution
          cost_field_.getOrAllocateNode(child_neighbor_index);
        }
      }
      continue;
    }

    // If the child has children, recurse
    if (const ClassifiedMap::Node* child_node =
            neighbor_node.getChild(child_idx);
        child_node) {
      recursiveSeedBlock(sink_index, child_index, *child_node);
    }
  }
}

bool MultiResLazyThetaStar::propagate(const OctreeIndex& goal_index,
                                      const Point3D& goal) {
  ProfilerZoneScoped;
  while (!open_queue_.empty()) {
    ProfilerPlot("QueueLength", static_cast<int64_t>(open_queue_.size()));
    const OctreeIndex node_index = open_queue_.top().node_index;
    auto& node = *CHECK_NOTNULL(open_queue_.top().node);
    open_queue_.pop();

    if (node.data().closed || node.hasAtLeastOneChild()) {
      continue;
    }
    node.data().closed = true;
    ProfilerPlot("Height", static_cast<float>(node_index.height));

    if (!checkNode(node_index, node.data())) {
      continue;
    }

    if (node_index == goal_index.computeParentIndex(node_index.height)) {
      return true;
    }

    push(node_index, node.data(), goal);
  }

  return false;
}

bool MultiResLazyThetaStar::checkNode(const OctreeIndex& node_index,
                                      GScoreAndMultiResPredecessor& node) {
  if (!hasLineOfSight(node_index, node.predecessor_index)) {
    // Reset the node
    node = GScoreAndMultiResPredecessor{};
    // Visit its neighbors to search a new direct predecessor with min g_cost
    pull(node_index, node);
    if (node.predecessor_index ==
        GScoreAndMultiResPredecessor{}.predecessor_index) {
      LOG(WARNING) << "Rewiring failed. This shouldn't happen.";
      return false;
    }
    node.closed = true;
  }
  return true;
}

void MultiResLazyThetaStar::push(
    const OctreeIndex& source_index,
    const MultiResCostField::Node::DataType& source_node, const Point3D& goal) {
  const auto [min_block_index, max_block_index] =
      minMaxNeighborsAtHeight(source_index, tree_height_);
  for (const OctreeIndex& neighbor_index :
       NdtreeGrid(min_block_index, max_block_index)) {
    const ClassifiedMap::Node* neighbor_occupancy_node =
        classified_map_->getNode(neighbor_index);
    if (!neighbor_occupancy_node) {
      continue;
    }
    const auto neighbor_occupancy_mask =
        neighbor_occupancy_node->data().occupancyMask();
    if (!OccupancyClassifier::has(neighbor_occupancy_mask, Occupancy::kFree)) {
      continue;
    }
    auto* neighbor_block = cost_field_.getBlock(neighbor_index.position);
    if (!neighbor_block) {
      seedBlock(neighbor_index);
      neighbor_block = &cost_field_.getOrAllocateBlock(neighbor_index.position);
    }
    auto& neighbor_node = neighbor_block->getRootNode();
    if (neighbor_node.data().closed) {
      continue;
    }
    recursivePush(source_index, source_node, neighbor_index, neighbor_node,
                  neighbor_occupancy_mask, neighbor_occupancy_node, true, goal);
  }
}

void MultiResLazyThetaStar::recursivePush(  // NOLINT
    const OctreeIndex& source_index,
    const MultiResCostField::Node::DataType& source_node,
    const OctreeIndex& neighbor_index, MultiResCostField::Node& neighbor_node,
    Occupancy::Mask neighbor_occupancy_mask,
    const ClassifiedMap::Node* neighbor_occupancy_node,
    bool neighbor_not_yet_in_queue, const Point3D& goal) {
  // If max res has been reached and node is not fully free, mark it occupied
  DCHECK_GE(neighbor_index.height, 0);
  const bool min_height_reached = neighbor_index.height <= 0;
  const bool neighbor_is_fully_free =
      OccupancyClassifier::isFully(neighbor_occupancy_mask, Occupancy::kFree);
  if (min_height_reached && !neighbor_is_fully_free) {
    LOG(WARNING)
        << "Expanded non-free node at min_height. This shouldn't happen.";
    neighbor_node.data() = GScoreAndMultiResPredecessor{};
    neighbor_node.data().closed = true;
    return;
  }

  const bool neighbor_is_free_leaf =
      neighbor_is_fully_free && !neighbor_node.hasAtLeastOneChild();
  if (min_height_reached || neighbor_is_free_leaf) {
    const auto action = determineUpdateAction(
        source_node, neighbor_index, neighbor_node.data(), min_height_reached);
    DCHECK(source_node.closed);
    if (action == UpdateAction::kSourcePredecessor) {
      neighbor_node.data().predecessor_index = source_node.predecessor_index;
      neighbor_node.data().predecessor_g_score =
          source_node.predecessor_g_score;
      addToOpenQueue(neighbor_index, neighbor_node, goal);
      return;
    }
    if (min_height_reached || action == UpdateAction::kOld) {
      if (neighbor_not_yet_in_queue) {
        addToOpenQueue(neighbor_index, neighbor_node, goal);
      }
      return;
    }
  }

  for (NdtreeIndexRelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    Occupancy::Mask child_occupancy_mask = neighbor_occupancy_mask;
    const ClassifiedMap::Node* child_occupancy_node = nullptr;
    if (neighbor_occupancy_node) {
      child_occupancy_mask = neighbor_occupancy_node->data().childOccupancyMask(
          relative_child_idx);
      if (!OccupancyClassifier::has(child_occupancy_mask, Occupancy::kFree)) {
        continue;
      }
      child_occupancy_node =
          neighbor_occupancy_node->getChild(relative_child_idx);
    }

    const OctreeIndex child_index =
        neighbor_index.computeChildIndex(relative_child_idx);
    const bool child_not_adjacent = !areAdjacent(child_index, source_index);
    auto* child_node = neighbor_node.getChild(relative_child_idx);
    bool child_is_new = false;
    if (!child_node) {
      child_is_new = true;
      child_node = &neighbor_node.getOrAllocateChild(relative_child_idx,
                                                     neighbor_node.data());
      if (child_not_adjacent) {
        if (!cost_field_raw_.equalsDefaultValue(neighbor_node.data())) {
          addToOpenQueue(child_index, *child_node, goal);
        }
        continue;
      }
    } else if (child_not_adjacent || neighbor_index == source_index ||
               child_node->data().closed) {
      continue;
    }

    recursivePush(source_index, source_node, child_index, *child_node,
                  child_occupancy_mask, child_occupancy_node, child_is_new,
                  goal);
  }
}

void MultiResLazyThetaStar::pull(const OctreeIndex& sink_index,
                                 MultiResCostField::Node::DataType& sink_node) {
  FloatingPoint sink_g_score = std::numeric_limits<FloatingPoint>::max();
  const auto [min_block_index, max_block_index] =
      minMaxNeighborsAtHeight(sink_index, tree_height_);
  for (const OctreeIndex& neighbor_index :
       NdtreeGrid(min_block_index, max_block_index)) {
    if (const auto* neighbor_node = cost_field_.getNode(neighbor_index);
        neighbor_node) {
      recursivePull(sink_index, sink_node, sink_g_score, neighbor_index,
                    *neighbor_node);
    }
  }
}

void MultiResLazyThetaStar::recursivePull(  // NOLINT
    const OctreeIndex& sink_index, MultiResCostField::Node::DataType& sink_node,
    FloatingPoint& sink_g_score, const OctreeIndex& neighbor_index,
    const MultiResCostField::Node& neighbor_node) const {
  const bool neighbor_is_leaf = !neighbor_node.hasAtLeastOneChild();
  if (neighbor_is_leaf && neighbor_node.data().closed) {
    const FloatingPoint neighbor_g_score =
        computeGScore(neighbor_node.data().predecessor_g_score,
                      neighbor_node.data().predecessor_index, neighbor_index);
    const FloatingPoint sink_candidate_g_score =
        computeGScore(neighbor_g_score, neighbor_index, sink_index);
    if (sink_candidate_g_score < sink_g_score) {
      sink_g_score = sink_candidate_g_score;
      sink_node.predecessor_index = neighbor_index;
      sink_node.predecessor_g_score = neighbor_g_score;
    }
    return;
  }

  for (NdtreeIndexRelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    if (auto* child_node = neighbor_node.getChild(relative_child_idx);
        child_node) {
      const OctreeIndex child_index =
          neighbor_index.computeChildIndex(relative_child_idx);
      if (!areAdjacent(child_index, sink_index)) {
        continue;
      }
      recursivePull(sink_index, sink_node, sink_g_score, child_index,
                    *child_node);
    }
  }
}

void MultiResLazyThetaStar::addToOpenQueue(const OctreeIndex& node_index,
                                           MultiResCostField::Node& node,
                                           const Point3D& goal) {
  DCHECK(!node.data().closed);
  DCHECK(node.data().predecessor_g_score !=
         GScoreAndMultiResPredecessor{}.predecessor_g_score);
  DCHECK(node.data().predecessor_index !=
         GScoreAndMultiResPredecessor{}.predecessor_index);

  const FloatingPoint f_score = computeFScore(
      node.data().predecessor_index, node.data().predecessor_g_score,
      node_index, goal, min_cell_width_, half_min_cell_width_);
  open_queue_.emplace(FScoreAndMultiResIndex{f_score, node_index, &node});
}

bool MultiResLazyThetaStar::hasLineOfSight(
    const OctreeIndex& node_index, const OctreeIndex& predecessor_index) {
  const Point3D node_center =
      convert::nodeIndexToCenterPoint(node_index, min_cell_width_);
  const Point3D predecessor_center =
      convert::nodeIndexToCenterPoint(predecessor_index, min_cell_width_);
  const Ray ray(node_center, predecessor_center, min_cell_width_);
  int step_count = 0;
  constexpr int kMaxNumSteps = 1000;
  for (const Index3D& ray_voxel_index : ray) {
    const OctreeIndex ray_node_index{0, ray_voxel_index};
    if (!classified_map_->isFully(ray_node_index, Occupancy::kFree) ||
        kMaxNumSteps < ++step_count) {
      return false;
    }
  }
  return true;
}

MultiResLazyThetaStar::UpdateAction
MultiResLazyThetaStar::determineUpdateAction(
    const MultiResCostField::Node::DataType& source_node,
    const OctreeIndex& neighbor_index,
    const MultiResCostField::Node::DataType& neighbor_node,
    bool cannot_refine) const {
  if (neighbor_node.predecessor_index == source_node.predecessor_index) {
    return UpdateAction::kOld;
  }

  const auto current_predecessor_center = convert::nodeIndexToCenterPoint(
      neighbor_node.predecessor_index, min_cell_width_);
  const auto source_predecessor_center = convert::nodeIndexToCenterPoint(
      source_node.predecessor_index, min_cell_width_);
  if (cannot_refine) {
    const auto neighbor_center =
        convert::nodeIndexToCenterPoint(neighbor_index, min_cell_width_);
    const FloatingPoint d_current_predecessor_neighbor =
        (neighbor_center - current_predecessor_center).norm();
    const FloatingPoint d_source_predecessor_neighbor =
        (neighbor_center - source_predecessor_center).norm();
    const FloatingPoint neighbor_g_score_current_predecessor =
        neighbor_node.predecessor_g_score + d_current_predecessor_neighbor;
    const FloatingPoint neighbor_g_score_source_predecessor =
        source_node.predecessor_g_score + d_source_predecessor_neighbor;
    if (neighbor_g_score_current_predecessor <
        neighbor_g_score_source_predecessor + kEpsilon) {
      return UpdateAction::kOld;
    }
    return UpdateAction::kSourcePredecessor;
  }

  auto neighbor_aabb =
      convert::nodeIndexToAABB(neighbor_index, min_cell_width_);
  neighbor_aabb.min.array() += half_min_cell_width_;
  neighbor_aabb.max.array() -= half_min_cell_width_;

  constexpr int kNumAabbCorners = AABB<Point3D>::kNumCorners;
  Eigen::Array<FloatingPoint, kNumAabbCorners, 1> d_current_predecessor_corner;
  Eigen::Array<FloatingPoint, kNumAabbCorners, 1> d_source_predecessor_corner;
  for (int corner_idx = 0; corner_idx < kNumAabbCorners; ++corner_idx) {
    const Point3D corner = neighbor_aabb.corner_point(corner_idx);
    d_current_predecessor_corner[corner_idx] =
        (corner - current_predecessor_center).norm();
    d_source_predecessor_corner[corner_idx] =
        (corner - source_predecessor_center).norm();
  }
  const Eigen::Array<FloatingPoint, kNumAabbCorners, 1>
      g_scores_current_predecessor =
          neighbor_node.predecessor_g_score + d_current_predecessor_corner;
  const Eigen::Array<FloatingPoint, kNumAabbCorners, 1>
      g_scores_source_predecessor =
          source_node.predecessor_g_score + d_source_predecessor_corner;

  if ((g_scores_current_predecessor < g_scores_source_predecessor + kEpsilon)
          .all()) {
    return UpdateAction::kOld;
  }
  if ((g_scores_source_predecessor < g_scores_current_predecessor + kEpsilon)
          .all()) {
    return UpdateAction::kSourcePredecessor;
  }

  const bool current_predecessor_acceptable =
      (g_scores_current_predecessor - g_scores_source_predecessor <
       d_current_predecessor_corner * config_.refinement_epsilon +
           kAbsolutePrecision)
          .all();
  if (current_predecessor_acceptable) {
    return UpdateAction::kOld;
  }

  const bool source_predecessor_acceptable =
      (g_scores_source_predecessor - g_scores_current_predecessor <
       d_source_predecessor_corner * config_.refinement_epsilon +
           kAbsolutePrecision)
          .all();
  if (source_predecessor_acceptable) {
    return UpdateAction::kSourcePredecessor;
  }

  // Otherwise, refine
  return UpdateAction::kRefine;
}
}  // namespace wavemap
