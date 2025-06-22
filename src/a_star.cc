#include "a_star.h"

#include <utility>
#include <vector>

#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(AStarConfig,
                      (heuristic_type));

bool AStarConfig::isValid(bool verbose) const {
  return IS_PARAM_EQ(heuristic_type.isValid(), true, verbose);
}

void AStar::clear() {
  ProfilerZoneScoped;
  start_index_.setZero();
  goal_index_.setZero();
  cost_field_.clear();
  open_queue_ = PriorityQueue{};
}

std::vector<Point3D> AStar::searchPath(const Point3D& start,
                                       const Point3D& goal) {
  ProfilerZoneScoped;
  if (solve(start, goal)) {
    return getPath();
  }
  return {};
}

bool AStar::solve(const Point3D& start, const Point3D& goal) {
  ProfilerZoneScoped;
  // Make sure the previous solution has been cleared
  clear();

  // Check that the start and end positions are collision free
  start_index_ = convert::pointToNearestIndex(start, min_cell_width_inv_);
  goal_index_ = convert::pointToNearestIndex(goal, min_cell_width_inv_);
  for (const auto& [name, index] :
       {std::tuple{"start", start_index_}, {"goal", goal_index_}}) {
    if (!classified_map_->isFully(index, Occupancy::kFree)) {
      LOG(WARNING) << "Collision free path can not be found: " << name
                   << " is not fully free";
      return false;
    }
  }

  // Initialize the cost field and queue
  cost_field_.getOrAllocateValue(start_index_) = {0.f, start_index_, false};
  open_queue_.emplace(FScoreAndIndex{0.f, start_index_});

  while (!open_queue_.empty()) {
    ProfilerPlot("QueueLength", static_cast<int64_t>(open_queue_.size()));
    const Index3D cell_index = open_queue_.top().index;
    open_queue_.pop();

    auto& cell = cost_field_.getOrAllocateValue(cell_index);
    if (cell.closed) {
      continue;
    } else {
      cell.closed = true;
    }

    if (cell_index == goal_index_) {
      return true;
    }

    for (size_t neighbor_idx = 0; neighbor_idx < kNeighborIndexOffsets.size();
         ++neighbor_idx) {
      const Index3D neighbor_index =
          cell_index + kNeighborIndexOffsets[neighbor_idx];

      if (!classified_map_->isFully(neighbor_index, Occupancy::kFree)) {
        continue;
      }

      auto& neighbor = cost_field_.getOrAllocateValue(neighbor_index);
      if (neighbor.closed) {
        continue;
      }

      const FloatingPoint neighbor_candidate_g_score =
          cell.g_score + neighbor_distance_offsets_[neighbor_idx];
      if (neighbor_candidate_g_score < neighbor.g_score) {
        neighbor.g_score = neighbor_candidate_g_score;
        neighbor.predecessor = cell_index;

        const Point3D neighbor_center =
            convert::indexToCenterPoint(neighbor_index, min_cell_width_);
        const FloatingPoint neighbor_f_score = computeFScore(
            config_.heuristic_type, neighbor_center, neighbor.g_score, goal);
        open_queue_.emplace(FScoreAndIndex{neighbor_f_score, neighbor_index});
      }
    }
  }

  return false;
}

std::vector<Point3D> AStar::getPath() const {
  ProfilerZoneScoped;
  std::vector<Point3D> path;
  Index3D index = goal_index_;
  while (true) {
    if (index == cost_field_.getDefaultValue().predecessor) {
      return {};
    }
    Point3D center = convert::indexToCenterPoint(index, min_cell_width_);
    path.emplace_back(std::move(center));
    if (index == start_index_) {
      std::reverse(path.begin(), path.end());
      return path;
    }
    index = cost_field_.getValueOrDefault(index).predecessor;
  }
}
}  // namespace wavemap
