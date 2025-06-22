#ifndef UTILS_CONTAINERS_H_
#define UTILS_CONTAINERS_H_

#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include <wavemap/core/common.h>
#include <wavemap/core/data_structure/dense_block_hash.h>

namespace wavemap {
struct GScoreAndPredecessor {
  FloatingPoint g_score;
  Index3D predecessor;
  bool closed;

  friend bool operator==(const GScoreAndPredecessor& lhs,
                         const GScoreAndPredecessor& rhs) {
    return lhs.predecessor == rhs.predecessor && lhs.g_score == rhs.g_score;
  }
  friend bool operator!=(const GScoreAndPredecessor& lhs,
                         const GScoreAndPredecessor& rhs) {
    return !(lhs == rhs);
  }
};

using CostField = DenseBlockHash<GScoreAndPredecessor, 3, 16>;

struct FScoreAndIndex {
  FloatingPoint f_score;
  Index3D index;

  friend bool operator>(const FScoreAndIndex& lhs, const FScoreAndIndex& rhs) {
    return lhs.f_score > rhs.f_score;
  }
};

struct GScoreAndMultiResPredecessor {
  // G_score at the node's center point
  FloatingPoint predecessor_g_score = std::numeric_limits<FloatingPoint>::max();
  OctreeIndex predecessor_index{
      0, Index3D::Constant(std::numeric_limits<IndexElement>::max())};
  bool closed = false;

  friend bool operator==(const GScoreAndMultiResPredecessor& lhs,
                         const GScoreAndMultiResPredecessor& rhs) {
    return lhs.predecessor_g_score == rhs.predecessor_g_score &&
           lhs.predecessor_index == rhs.predecessor_index &&
           lhs.closed == rhs.closed;
  }
  friend bool operator!=(const GScoreAndMultiResPredecessor& lhs,
                         const GScoreAndMultiResPredecessor& rhs) {
    return !(lhs == rhs);
  }
};
using MultiResCostField = OctreeBlockHash<GScoreAndMultiResPredecessor>;

struct FScoreAndMultiResIndex {
  FloatingPoint f_score = std::numeric_limits<FloatingPoint>::max();
  OctreeIndex node_index{
      0, Index3D::Constant(std::numeric_limits<IndexElement>::max())};
  MultiResCostField::Node* node = nullptr;

  friend bool operator>(const FScoreAndMultiResIndex& lhs,
                        const FScoreAndMultiResIndex& rhs) {
    return lhs.f_score > rhs.f_score;
  }
};

template <typename T>
using min_priority_queue =
    std::priority_queue<T, std::vector<T>, std::greater<T>>;
}  // namespace wavemap

#endif  // UTILS_CONTAINERS_H_
