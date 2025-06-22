#ifndef UTILS_GEOMETRY_H_
#define UTILS_GEOMETRY_H_

#include <utility>

#include <wavemap/core/common.h>
#include <wavemap/core/indexing/ndtree_index.h>

namespace wavemap {
inline std::pair<OctreeIndex, OctreeIndex> minMaxNeighborsAtHeight(
    const OctreeIndex& index, IndexElement start_height) {
  const OctreeIndex min_neighbor{index.height, index.position.array() - 1};
  const OctreeIndex max_neighbor{index.height, index.position.array() + 1};
  return {min_neighbor.computeParentIndex(start_height),
          max_neighbor.computeParentIndex(start_height)};
}

inline FloatingPoint distanceBetweenCenters(const OctreeIndex& node_a,
                                            const OctreeIndex& node_b,
                                            FloatingPoint min_cell_width) {
  return (convert::nodeIndexToCenterPoint(node_b, min_cell_width) -
          convert::nodeIndexToCenterPoint(node_a, min_cell_width))
      .norm();
}
}  // namespace wavemap

#endif  // UTILS_GEOMETRY_H_
