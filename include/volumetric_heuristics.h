#ifndef VOLUMETRIC_HEURISTICS_H_
#define VOLUMETRIC_HEURISTICS_H_

#include <wavemap/core/common.h>
#include <wavemap/core/indexing/ndtree_index.h>

namespace wavemap {
FloatingPoint computeFScore(const OctreeIndex& predecessor_index,
                            FloatingPoint predecessor_g_score,
                            const OctreeIndex& node_index, const Point3D& goal,
                            FloatingPoint min_cell_width,
                            FloatingPoint node_negative_padding);
}  // namespace wavemap

#endif  // VOLUMETRIC_HEURISTICS_H_
