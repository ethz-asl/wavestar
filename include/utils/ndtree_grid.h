#ifndef UTILS_NDTREE_GRID_H_
#define UTILS_NDTREE_GRID_H_

#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/ndtree_index.h"

namespace wavemap {
template <int dim>
class NdtreeGrid {
 public:
  NdtreeGrid(NdtreeIndex<dim> min_index, NdtreeIndex<dim> max_index)
      : min_index_(min_index), max_index_(max_index) {}

  class Iterator {
   public:
    using difference_type = std::ptrdiff_t;
    using value_type = NdtreeIndex<dim>;
    using pointer = NdtreeIndex<dim>*;
    using reference = NdtreeIndex<dim>&;
    using iterator_category = std::forward_iterator_tag;

    explicit Iterator(const NdtreeGrid& grid)
        : grid_(grid), current_index_(grid_.min_index_) {
      CHECK_EQ(grid_.min_index_.height, grid_.max_index_.height);
    }
    Iterator(const NdtreeGrid& grid, bool end) : Iterator(grid) {
      if (end) {
        current_index_.position[dim - 1] =
            grid_.max_index_.position[dim - 1] + 1;
      }
    }

    const NdtreeIndex<dim>& operator*() const { return current_index_; }
    Iterator& operator++() {  // prefix ++
      ++current_index_.position[0];
      for (int dim_idx = 0; dim_idx < dim - 1; ++dim_idx) {
        if (grid_.max_index_.position[dim_idx] <
            current_index_.position[dim_idx]) {
          current_index_.position[dim_idx] = grid_.min_index_.position[dim_idx];
          ++current_index_.position[dim_idx + 1];
        }
      }
      return *this;
    }
    Iterator operator++(int) {  // postfix ++
      Iterator retval = *this;
      ++(*this);  // call the above prefix incrementer
      return retval;
    }
    friend bool operator==(const Iterator& lhs, const Iterator& rhs) {
      return lhs.current_index_ == rhs.current_index_;
    }
    friend bool operator!=(const Iterator& lhs, const Iterator& rhs) {
      return !(lhs == rhs);  // NOLINT
    }

   private:
    const NdtreeGrid& grid_;
    NdtreeIndex<dim> current_index_;
  };

  Iterator begin() const { return Iterator{*this}; }
  Iterator end() const { return Iterator{*this, /*end*/ true}; }

 private:
  const NdtreeIndex<dim> min_index_;
  const NdtreeIndex<dim> max_index_;
};
}  // namespace wavemap

#endif  // UTILS_NDTREE_GRID_H_
