#ifndef PLANNER_BASE_H_
#define PLANNER_BASE_H_

#include <memory>
#include <utility>
#include <vector>

#include <wavemap/core/common.h>
#include <wavemap/core/config/type_selector.h>
#include <wavemap/core/utils/query/classified_map.h>

namespace wavemap {
struct PlannerType : public TypeSelector<PlannerType> {
  using TypeSelector<PlannerType>::TypeSelector;

  enum Id : TypeId {
    kAStar,
    kThetaStar,
    kLazyThetaStar,
    kRRTConnect,
    kRRTStar,
    kMultiResThetaStar,
    kMultiResLazyThetaStar
  };

  static constexpr std::array names = {"a_star",
                                       "theta_star",
                                       "lazy_theta_star",
                                       "rrt_connect",
                                       "rrt_star",
                                       "multi_res_theta_star",
                                       "multi_res_lazy_theta_star"};
};

class PlannerBase {
 public:
  using Ptr = std::shared_ptr<PlannerBase>;

  explicit PlannerBase(ClassifiedMap::ConstPtr classified_map)
      : classified_map_(std::move(CHECK_NOTNULL(classified_map))) {}
  virtual ~PlannerBase() = default;

  virtual void clear() = 0;

  virtual std::vector<Point3D> searchPath(const Point3D& start,
                                          const Point3D& goal) = 0;

 protected:
  const ClassifiedMap::ConstPtr classified_map_;
};
}  // namespace wavemap

#endif  // PLANNER_BASE_H_
