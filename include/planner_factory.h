#ifndef PLANNER_FACTORY_H_
#define PLANNER_FACTORY_H_

#include "planner_base.h"

namespace wavemap {
class PlannerFactory {
 public:
  static PlannerBase::Ptr create(
      const param::Value& params, ClassifiedMap::ConstPtr classified_map,
      std::optional<PlannerType> default_planner_type = std::nullopt);

  static PlannerBase::Ptr create(PlannerType planner_type,
                                 const param::Value& params,
                                 ClassifiedMap::ConstPtr classified_map);
};
}  // namespace wavemap

#endif  // PLANNER_FACTORY_H_
