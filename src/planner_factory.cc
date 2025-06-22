#include "planner_factory.h"

#include <memory>
#include <utility>

#include "a_star.h"
#include "lazy_theta_star.h"
#include "multi_res_lazy_theta_star.h"
#include "multi_res_theta_star.h"
#include "ompl_planner.h"
#include "theta_star.h"

namespace wavemap {
PlannerBase::Ptr PlannerFactory::create(
    const param::Value& params, ClassifiedMap::ConstPtr classified_map,
    std::optional<PlannerType> default_planner_type) {
  if (const auto type = PlannerType::from(params); type) {
    return create(type.value(), params, std::move(classified_map));
  }

  if (default_planner_type.has_value()) {
    LOG(WARNING) << "Default type \"" << default_planner_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_planner_type.value(), params,
                  std::move(classified_map));
  }

  LOG(ERROR) << "No default was set. Returning nullptr.";
  return nullptr;
}

PlannerBase::Ptr PlannerFactory::create(
    PlannerType planner_type, const param::Value& params,
    ClassifiedMap::ConstPtr classified_map) {
  // Assemble the planner
  switch (planner_type) {
    case PlannerType::kAStar: {
      const auto planner_config = AStarConfig::from(params);
      if (planner_config) {
        LOG(INFO) << "Creating A Star planner";
        return std::make_shared<AStar>(planner_config.value(), classified_map);
      } else {
        LOG(ERROR) << "A Star planner config could not be loaded.";
        return nullptr;
      }
    }
    case PlannerType::kThetaStar: {
      LOG(INFO) << "Creating Theta Star planner";
      return std::make_shared<ThetaStar>(classified_map);
    }
    case PlannerType::kLazyThetaStar: {
      LOG(INFO) << "Creating Lazy Theta Star planner";
      return std::make_shared<LazyThetaStar>(classified_map);
    }
    case PlannerType::kRRTConnect: {
      const auto planner_config = OmplPlannerConfig::from(params);
      if (planner_config) {
        LOG(INFO) << "Creating RRT Connect planner";
        return std::make_shared<OmplPlanner>(planner_config.value(),
                                             classified_map, planner_type);
      } else {
        LOG(ERROR) << "RRT Connect planner config could not be loaded.";
        return nullptr;
      }
    }
    case PlannerType::kRRTStar: {
      const auto planner_config = OmplPlannerConfig::from(params);
      if (planner_config) {
        LOG(INFO) << "Creating RRT Star planner";
        return std::make_shared<OmplPlanner>(planner_config.value(),
                                             classified_map, planner_type);
      } else {
        LOG(ERROR) << "RRT Star planner config could not be loaded.";
        return nullptr;
      }
    }
    case PlannerType::kMultiResThetaStar: {
      const auto planner_config = MultiResThetaStarConfig::from(params);
      if (planner_config) {
        LOG(INFO) << "Creating Multi-Resolution Theta Star planner";
        return std::make_shared<MultiResThetaStar>(planner_config.value(),
                                                   classified_map);
      } else {
        LOG(ERROR) << "Multi-Resolution Theta Star planner config could not be "
                      "loaded.";
        return nullptr;
      }
    }
    case PlannerType::kMultiResLazyThetaStar: {
      const auto planner_config = MultiResLazyThetaStarConfig::from(params);
      if (planner_config) {
        LOG(INFO) << "Creating Multi-Resolution Lazy Theta Star planner";
        return std::make_shared<MultiResLazyThetaStar>(planner_config.value(),
                                                       classified_map);
      } else {
        LOG(ERROR) << "Multi-Resolution Lazy Theta Star planner config could "
                      "not be loaded.";
        return nullptr;
      }
    }
    default:
      LOG(ERROR) << "Attempted to create planner with unknown type ID: "
                 << planner_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
