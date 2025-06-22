#include "ompl_planner.h"

#include <memory>
#include <utility>
#include <vector>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <wavemap/core/utils/iterate/ray_iterator.h>
#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(OmplPlannerConfig,
                      (max_solve_time));

bool OmplPlannerConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(max_solve_time, 0.f, verbose);

  return is_valid;
}

OmplPlanner::OmplPlanner(const OmplPlannerConfig& config,
                         ClassifiedMap::ConstPtr classified_map,
                         PlannerType planner_type)
    : PlannerBase(std::move(classified_map)),
      config_(config.checkValid()),
      planner_type_(std::move(planner_type)) {
  // Set OMPL logging level
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

  // Set bounds
  const Point3D min_corner = convert::indexToMinCorner(
      classified_map_->getMinIndex(), classified_map_->getMinCellWidth());
  const Point3D max_corner = convert::indexToMaxCorner(
      classified_map_->getMaxIndex(), classified_map_->getMinCellWidth());
  ompl::base::RealVectorBounds bounds(3);
  for (int dim_idx = 0; dim_idx < 3; ++dim_idx) {
    bounds.low[dim_idx] = min_corner[dim_idx];
    bounds.high[dim_idx] = max_corner[dim_idx];
  }
  space_->setBounds(bounds);

  // Set the state validity checker
  space_info_->setStateValidityChecker([this](const ompl::base::State* state) {
    const Point3D point = convert::omplToEigen(state);
    const OctreeIndex index =
        convert::pointToNodeIndex(point, min_cell_width_, 0);
    return classified_map_->isFully(index, Occupancy::kFree);
  });

  // Set the motion validity checker and grid resolution
  // NOTE: Since we provide our own MotionValidator, we do not need to set
  //       space_info_->setStateValidityCheckingResolution()
  space_info_->setMotionValidator(ompl::base::MotionValidatorPtr(
      new MotionValidator(space_info_, *classified_map_, 0)));
}

void OmplPlanner::clear() {
  ProfilerZoneScoped;
  start_index_.setZero();
  goal_index_.setZero();
  problem_ = std::make_shared<ompl::base::ProblemDefinition>(space_info_);
}

std::vector<Point3D> OmplPlanner::searchPath(const Point3D& start,
                                             const Point3D& goal) {
  ProfilerZoneScoped;
  if (solve(start, goal)) {
    return getPath();
  }
  return {};
}

bool OmplPlanner::solve(const Point3D& start, const Point3D& goal) {
  ProfilerZoneScoped;
  // Make sure the previous solution has been cleared
  clear();

  // Set the start and goal states
  auto ompl_start = convert::eigenToOmpl(space_, start);
  auto ompl_goal = convert::eigenToOmpl(space_, goal);
  problem_->setStartAndGoalStates(ompl_start, ompl_goal);

  // Set the optimization objective
  problem_->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(
      new ompl::base::PathLengthOptimizationObjective(space_info_)));

  // Create the planner
  std::shared_ptr<ompl::base::Planner> planner;
  switch (planner_type_) {
    case PlannerType::kRRTConnect:
      planner = std::make_shared<ompl::geometric::RRTConnect>(space_info_);
      break;
    case PlannerType::kRRTStar:
      planner = std::make_shared<ompl::geometric::RRTstar>(space_info_);
      break;
    default:
      return false;
  }

  // Configure the planner
  planner->setProblemDefinition(problem_);
  planner->setup();

  // Run the solver
  planner->ompl::base::Planner::solve(config_.max_solve_time);

  // Indicate whether a solution was found
  return problem_->hasExactSolution();
}

std::vector<Point3D> OmplPlanner::getPath() const {
  ProfilerZoneScoped;
  if (problem_->hasExactSolution()) {
    const auto ompl_path =
        problem_->getSolutionPath()->as<ompl::geometric::PathGeometric>();

    std::vector<Point3D> path;
    path.reserve(ompl_path->getStateCount());
    for (const auto& ompl_state : ompl_path->getStates()) {
      const auto waypoint = convert::omplToEigen(ompl_state);
      path.emplace_back(waypoint);
    }

    return path;
  }

  return {};
}

Point3D convert::omplToEigen(const ompl::base::State* state) {
  const auto* derived =
      state->as<ompl::base::RealVectorStateSpace::StateType>();
  const Eigen::Vector3d eigen_state(derived->values[0], derived->values[1],
                                    derived->values[2]);
  return eigen_state.cast<FloatingPoint>();
}

ompl::base::ScopedState<> convert::eigenToOmpl(
    const ompl::base::StateSpacePtr& space, const Point3D& point) {
  ompl::base::ScopedState<> state(space);
  state[0] = point[0];
  state[1] = point[1];
  state[2] = point[2];
  return state;
}

bool MotionValidator::checkMotion(
    const ompl::base::State* s1, const ompl::base::State* s2,
    std::pair<ompl::base::State*, double>& last_valid) const {
  const Point3D start = convert::omplToEigen(s1);
  const Point3D goal = convert::omplToEigen(s2);

  const Ray ray(start, goal, cell_width_at_min_height_);
  int step_count = 0;
  constexpr int kMaxNumSteps = 1000;
  Index3D last_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  for (const Index3D& ray_voxel_index : ray) {
    const OctreeIndex ray_node_index{min_height_, ray_voxel_index};
    if (!classified_map_.isFully(ray_node_index, Occupancy::kFree) ||
        kMaxNumSteps < step_count) {
      if (last_valid.first) {
        const Point3D last_position =
            convert::indexToCenterPoint(last_index, cell_width_at_min_height_);
        const auto last_valid_state =
            convert::eigenToOmpl(si_->getStateSpace(), last_position);
        si_->copyState(last_valid.first, last_valid_state.get());
      }
      last_valid.second =
          static_cast<double>(step_count) / static_cast<double>(ray.size());
      return false;
    }
    last_index = ray_voxel_index;
    ++step_count;
  }
  return true;
}
}  // namespace wavemap
