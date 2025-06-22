#ifndef OMPL_PLANNER_H_
#define OMPL_PLANNER_H_

#include <memory>
#include <utility>
#include <vector>

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "planner_base.h"

namespace wavemap {
namespace convert {
Point3D omplToEigen(const ompl::base::State* state);
ompl::base::ScopedState<> eigenToOmpl(const ompl::base::StateSpacePtr& space,
                                      const Point3D& point);
}  // namespace convert

/**
 * Config struct for OMPL-based planners.
 */
struct OmplPlannerConfig : ConfigBase<OmplPlannerConfig, 1> {
  Seconds<FloatingPoint> max_solve_time = 10.f;

  static MemberMap memberMap;

  // Constructors
  OmplPlannerConfig() = default;
  explicit OmplPlannerConfig(FloatingPoint max_solve_time)
      : max_solve_time(max_solve_time) {}

  bool isValid(bool verbose) const override;
};

class OmplPlanner : public PlannerBase {
 public:
  OmplPlanner(const OmplPlannerConfig& config,
              ClassifiedMap::ConstPtr classified_map, PlannerType planner_type);

  void clear() override;

  std::vector<Point3D> searchPath(const Point3D& start,
                                  const Point3D& goal) override;

  bool solve(const Point3D& start, const Point3D& goal);

  std::vector<Point3D> getPath() const;

 private:
  const OmplPlannerConfig config_;
  const PlannerType planner_type_;

  Index3D start_index_ = Index3D::Zero();
  Index3D goal_index_ = Index3D::Zero();

  const FloatingPoint min_cell_width_ = classified_map_->getMinCellWidth();

  std::shared_ptr<ompl::base::RealVectorStateSpace> space_ =
      std::make_shared<ompl::base::RealVectorStateSpace>(3);
  std::shared_ptr<ompl::base::SpaceInformation> space_info_ =
      std::make_shared<ompl::base::SpaceInformation>(space_);
  std::shared_ptr<ompl::base::ProblemDefinition> problem_ =
      std::make_shared<ompl::base::ProblemDefinition>(space_info_);
};

class MotionValidator : public ompl::base::MotionValidator {
 public:
  MotionValidator(const ompl::base::SpaceInformationPtr& space_info,
                  const ClassifiedMap& classified_map, IndexElement min_height)
      : ompl::base::MotionValidator(space_info),
        classified_map_(classified_map),
        min_height_(min_height) {}

  bool checkMotion(const ompl::base::State* s1,
                   const ompl::base::State* s2) const override {
    std::pair<ompl::base::State*, double> unused;
    return checkMotion(s1, s2, unused);
  }

  bool checkMotion(
      const ompl::base::State* s1, const ompl::base::State* s2,
      std::pair<ompl::base::State*, double>& last_valid) const override;

 protected:
  const ClassifiedMap& classified_map_;
  const IndexElement min_height_;
  const FloatingPoint min_cell_width_ = classified_map_.getMinCellWidth();
  const FloatingPoint cell_width_at_min_height_ =
      convert::heightToCellWidth(min_cell_width_, min_height_);
};
}  // namespace wavemap

#endif  // OMPL_PLANNER_H_
