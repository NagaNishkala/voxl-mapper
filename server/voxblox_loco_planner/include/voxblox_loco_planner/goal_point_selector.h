#ifndef VOXBLOX_LOCO_PLANNER_GOAL_POINT_SELECTOR_H_
#define VOXBLOX_LOCO_PLANNER_GOAL_POINT_SELECTOR_H_

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_common/utils.h>
#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>

namespace mav_planning {

struct GoalPointSelectorParameters {
  enum Strategy {
      kNoIntermediateGoal = 0,
      kRandom,
      kLocalExploration
      };

  Strategy strategy = kNoIntermediateGoal;

  // For all random-based selectors.
  double random_sample_range = 5.0;

  // For random sampling in general.
  int max_random_tries = 100;

  // For the exploration planner.
  int num_exploration_samples = 15;
  int exp_modulus = 20;
  double w_exploration = 1.0;
  double w_goal = 0.5;
};

class GoalPointSelector {
 public:
  GoalPointSelector();

  // Setup
  void setParameters(const GoalPointSelectorParameters& params);
  bool setTsdfMap(std::shared_ptr<voxblox::TsdfMap> tsdf_map);
  GoalPointSelectorParameters getParameters() const;

  bool selectNextGoal(const mav_msgs::EigenTrajectoryPoint& global_goal,
                      const mav_msgs::EigenTrajectoryPoint& current_goal,
                      const mav_msgs::EigenTrajectoryPoint& current_pose,
                      mav_msgs::EigenTrajectoryPoint* next_goal);

 private:
  void selectRandomPose(const mav_msgs::EigenTrajectoryPoint& input_pose,
                        double range_meters,
                        mav_msgs::EigenTrajectoryPoint* sampled_pose) const;

  bool selectRandomFreePose(const mav_msgs::EigenTrajectoryPoint& input_pose,
                            double range_meters,
                            mav_msgs::EigenTrajectoryPoint* sampled_pose) const;

  bool selectLocalExplorationGoal(
      const mav_msgs::EigenTrajectoryPoint& global_goal,
      const mav_msgs::EigenTrajectoryPoint& current_goal,
      const mav_msgs::EigenTrajectoryPoint& current_pose,
      mav_msgs::EigenTrajectoryPoint* next_goal);

  // Settings.
  GoalPointSelectorParameters params_;


  // Map.
  std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_LOCO_PLANNER_GOAL_POINT_SELECTOR_H_
