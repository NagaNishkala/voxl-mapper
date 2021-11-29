#ifndef MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_
#define MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_

#include <string>
#include <thread>
#include <vector>


#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <mav_path_smoothing/polynomial_smoother.h>
#include <mav_path_smoothing/velocity_ramp_smoother.h>
#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_utils.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_common/semaphore.h>
#include <mav_planning_common/yaw_policy.h>
#include <mav_planning_msgs/mav_planning_msgs.h>

#include <voxblox_loco_planner/goal_point_selector.h>
#include <voxblox_loco_planner/voxblox_loco_planner.h>

// just pass in our tsdf and esdf map (shared_ptrs) instead
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>


namespace mav_planning {

typedef struct MavLocalPlannerParams {
    float tsdf_voxel_size = 0.25;
    int tsdf_voxels_per_side = 16;
    float esdf_max_distance_m = 2.0;
    float traversability_radius = 1.0;
    float slice_level = 1.0;
    float replan_dt = 1.0;
    float command_publishing_dt = 0.25;
    float replan_lookahead_sec = 1.0;
    int mpc_prediction_horizon = 300;
    float planning_horizon_m = 20.0;
    bool verbose = false;
    float v_max = 1.0;
    float a_max = 2.0;
    float yaw_rate_max = M_PI / 4.0;
    float sampling_dt = 0.01;
    bool avoid_collisions = true;
    std::string smoother_name = "loco";
    int max_failures = 5;
    bool plan_to_start_ = true;
} MavLocalPlannerParams;


class MavLocalPlanner {
 public:
  MavLocalPlanner();
  MavLocalPlanner(MavLocalPlannerParams _mav_params, LocoPlannerParams _loco_params, std::shared_ptr<voxblox::TsdfMap> _tsdf_map, std::shared_ptr<voxblox::EsdfMap> _esdf_map);

  // Input data.

  void setRobotPose(const mav_msgs::EigenTrajectoryPoint& curr_pose);
  bool waypointCallback(const mav_msgs::EigenTrajectoryPoint& goal_pose, mav_planning_msgs::PolynomialTrajectory4D &msg);

  void waypointListCallback(const mav_msgs::EigenTrajectoryPointVector& goal_pose_vector);

  // Stops path publishing and clears all recent trajectories.
  void clearTrajectory();
  // Same as above, but also sends the current pose of the helicopter to the
  // controller to clear the queue.
  void abort();

 private:

  void planningStep();

  // Returns if the next waypoint is a valid waypoint.
  bool nextWaypoint();
  void finishWaypoints();

  void replacePath(const mav_msgs::EigenTrajectoryPointVector& path);

  // What to do if we fail to find a suitable path, depending on the
  // intermediate goal selection settings.
  bool dealWithFailure();

  // Functions to help out replanning.
  // Track a single waypoint, planning only in a short known horizon.
  void avoidCollisionsTowardWaypoint();

  // bool avoidCollisionsTowardWaypoint(mav_planning_msgs::PolynomialTrajectory4D &msg);
  // Get a path through a bunch of waypoints.
  bool planPathThroughWaypoints(
      const mav_msgs::EigenTrajectoryPointVector& waypoints,
      mav_msgs::EigenTrajectoryPointVector* path);

  // Map access.
  double getMapDistance(const Eigen::Vector3d& position) const;
  double getMapDistanceAndGradient(const Eigen::Vector3d& position,
                                   Eigen::Vector3d* gradient) const;

  // Double-check that everything is safe w.r.t. current map.
  bool isPathCollisionFree(
      const mav_msgs::EigenTrajectoryPointVector& path) const;
  bool isPathFeasible(const mav_msgs::EigenTrajectoryPointVector& path) const;

  // Other internal stuff.
  void sendCurrentPose();

  // Settings -- general
  bool verbose_;
  bool plan_to_start_;


  // Settings -- constraints.
  PhysicalConstraints constraints_;
  ShotgunParameters _shotgun_params;


  // Settings -- controller interface.
  int mpc_prediction_horizon_;  // Units: timesteps.
  // TODO(helenol): do I need these two to be separate? I guess so...
  double command_publishing_dt_;
  double replan_dt_;
  double replan_lookahead_sec_;

  // Settings -- general planning.
  bool avoid_collisions_;
  // for start service call.
  std::string smoother_name_;

  // State -- robot state.
  mav_msgs::EigenTrajectoryPoint odometry_;

  // State -- waypoints.
  mav_msgs::EigenTrajectoryPointVector waypoints_;
  int64_t current_waypoint_;

  // State -- current tracked path.
  mav_msgs::EigenTrajectoryPointVector path_queue_;
  size_t path_index_;
  // Super important: mutex for locking the path queues.
  std::recursive_mutex path_mutex_;
  std::recursive_mutex map_mutex_;
  RosSemaphore should_replan_;

  // State -- planning.
  int max_failures_;
  int num_failures_;

  // Map!
  std::shared_ptr<voxblox::TsdfMap> tsdf_map;
  std::shared_ptr<voxblox::EsdfMap> esdf_map;

  // Planners -- yaw policy
  YawPolicy yaw_policy_;

  // Planners -- local planners.
  VoxbloxLocoPlanner loco_planner_;

  // Planners -- path smoothers.
  VelocityRampSmoother ramp_smoother_;
  PolynomialSmoother poly_smoother_;
  LocoSmoother loco_smoother_;

  // Intermediate goal selection, optionally in case of path-planning failures:
  GoalPointSelector goal_selector_;
  bool temporary_goal_;
};

}  // namespace mav_planning

#endif  // MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_
