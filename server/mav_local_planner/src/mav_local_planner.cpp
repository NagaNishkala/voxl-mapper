#include "mav_local_planner/mav_local_planner.h"
#include "mav_local_planner/conversions.h"

#include <mav_trajectory_generation/trajectory_sampling.h>


namespace mav_planning {

MavLocalPlanner::MavLocalPlanner(MavLocalPlannerParams _mav_params, LocoPlannerParams _loco_params, std::shared_ptr<voxblox::TsdfMap> _tsdf_map, std::shared_ptr<voxblox::EsdfMap> _esdf_map)
    : loco_planner_(_loco_params, _shotgun_params, constraints_) {
    verbose_ = _mav_params.verbose;
    mpc_prediction_horizon_ = _mav_params.mpc_prediction_horizon;
    command_publishing_dt_ = _mav_params.command_publishing_dt;
    replan_dt_ = _mav_params.replan_dt;
    replan_lookahead_sec_ = _mav_params.replan_lookahead_sec;
    avoid_collisions_ = _mav_params.avoid_collisions;
    smoother_name_ = _mav_params.smoother_name;
    current_waypoint_ = -1;
    path_index_ = 0;
    max_failures_ = _mav_params.max_failures;
    num_failures_ = 0;
    tsdf_map = _tsdf_map;
    esdf_map = _esdf_map;
    plan_to_start_ = _mav_params.plan_to_start_;

    // Set up some settings.
    // leave phys constraints default for now
    loco_planner_.setEsdfMap(_esdf_map);
    GoalPointSelectorParameters _gp_params_;
    goal_selector_.setParameters(_gp_params_);  // leave default for now
    goal_selector_.setTsdfMap(_tsdf_map);

    // Set up yaw policy.
    yaw_policy_.setPhysicalConstraints(constraints_);
    yaw_policy_.setYawPolicy(YawPolicy::PolicyType::kVelocityVector);

    // Set up smoothers.
    const double voxel_size = esdf_map->getEsdfLayerPtr()->voxel_size();

    // Straight-line smoother.
    bool verbose = false;
    ramp_smoother_.setParameters(constraints_, verbose);

    // Poly smoother.
    poly_params pp;  // leave default for now
    poly_smoother_.setParameters(pp, constraints_, verbose);
    poly_smoother_.setMinCollisionCheckResolution(voxel_size);
    poly_smoother_.setMapDistanceCallback(
        std::bind(&MavLocalPlanner::getMapDistance, this, std::placeholders::_1));
    poly_smoother_.setOptimizeTime(true);
    poly_smoother_.setSplitAtCollisions(avoid_collisions_);

    // Loco smoother!
    locoParams loco_p;
    loco_smoother_.setParameters(loco_p);
    loco_smoother_.setMinCollisionCheckResolution(voxel_size);
    loco_smoother_.setDistanceAndGradientFunction(
        std::bind(&MavLocalPlanner::getMapDistanceAndGradient, this,
                  std::placeholders::_1, std::placeholders::_2));
    loco_smoother_.setOptimizeTime(true);
    loco_smoother_.setResampleTrajectory(true);
    loco_smoother_.setResampleVisibility(true);
    loco_smoother_.setNumSegments(5);
}

// manually update odometry data with our last qvio callback data, or we can sub seperately
void MavLocalPlanner::setRobotPose(const mav_msgs::EigenTrajectoryPoint& curr_pose) {
    odometry_.position_W = curr_pose.position_W;
    odometry_.orientation_W_B = curr_pose.orientation_W_B;
}

// need a "point" subscriber, so we can plan where to go
bool MavLocalPlanner::waypointCallback(const mav_msgs::EigenTrajectoryPoint& goal_pose, __attribute__((unused)) mav_planning_msgs::PolynomialTrajectory4D &msg) {
    // Plan a path from the current position to the target pose stamped.
    printf("[Mav Local Planner] Got a waypoint!\n");
    // Cancel any previous trajectory on getting a new one.
    clearTrajectory();

    waypoints_.clear();
    waypoints_.push_back(goal_pose);
    current_waypoint_ = 0;

    // Execute one planning step on main thread.
    // bool success = avoidCollisionsTowardWaypoint(msg);
    planningStep();
    // if (success){
    //     printf("planning successful\n");
    //     return true;
    // }
    // else printf("planning failed\n");
    return false;
    // startPublishingCommands();
}

// need a "point array" subscriber, " "
void MavLocalPlanner::waypointListCallback(const mav_msgs::EigenTrajectoryPointVector& goal_pose_vector) {
    // Plan a path from the current position to the target pose stamped.
    // TODO:  not sure what the traj point vector members are
    printf("[Mav Local Planner] Got a list of waypoints, %zu long!\n", goal_pose_vector.size());
    // Cancel any previous trajectory on getting a new one.
    clearTrajectory();

    waypoints_.clear();

    for (const auto& trajectory_point : goal_pose_vector) {
        waypoints_.push_back(trajectory_point);
    }
    current_waypoint_ = 0;

    // Execute one planning step on main thread.
    planningStep();
    // startPublishingCommands();
}

void MavLocalPlanner::planningStep() {
    printf("[Mav Local Planner][Plan Step] Waypoint index: %zd Total waypoints: %zu\n", current_waypoint_, waypoints_.size());
    if (current_waypoint_ < 0 || static_cast<int>(waypoints_.size()) <= current_waypoint_) {
        // This means that we probably planned to the end of the waypoints!
        // If we're done with sending waypoints, alllll good. Just quit.
        if (path_index_ >= path_queue_.size() || path_queue_.empty()) {
            return;
        }
        // If we're not though, we should probably double check the trajectory!
    }

    mav_trajectory_generation::timing::MiniTimer timer;
    constexpr double kCloseToOdometry = 0.1;

    // First, easiest case: if we're not avoiding collisions, just use the
    // favorite path smoother. We only do this on the first planning call then
    // ignore all the rest.
    if (!avoid_collisions_) {
        mav_msgs::EigenTrajectoryPointVector waypoints;
        mav_msgs::EigenTrajectoryPoint current_point;
        current_point.position_W = odometry_.position_W;
        current_point.orientation_W_B = odometry_.orientation_W_B;

        if (plan_to_start_) {
            waypoints.push_back(current_point);
        }
        waypoints.insert(waypoints.end(), waypoints_.begin(), waypoints_.end());

        mav_msgs::EigenTrajectoryPointVector path;

        if (planPathThroughWaypoints(waypoints, &path)) {
            replacePath(path);
            current_waypoint_ = waypoints_.size();
        } else {
            fprintf(stderr, "[Mav Local Planner] Waypoint planning failed!\n");
        }
    } else if (path_queue_.empty()) {
        // First check how many waypoints we haven't covered yet are in free space.
        mav_msgs::EigenTrajectoryPointVector free_waypoints;
        // Do we need the odometry in here? Let's see.
        mav_msgs::EigenTrajectoryPoint current_point;
        current_point.position_W = odometry_.position_W;
        current_point.orientation_W_B = odometry_.orientation_W_B;

        // If the path doesn't ALREADY start near the odometry, the first waypoint
        // should be the current pose.
        int waypoints_added = 0;
        if (plan_to_start_ &&
            (current_point.position_W - waypoints_.front().position_W).norm() >
                kCloseToOdometry) {
            free_waypoints.push_back(current_point);
            waypoints_added = 1;
        }

        for (const mav_msgs::EigenTrajectoryPoint& waypoint : waypoints_) {
            if (getMapDistance(waypoint.position_W) < constraints_.robot_radius) {
                break;
            }
            free_waypoints.push_back(waypoint);
        }

        printf("[Mav Local Planner] Of %zu waypoints, %zu are free.\n", waypoints_.size(), free_waypoints.size());
        bool success = false;
        if (free_waypoints.size() <= static_cast<size_t>(waypoints_added) ||
            free_waypoints.size() == 2) {
            // Okay whatever just search for the first waypoint.
            success = false;
        } else {
            // There is some hope! Maybe we can do path smoothing on these guys.
            mav_msgs::EigenTrajectoryPointVector path;
            success = planPathThroughWaypoints(free_waypoints, &path);
            if (success) {
                printf("[Mav Local Planner]  Successfully planned path through %zu free waypoints.\n", free_waypoints.size());
                success = isPathCollisionFree(path);
                if (success) {
                    replacePath(path);
                    current_waypoint_ = std::min(free_waypoints.size() - waypoints_added,
                                                 waypoints_.size() - 1);
                    printf("[Mav Local Planner] Used smoothing through %zu waypoints! Total waypoint size: %zu, current point: %zd, added? %d\n",
                           free_waypoints.size(), waypoints_.size(), current_waypoint_, waypoints_added);
                } else {
                    fprintf(stderr, "[Mav Local Planner] But path was not collision free. :(");
                }
            }
        }
        // Give up!
        if (!success) {
            avoidCollisionsTowardWaypoint();
        }
    }
    else {
        // Otherwise let's just keep exploring.
        avoidCollisionsTowardWaypoint();
    }
    printf("[Mav Local Planner][Plan Step] Planning finished. Time taken: %f\n", timer.stop());
    //visualizePath();
}
void MavLocalPlanner::avoidCollisionsTowardWaypoint() {
  if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size())) {
    return;
  }
  mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
  const double kCloseEnough = 0.05;

  const int64_t kDtNs = mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);

  std::cout << "[Mav Local Planner][Plan Step] Current odometry: " << odometry_.position_W.transpose() << " Tracking waypoint ["
                  << current_waypoint_ << "]: " << waypoint.position_W.transpose() << std::endl;

  // Save success and Trajectory.
  mav_trajectory_generation::Trajectory trajectory;
  bool success = false;

  if (!path_queue_.empty()) {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    printf("[Mav Local Planner][Plan Step] Trying to replan on existing path.\n");
    mav_msgs::EigenTrajectoryPointVector path_chunk;
    size_t replan_start_index;
    {
      replan_start_index =
          std::min(path_index_ + static_cast<size_t>((replan_lookahead_sec_) /
                                                     constraints_.sampling_dt),
                   path_queue_.size());
      printf("[Mav Local Planner][Plan Step] Current path index: %zu Replan start index: %zu\n", path_index_, replan_start_index);
      // Cut out the remaining snippet of the trajectory so we can do
      // something with it.
      std::copy(path_queue_.begin() + replan_start_index, path_queue_.end(),
                std::back_inserter(path_chunk));
      if (path_chunk.size() == 0) {
        path_chunk.push_back(path_queue_.back());
        if (!nextWaypoint()) {
          finishWaypoints();
        }
      }
    }

    bool path_chunk_collision_free = isPathCollisionFree(path_chunk);
    printf("[Mav Local Planner][Plan Step] Existing chunk is collision free? %d\n", path_chunk_collision_free);
    // Check if the current path queue goes to the goal and is collision free.
    if ((path_chunk.back().position_W - waypoint.position_W).norm() < kCloseEnough) {
      // Collision check the remaining chunk of the trajectory.
      if (path_chunk_collision_free) {
        printf("[Mav Local Planner][Plan Step] Current plan is valid, just rollin' with it.\n");
        nextWaypoint();
        return;
      }
    }
    // Otherwise we gotta replan this thing anyway.
    success = loco_planner_.getTrajectoryTowardGoal(path_chunk.front(),
                                                    waypoint, &trajectory);
    if (!success) {
      if (path_chunk_collision_free) {
        printf("[Mav Local Planner][Plan Step] Couldn't find a solution :( Continuing existing solution.\n");
      } else {
        printf("[Mav Local Planner][Plan Step] ABORTING! No local solution found.\n");
        // TODO(helenol): which order to abort in?
        abort();
        dealWithFailure();
      }
      return;
    } else {
      printf("[Mav Local Planner][Plan Step] Appending new path chunk.\n");
      if (trajectory.getMaxTime() <= 1e-6) {
        nextWaypoint();
      } else {
        num_failures_ = 0;
        mav_msgs::EigenTrajectoryPointVector new_path_chunk;
        mav_trajectory_generation::sampleWholeTrajectory(
            trajectory, constraints_.sampling_dt, &new_path_chunk);

        retimeTrajectoryWithStartTimeAndDt(
            path_chunk.front().time_from_start_ns, kDtNs, &new_path_chunk);

        new_path_chunk.front().orientation_W_B =
            path_chunk.front().orientation_W_B;
        yaw_policy_.applyPolicyInPlace(&new_path_chunk);

        // Remove what was in the trajectory before.
        if (replan_start_index < path_queue_.size()) {
          path_queue_.erase(path_queue_.begin() + replan_start_index,
                            path_queue_.end());
        }
        // Stick the new one in.
        path_queue_.insert(path_queue_.end(), new_path_chunk.begin(),
                           new_path_chunk.end());
      }
    }
  } else {
    printf("[Mav Local Planner][Plan Step] Trying to plan from scratch.\n");

    // There's nothing planned so far! So we plan from the current odometry.
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    // Check if the current waypoint is basically the odometry.
    if ((current_point.position_W - waypoint.position_W).norm() <
        kCloseEnough) {
      if (nextWaypoint()) {
        waypoint = waypoints_[current_waypoint_];
      } else {
        return;
      }
    }

    success = loco_planner_.getTrajectoryTowardGoal(current_point, waypoint,
                                                    &trajectory);
    printf("[Mav Local Planner][Plan Step] Planning success? %d\n", success);

    if (success) {
      if (trajectory.getMaxTime() <= 0.1) {
        nextWaypoint();
      } else {
        // Copy this straight into the queue.
        num_failures_ = 0;
        mav_msgs::EigenTrajectoryPointVector path;
        mav_trajectory_generation::sampleWholeTrajectory(
            trajectory, constraints_.sampling_dt, &path);
        replacePath(path);
      }
    } else {
      dealWithFailure();
    }
  }
}

// bool MavLocalPlanner::avoidCollisionsTowardWaypoint(mav_planning_msgs::PolynomialTrajectory4D &msg) {
//     if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size())) {
//         return false;
//     }
//     mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
//     const double kCloseEnough = 0.05;

//     const int64_t kDtNs = mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);

//     std::cout << "[Mav Local Planner][Plan Step] Current odometry: " << odometry_.position_W.transpose() << " Tracking waypoint ["
//               << current_waypoint_ << "]: " << waypoint.position_W.transpose() << std::endl;

//     // Save success and Trajectory.
//     mav_trajectory_generation::Trajectory trajectory;
//     bool success = false;

    // mav_planning_msgs::PolynomialTrajectory4D ros_msg;


    // if (!path_queue_.empty()) {
    //     std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    //     printf("[Mav Local Planner][Plan Step] Trying to replan on existing path.\n");
    //     mav_msgs::EigenTrajectoryPointVector path_chunk;
    //     size_t replan_start_index;
    //     {
    //         replan_start_index =
    //             std::min(path_index_ + static_cast<size_t>((replan_lookahead_sec_) /
    //                                                        constraints_.sampling_dt),
    //                      path_queue_.size());
    //         printf("[Mav Local Planner][Plan Step] Current path index: %zu Replan start index: %zu\n", path_index_, replan_start_index);
    //         // Cut out the remaining snippet of the trajectory so we can do
    //         // something with it.
    //         std::copy(path_queue_.begin() + replan_start_index, path_queue_.end(),
    //                   std::back_inserter(path_chunk));
    //         if (path_chunk.size() == 0) {
    //             path_chunk.push_back(path_queue_.back());
    //             if (!nextWaypoint()) {
    //                 finishWaypoints();
    //             }
    //         }
    //     }

    //     bool path_chunk_collision_free = isPathCollisionFree(path_chunk);
    //     printf("[Mav Local Planner][Plan Step] Existing chunk is collision free? %d\n", path_chunk_collision_free);
    //     // Check if the current path queue goes to the goal and is collision free.
    //     if ((path_chunk.back().position_W - waypoint.position_W).norm() < kCloseEnough) {
    //         // Collision check the remaining chunk of the trajectory.
    //         if (path_chunk_collision_free) {
    //             printf("[Mav Local Planner][Plan Step] Current plan is valid, just rollin' with it.\n");
    //             nextWaypoint();
    //             return;
    //         }
    //     }
    //     // Otherwise we gotta replan this thing anyway.
    //     success = loco_planner_.getTrajectoryTowardGoal(path_chunk.front(),
    //                                                     waypoint, &trajectory);
    //     if (!success) {
    //         if (path_chunk_collision_free) {
    //             printf("[Mav Local Planner][Plan Step] Couldn't find a solution :( Continuing existing solution.\n");
    //         } else {
    //             printf("[Mav Local Planner][Plan Step] ABORTING! No local solution found.\n");
    //             // TODO(helenol): which order to abort in?
    //             abort();
    //             dealWithFailure();
    //         }
    //         return;
    //     } else {
    //         printf("[Mav Local Planner][Plan Step] Appending new path chunk.\n");
    //         if (trajectory.getMaxTime() <= 1e-6) {
    //             nextWaypoint();
    //         } else {
    //             num_failures_ = 0;
    //             mav_msgs::EigenTrajectoryPointVector new_path_chunk;
    //             mav_trajectory_generation::sampleWholeTrajectory(
    //                 trajectory, constraints_.sampling_dt, &new_path_chunk);

    //             retimeTrajectoryWithStartTimeAndDt(
    //                 path_chunk.front().time_from_start_ns, kDtNs, &new_path_chunk);

    //             new_path_chunk.front().orientation_W_B =
    //                 path_chunk.front().orientation_W_B;
    //             yaw_policy_.applyPolicyInPlace(&new_path_chunk);

    //             // Remove what was in the trajectory before.
    //             if (replan_start_index < path_queue_.size()) {
    //                 path_queue_.erase(path_queue_.begin() + replan_start_index,
    //                                   path_queue_.end());
    //             }
    //             // Stick the new one in.
    //             path_queue_.insert(path_queue_.end(), new_path_chunk.begin(),
    //                                new_path_chunk.end());
    //         }
    //     }
    // }
    // else {
//         printf("[Mav Local Planner][Plan Step] Trying to plan from scratch.\n");

//         // There's nothing planned so far! So we plan from the current odometry.
//         mav_msgs::EigenTrajectoryPoint current_point;
//         current_point.position_W = odometry_.position_W;
//         current_point.orientation_W_B = odometry_.orientation_W_B;

//         // Check if the current waypoint is basically the odometry.
//         if ((current_point.position_W - waypoint.position_W).norm() <
//             kCloseEnough) {
//             if (nextWaypoint()) {
//                 waypoint = waypoints_[current_waypoint_];
//             } else {
//                 return false;
//             }
//         }

//         success = loco_planner_.getTrajectoryTowardGoal(current_point, waypoint,
//                                                         &trajectory);
//         printf("[Mav Local Planner][Plan Step] Planning success? %d\n", success);
//         // printf("trajectory segments: %d\n", trajectory.segments_.size());

//         if (success) {
//             // success, so create the correct struct
//             mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);


//             if (trajectory.getMaxTime() <= 0.1) {
//                 nextWaypoint();
//             } else {
//                 // Copy this straight into the queue.
//                 num_failures_ = 0;
//                 mav_msgs::EigenTrajectoryPointVector path;
//                 mav_trajectory_generation::sampleWholeTrajectory(
//                     trajectory, constraints_.sampling_dt, &path);
//                 replacePath(path);
//             }
//         } else {
//             dealWithFailure();
//         }
//         return success;
//     // }
// }

bool MavLocalPlanner::planPathThroughWaypoints(const mav_msgs::EigenTrajectoryPointVector& waypoints,
                                               mav_msgs::EigenTrajectoryPointVector* path) {
    if (path == nullptr) {
        fprintf(stderr, "path is null: %s\n", __FUNCTION__);
    }
    bool success = false;
    if (smoother_name_ == "loco") {
        if (waypoints.size() == 2) {
            success = loco_smoother_.getPathBetweenTwoPoints(waypoints[0],
                                                             waypoints[1], path);
        } else {
            success = loco_smoother_.getPathBetweenWaypoints(waypoints, path);
        }
    } else if (smoother_name_ == "polynomial") {
        success = poly_smoother_.getPathBetweenWaypoints(waypoints, path);

    } else if (smoother_name_ == "ramp") {
        success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
    } else {
        // Default case is ramp!
        fprintf(stderr, "[Mav Local Planner] Unknown smoother type %s, using ramp instead.\n", smoother_name_.c_str());
        success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
    }
    return success;
}

bool MavLocalPlanner::nextWaypoint() {
    if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size()) - 1) {
        current_waypoint_ = waypoints_.size() - 1;
        return false;
    } else {
        current_waypoint_++;
        return true;
    }
}

void MavLocalPlanner::finishWaypoints() {
    current_waypoint_ = waypoints_.size();
}

void MavLocalPlanner::replacePath(const mav_msgs::EigenTrajectoryPointVector& path) {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);
    path_queue_.clear();
    path_queue_ = path;
    path_queue_.front().orientation_W_B = odometry_.orientation_W_B;
    yaw_policy_.applyPolicyInPlace(&path_queue_);
    path_index_ = 0;
}

void MavLocalPlanner::abort() {
    // No need to check anything on stop, just clear all the paths.
    clearTrajectory();
    // Make sure to clear the queue in the controller as well (we send about a
    // second of trajectories ahead).
    // sendCurrentPose();
}

void MavLocalPlanner::clearTrajectory() {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);
    // command_publishing_timer_.stop();
    path_queue_.clear();
    path_index_ = 0;
}

double MavLocalPlanner::getMapDistance(const Eigen::Vector3d& position) const {
    double distance = 0.0;
    const bool kInterpolate = false;
    if (!esdf_map->getDistanceAtPosition(
            position, kInterpolate, &distance)) {
        return 0.0;
    }
    return distance;
}

double MavLocalPlanner::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
    double distance = 0.0;
    const bool kInterpolate = false;
    if (!esdf_map->getDistanceAndGradientAtPosition(
            position, kInterpolate, &distance, gradient)) {
        return 0.0;
    }
    return distance;
}

bool MavLocalPlanner::isPathCollisionFree(const mav_msgs::EigenTrajectoryPointVector& path) const {
    for (const mav_msgs::EigenTrajectoryPoint& point : path) {
        if (getMapDistance(point.position_W) < constraints_.robot_radius - 0.1) {
            return false;
        }
    }
    return true;
}

bool MavLocalPlanner::isPathFeasible(const mav_msgs::EigenTrajectoryPointVector& path) const {
    // This is easier to check in the trajectory but then we are limited in how
    // we do the smoothing.
    for (const mav_msgs::EigenTrajectoryPoint& point : path) {
        if (point.acceleration_W.norm() > constraints_.a_max + 1e-2) {
            return false;
        }
        if (point.velocity_W.norm() > constraints_.v_max + 1e-2) {
            return false;
        }
    }
    return true;
}

bool MavLocalPlanner::dealWithFailure() {
    if (current_waypoint_ < 0) {
        return false;
    }

    constexpr double kCloseEnough = 0.05;
    mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
    mav_msgs::EigenTrajectoryPoint goal = waypoint;
    if (temporary_goal_ && static_cast<int64_t>(waypoints_.size()) > current_waypoint_ + 1) {
        goal = waypoints_[current_waypoint_ + 1];
    }
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    mav_msgs::EigenTrajectoryPoint current_goal;
    if (!goal_selector_.selectNextGoal(goal, waypoint, current_point,
                                       &current_goal)) {
        num_failures_++;
        if (num_failures_ > max_failures_) {
            current_waypoint_ = -1;
        }
        return false;
    } else {
        if ((current_goal.position_W - waypoint.position_W).norm() < kCloseEnough) {
            // Goal is unchanged. :(
            temporary_goal_ = false;
            return false;
        } else if ((current_goal.position_W - goal.position_W).norm() <
                   kCloseEnough) {
            // This is just the next waypoint that we're trying to go to.
            current_waypoint_++;
            temporary_goal_ = false;
            return true;
        } else {
            // Then this is something different!
            temporary_goal_ = true;
            waypoints_.insert(waypoints_.begin() + current_waypoint_, current_goal);
            return true;
        }
    }
}

}  // namespace mav_planning
