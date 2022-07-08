#ifndef PLANNER_UTILS_H_
#define PLANNER_UTILS_H_

#include <mav_path_smoothing/loco_smoother.h>
#include "voxl_trajectory.h"

typedef Eigen::Vector3f Point3f;
typedef Eigen::Vector3d Point3d;
typedef std::vector<Point3f, Eigen::aligned_allocator<Point3f>> Point3fVector;

/**
 * @brief Sets up the smoother from the voxl config file.
 * 
 * NOTE:  You still need to setup setInCollisionCallback and setDistanceAndGradientFunction
 *        for the smoother
 * 
 * @param loco_smoother The LocoSmoother object
 */
void setupSmootherFromConfig(mav_planning::LocoSmoother &loco_smoother);

void convertPointsToSmootherFormat(const Point3fVector &points, mav_msgs::EigenTrajectoryPointVector &out);

bool convertMavTrajectoryToVoxlTrajectory(const mav_trajectory_generation::Trajectory &trajectory, trajectory_t &out);

#endif