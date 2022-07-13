#ifndef PLANNER_UTILS_H_
#define PLANNER_UTILS_H_

#include <voxblox/core/esdf_map.h>
#include <mav_path_smoothing/loco_smoother.h>
#include "voxl_trajectory.h"

typedef Eigen::Vector3f Point3f;
typedef Eigen::Vector3d Point3d;
typedef std::vector<Point3f, Eigen::aligned_allocator<Point3f>> Point3fVector;

/**
 * @brief Sets up the smoother from the voxl config file.
 *
 * @param loco_smoother The LocoSmoother object
 */
void setupSmootherFromConfig(mav_planning::LocoSmoother &loco_smoother, const voxblox::EsdfMap *map);

void convertPointsToSmootherFormat(const Point3fVector &points, mav_msgs::EigenTrajectoryPointVector &out);

bool convertMavTrajectoryToVoxlTrajectory(const mav_trajectory_generation::Trajectory &trajectory, trajectory_t &out);

// Map Helper functions
float getMapDistance(const voxblox::EsdfMap *map, const Point3f &position);

bool detectCollision(const voxblox::EsdfMap *map, const Point3f &pos);

bool detectCollisionEdge(const voxblox::EsdfMap *map, const Point3f &start, const Point3f &end, float step_size = -1.0);

bool smootherCollisionCallback(const voxblox::EsdfMap *map, const Point3d &pos);

double smootherDistanceGradientCallback(const voxblox::EsdfMap *map, const Point3d &position, Point3d *gradient);

#endif