#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>

#include "voxblox_skeleton_planner/skeleton_graph_planner.h"

namespace mav_planning {

SkeletonGraphPlanner::SkeletonGraphPlanner(double robot_radius, bool verbose, bool shorten_path)
    : robot_radius_(robot_radius),
      verbose_(verbose),
      shorten_path_(shorten_path) {
  setRobotRadius(robot_radius_);
  skeleton_planner_.setMaxIterations(10000);
}

void SkeletonGraphPlanner::setRobotRadius(double robot_radius) {
  robot_radius_ = robot_radius;

  skeleton_planner_.setMinEsdfDistance(robot_radius);

  mav_planning::PhysicalConstraints constraints;
  constraints.robot_radius = robot_radius_;
  path_shortener_.setConstraints(constraints);
}

void SkeletonGraphPlanner::setEsdfLayer(
    voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer) {
  // Set up the A* planners.
  skeleton_planner_.setEsdfLayer(esdf_layer);

  // Set up shortener.
  path_shortener_.setEsdfLayer(esdf_layer);
}

void SkeletonGraphPlanner::setSkeletonLayer(
    voxblox::Layer<voxblox::SkeletonVoxel>* skeleton_layer) {
  skeleton_planner_.setSkeletonLayer(skeleton_layer);
}

void SkeletonGraphPlanner::setSparseGraph(
    voxblox::SparseSkeletonGraph* sparse_graph) {
  sparse_graph_planner_.setGraph(sparse_graph);
  mav_trajectory_generation::timing::Timer kd_tree_init("skeleton_plan/setup");
  sparse_graph_planner_.setup();
  kd_tree_init.Stop();
}

// Fixed start and end locations, returns list of waypoints between.
bool SkeletonGraphPlanner::getPathBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPoint::Vector* solution) {
  voxblox::Point start_point = start.position_W.cast<float>();
  voxblox::Point goal_point = goal.position_W.cast<float>();

  mav_trajectory_generation::timing::Timer graph_timer("skeleton_plan");
  voxblox::AlignedVector<voxblox::Point> graph_coordinate_path;
  bool success = sparse_graph_planner_.getPath(start_point, goal_point,
                                               &graph_coordinate_path);
  mav_msgs::EigenTrajectoryPointVector graph_path;
  convertCoordinatePathToPath(graph_coordinate_path, &graph_path);
  printf("Got sparse graph path.\n");
  if (!success) {
    return false;
  }

  voxblox::AlignedVector<voxblox::Point> exact_start_path, exact_goal_path;

  success &= skeleton_planner_.getPathInEsdf(
      start_point, graph_coordinate_path.front(), &exact_start_path);
  success &= skeleton_planner_.getPathInEsdf(graph_coordinate_path.back(),
                                             goal_point, &exact_goal_path);

  graph_coordinate_path.insert(graph_coordinate_path.begin(),
                               exact_start_path.begin(),
                               exact_start_path.end());
  graph_coordinate_path.insert(graph_coordinate_path.end(),
                               exact_goal_path.begin(), exact_goal_path.end());
  convertCoordinatePathToPath(graph_coordinate_path, &graph_path);
  printf("Got ESDF path.\n");

  if (shorten_path_) {
    shortenPath(graph_path, solution);
  } else {
    *solution = graph_path;
  }
  return success;
}

void SkeletonGraphPlanner::convertCoordinatePathToPath(
    const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) const {
  if(path == nullptr){
      fprintf(stderr, "path is a nullptr: %s\n", __FUNCTION__);
      return;
  }
  path->clear();
  path->reserve(coordinate_path.size());

  for (const voxblox::Point& voxblox_point : coordinate_path) {
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = voxblox_point.cast<double>();
    path->push_back(point);
  }
}

void SkeletonGraphPlanner::shortenPath(
    const mav_msgs::EigenTrajectoryPoint::Vector& path_in,
    mav_msgs::EigenTrajectoryPoint::Vector* path_out) const {
  path_shortener_.shortenPath(path_in, path_out);
}

}  // namespace mav_planning
