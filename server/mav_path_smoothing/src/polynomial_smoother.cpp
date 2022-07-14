#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/feasibility_analytic.h>

#include "mav_path_smoothing/polynomial_smoother.h"

namespace mav_planning {

PolynomialSmoother::PolynomialSmoother()
    : PathSmootherBase(),
      optimize_time_(true),
      split_at_collisions_(true),
      min_col_check_resolution_(0.1) {}

void PolynomialSmoother::setParameters(poly_params pp, PhysicalConstraints constraints, bool verbose) {
    PathSmootherBase::setParameters(constraints, verbose);

    optimize_time_ = pp.optimize_time;
    split_at_collisions_ = pp.split_at_collisions;
    min_col_check_resolution_ = pp.min_col_check_resolution;
}

bool PolynomialSmoother::getTrajectoryBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
    mav_trajectory_generation::Trajectory* trajectory) const {
  if (waypoints.size() < 2) {
    return false;
  }

  // I guess this is only if method is linear.
  mav_trajectory_generation::timing::Timer linear_timer(
      "smoothing/poly_linear");

  // Polynomials should always be of degree kPolynomialDegree 
  // Polynomials are need for three dimensions (x, y, z)
  constexpr int N = mav_trajectory_generation::kPolynomialDegree;
  constexpr int D = 3;

  mav_trajectory_generation::PolynomialOptimization<N> poly_opt(D);

  int num_vertices = waypoints.size();

  int derivative_to_optimize = mav_trajectory_generation::kDerivativeToOptimize;

  mav_trajectory_generation::Vertex::Vector vertices(
      num_vertices, mav_trajectory_generation::Vertex(D));

  // Add the first and last.
  vertices.front().makeStartOrEnd(0, derivative_to_optimize);

  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.front().position_W);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      waypoints.front().velocity_W);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      waypoints.front().acceleration_W);

  vertices.back().makeStartOrEnd(0, derivative_to_optimize);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.back().position_W);

  // Now do the middle bits.
  size_t j = 1;
  for (size_t i = 1; i < waypoints.size() - 1; i += 1) {
    vertices[j].addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        waypoints[i].position_W);
    j++;
  }

  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimesVelocityRamp(
          vertices, constraints_.v_max, constraints_.a_max, 1.2);

  poly_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  if (poly_opt.solveLinear()) {
    poly_opt.getTrajectory(trajectory);
  } else {
    return false;
  }
  linear_timer.Stop();

  if (optimize_time_) {
    mav_trajectory_generation::timing::Timer time_opt_timer(
        "smoothing/poly_time_opt");
    mav_trajectory_generation::NonlinearOptimizationParameters nlopt_parameters;
    nlopt_parameters.algorithm = nlopt::LD_LBFGS;
    nlopt_parameters.time_alloc_method = mav_trajectory_generation::
        NonlinearOptimizationParameters::kMellingerOuterLoop;  // HERE: mellinger outer loop
    nlopt_parameters.print_debug_info_time_allocation = true;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> nlopt(
        D, nlopt_parameters);
    nlopt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    nlopt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::VELOCITY,
        constraints_.v_max);
    nlopt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::ACCELERATION,
        constraints_.a_max);
    nlopt.optimize();
    nlopt.getTrajectory(trajectory);
  }

  // Ok now do more stuff, if the method requires.
  if (split_at_collisions_) {
    mav_trajectory_generation::timing::Timer split_timer(
        "smoothing/poly_split");

    mav_msgs::EigenTrajectoryPoint::Vector path;

    // Sample it!
    double dt = constraints_.sampling_dt;
    mav_trajectory_generation::sampleWholeTrajectory(*trajectory, dt, &path);

    // Check if it's in collision.
    double t = 0.0;
    bool path_in_collision = isPathInCollision(path, &t);

    const int kMaxNumberOfAdditionalVertices = 10;
    int num_added = 0;
    while (path_in_collision) {
      if (!addVertex(t, *trajectory, &vertices, &segment_times)) {
        // Well this isn't going anywhere.
        return false;
      }
      poly_opt.setupFromVertices(vertices, segment_times,
                                 derivative_to_optimize);
      poly_opt.solveLinear();
      poly_opt.getTrajectory(trajectory);
      mav_trajectory_generation::sampleWholeTrajectory(*trajectory, dt, &path);
      path_in_collision = isPathInCollision(path, &t);
      num_added++;
      if (num_added > kMaxNumberOfAdditionalVertices) {
        break;
      }
    }
    // printf("[SPLIT SMOOTHING] Added %d additional vertices.\n", num_added);

    // If we had to do this, let's scale the times back to make sure we're
    // still within constraints.
    if (optimize_time_) {
      trajectory->scaleSegmentTimesToMeetConstraints(constraints_.v_max,
                                                     constraints_.a_max);
    }
  }

  double v_max, a_max;
  trajectory->computeMaxVelocityAndAcceleration(&v_max, &a_max);

//   printf("[SMOOTHING] V max/limit: %f/%f, A max/limit: %f/%f\n", v_max,
//            constraints_.v_max, a_max, constraints_.a_max);

  return true;
}

bool PolynomialSmoother::getPathBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPointVector& waypoints,
    mav_msgs::EigenTrajectoryPoint::Vector* path) const {
  mav_trajectory_generation::Trajectory trajectory;
  bool success = getTrajectoryBetweenWaypoints(waypoints, &trajectory);
  if (success) {
    //   fprintf(stderr, "getTrajectoryBetweenWaypoints succeded\n");
    mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, constraints_.sampling_dt, path);
    return true;
  }
  return false;
}

bool PolynomialSmoother::getPathBetweenTwoPoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPoint::Vector* path) const {
  mav_msgs::EigenTrajectoryPoint::Vector waypoints;
  waypoints.push_back(start);
  waypoints.push_back(goal);

  return getPathBetweenWaypoints(waypoints, path);
}

bool PolynomialSmoother::addVertex(
    double t, const mav_trajectory_generation::Trajectory& trajectory,
    mav_trajectory_generation::Vertex::Vector* vertices,
    std::vector<double>* segment_times) const {
  // First, go through the trajectory segments and figure out between which two
  // segments the new vertex will lie.
  const mav_trajectory_generation::Segment::Vector& segments =
      trajectory.segments();

  double time_so_far = 0.0;
  size_t seg_ind = 0;
  for (seg_ind = 0; seg_ind < segments.size(); ++seg_ind) {
    time_so_far += segments[seg_ind].getTime();
    if (time_so_far > t) {
      break;
    }
  }

  // Relative time within the segment.
  double seg_max_time = segments[seg_ind].getTime();
  double rel_time_sec = t - time_so_far + seg_max_time;
  // Get the start and goal positions of those segments.
  Eigen::VectorXd start_pos = segments[seg_ind].evaluate(0.0);
  Eigen::VectorXd end_pos = segments[seg_ind].evaluate(seg_max_time);

  // Get the relative time of the new vertex (relative to the start vertex),
  // and make sure it's not too close to the start or end to avoid numerical
  // issues.
  double rel_time_scaled = (rel_time_sec / seg_max_time);
  constexpr double kRelativeTimeMargin = 0.1;
  constexpr double kMinTimeSec = 0.1;
  rel_time_scaled =
      std::max(std::min(rel_time_scaled, 1.0 - kRelativeTimeMargin),
               kRelativeTimeMargin);
  Eigen::VectorXd new_pos =
      (-start_pos + end_pos) * rel_time_scaled + start_pos;

  if (isPositionInCollision(new_pos.head<3>())) {
    // fprintf(stderr, "[SMOOTH ERROR] Point along the straight line is occupied. Polynomial won't be collision-free.\n");
    return false;
  }

  rel_time_sec = std::max(rel_time_scaled * seg_max_time, kMinTimeSec);

  // Add the vertex with the correct constraints.
  mav_trajectory_generation::Vertex new_vertex = (*vertices)[seg_ind];
  new_vertex.addConstraint(
      mav_trajectory_generation::derivative_order::POSITION, new_pos);
  vertices->insert(vertices->begin() + seg_ind + 1, new_vertex);

  // Add the segment time in.
  segment_times->insert(segment_times->begin() + seg_ind, rel_time_sec);
  (*segment_times)[seg_ind + 1] =
      std::max(seg_max_time - rel_time_sec, kMinTimeSec);

  return true;
}

// Uses whichever collision checking method is set to check for collisions.
bool PolynomialSmoother::isPositionInCollision(
    const Eigen::Vector3d& pos) const {
  if (map_distance_func_) {
    return map_distance_func_(pos) < constraints_.robot_radius;
  }
  if (in_collision_func_) {
    return in_collision_func_(pos);
  }
  return false;
}

bool PolynomialSmoother::isPathInCollision(
    const mav_msgs::EigenTrajectoryPoint::Vector& path, double* t) const {
  if (path.size() < 1) {
    return true;
  }
  double distance_since_last_check = 0.0;
  Eigen::Vector3d last_pos = path[0].position_W;

  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    distance_since_last_check += (point.position_W - last_pos).norm();
    if (distance_since_last_check > min_col_check_resolution_) {
      if (isPositionInCollision(point.position_W)) {
        if (t != NULL) {
          *t = mav_msgs::nanosecondsToSeconds(point.time_from_start_ns);
        }
        return true;
      }
      last_pos = point.position_W;
      distance_since_last_check = 0.0;
    }
  }
  return false;
}

}  // namespace mav_planning
