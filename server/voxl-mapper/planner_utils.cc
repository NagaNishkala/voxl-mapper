#include "planner_utils.h"
#include "config_file.h"

void setupSmootherFromConfig(mav_planning::LocoSmoother &loco_smoother)
{
    /** These are mainly used in the nonlinear smoothing:
     *
     * resample_trajectory:     If true will take the initial guess from the linear solver and resample the
     *                          trajectory to get a new trajectory with num_segments in it that is then
     *                          passed to the nonlinear solver.
     * resample_visibility:     If true will resample before running the linear solver. Uses the visiblity
     *                          graph and a time estimation of the entire path to resample points. the
     *                          visibility graph is essentially the graph made up of the waypoints passed to
     *                          the smoother.
     * num_segments:            The number of segments that the resampled trajectory will have (only applies
     *                          if resample_trajectory is true.
     * add_waypoints:           Adds waypoints into the nonlinear smoother to optimize passing through each
     *                          waypoint. If disabled then waypoint cost weight has no effect.
     * scale_time:              Scales the segment times evenly to ensure that the trajectory is feasible
     *                          given the provided v_max and a_max. Does not change the shape of the trajectory,
     *                          and only increases segment times
     */
    mav_planning::locoParams loco_params;
    loco_params.resample_trajectory_ = loco_resample_trajectory;
    loco_params.resample_visibility_ = loco_resample_visibility;
    loco_params.num_segments_ = loco_num_segments;
    loco_params.add_waypoints_ = loco_add_waypoints;
    loco_params.scale_time_ = loco_scale_time;

    /**
     * Used across both solvers
     *
     * min_col_check_resolution:    Minimum distance between collision checks for BOTH solvers
     * optimize_time:               Runs an additional optimization step (using nlopt) to optimize the segment
     *                              times in order to better meet the dynamic constraints
     * split_at_collisions:         Adds additional points to the trajectory if any portion of the initial linear
     *                              solvers trajectory is in collision
     */
    mav_planning::poly_params poly_params;
    poly_params.min_col_check_resolution = voxel_size;
    poly_params.optimize_time = loco_optimize_time;
    poly_params.split_at_collisions = loco_split_at_collisions;

    /**
     * These are used in the initial linear solver but also used to calculate time of segments for input to the nonlinear:
     *
     * v_max:                       Max velocity of robot
     * a_max:                       Max acceleration of robot
     * yaw_rate_max:                Max yaw rate of robot
     * robot_radius:                Robot radius
     * sampling_dt:                 Time step delta at which to sample points from the trajectory to check for collisions
     *                              this is used in the linear solver and generally only when split at collisions is true
     */
    mav_planning::PhysicalConstraints physical_constraints;
    physical_constraints.v_max = loco_v_max;
    physical_constraints.a_max = loco_a_max;
    physical_constraints.yaw_rate_max = loco_yaw_rate_max;
    physical_constraints.robot_radius = robot_radius;
    physical_constraints.sampling_dt = loco_sampling_dt;

    // Set parameters and callbacks
    loco_smoother.setParameters(loco_params, poly_params, physical_constraints, loco_verbose);
    // loco_smoother.setInCollisionCallback(std::bind(&RRTConnect::detectCollision, this, std::placeholders::_1));
    // loco_smoother.setDistanceAndGradientFunction(std::bind(&RRTConnect::getMapDistanceAndGradient, this, std::placeholders::_1, std::placeholders::_2));

    /**
     * These are only used in the nonlinear solver
     *
     * epsilon:                      Tuning value for how far outside the robot radius we care about collisions. See eq 9
     *                               in https://arxiv.org/pdf/1812.03892.pdf
     * robot_radius:                 Robot radius (doesnt actually need to be set since its set to the constraints above)
     * w_d:                          Weighting for smoothness of derivative we are optimizing for.
     * w_c:                          Weighting for collisions
     * w_w:                          Weighting for waypoints (has no effect if add_waypoints_ is false)
     * min_collision_sampling_dt:    Time step delta at which to evaluate cost/gradient for collisions
     * map_resolution:               Resolution of map
     * verbose:                      Whether to print debug statements or not
     */
    loco_smoother.loco_config.epsilon = 1.0;
    loco_smoother.loco_config.robot_radius = robot_radius;
    loco_smoother.loco_config.w_d = loco_smoothness_cost_weight;
    loco_smoother.loco_config.w_c = loco_collision_cost_weight;
    loco_smoother.loco_config.w_w = loco_waypoint_cost_weight;
    loco_smoother.loco_config.min_collision_sampling_dt = loco_min_collision_sampling_dist;
    loco_smoother.loco_config.map_resolution = voxel_size;
    loco_smoother.loco_config.verbose = loco_verbose;
}

void convertPointsToSmootherFormat(const Point3fVector &points, mav_msgs::EigenTrajectoryPointVector &out)
{
    // sanity checks
    if (points.empty())
    {
        fprintf(stderr, "Point vector is empty!\n");
        return;
    }

    out.reserve(points.size());

    for (size_t i = 0; i < points.size(); i++)
    {
        mav_msgs::EigenTrajectoryPoint curr_pt;
        curr_pt.position_W = points[i].cast<double>();
        out.push_back(curr_pt);
    }
}

bool convertMavTrajectoryToVoxlTrajectory(const mav_trajectory_generation::Trajectory &trajectory, trajectory_t &out)
{
    mav_trajectory_generation::Segment::Vector segments;
    trajectory.getSegments(&segments);

    if (segments.size() > TRAJ_MAX_SEGMENTS)
    {
        fprintf(stderr, "ERROR: Too many segments to copy into trajectory_t\n");
        return false;
    }

    out.n_segments = segments.size();

    for (size_t i = 0; i < segments.size(); ++i) {
        const mav_trajectory_generation::Segment& segment = segments[i];
        
        memcpy(out.segments[i].cx, segment[0].getCoefficients().data(), segment.N() * sizeof(double));
        memcpy(out.segments[i].cy, segment[1].getCoefficients().data(), segment.N() * sizeof(double));
        memcpy(out.segments[i].cz, segment[2].getCoefficients().data(), segment.N() * sizeof(double));
        memcpy(out.segments[i].cyaw, segment[2].getCoefficients().data(), segment.N() * sizeof(double));

        out.segments[i].n_coef = segment.N();
        out.segments[i].duration_s = segment.getTime();
    }

    return true;
}