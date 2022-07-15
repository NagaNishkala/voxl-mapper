#include "mav_path_smoothing/loco_smoother.h"

#include <loco_planner/loco.h>
#include <mav_planning_common/visibility_resampling.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

namespace mav_planning
{

    LocoSmoother::LocoSmoother()
        : PolynomialSmoother(),
          resample_trajectory_(false),
          resample_visibility_(false),
          num_segments_(3),
          add_waypoints_(false),
          scale_time_(true)
    {}

    void LocoSmoother::setParameters(locoParams loco_params, poly_params pp, PhysicalConstraints constraints, bool verbose)
    {
        resample_trajectory_ = loco_params.resample_trajectory_;
        resample_visibility_ = loco_params.resample_visibility_;
        num_segments_ = loco_params.num_segments_;
        add_waypoints_ = loco_params.add_waypoints_;
        scale_time_ = loco_params.scale_time_;

        PolynomialSmoother::setParameters(pp, constraints, verbose);
    }

    void LocoSmoother::setParameters(locoParams loco_params) {
        resample_trajectory_ = loco_params.resample_trajectory_;
        resample_visibility_ = loco_params.resample_visibility_;
        num_segments_ = loco_params.num_segments_;
        add_waypoints_ = loco_params.add_waypoints_;
        scale_time_ = loco_params.scale_time_;
    }

    bool LocoSmoother::getTrajectoryBetweenWaypoints(
        const mav_msgs::EigenTrajectoryPoint::Vector &waypoints,
        mav_trajectory_generation::Trajectory *trajectory, bool only_linear) const
    {
        // If there's less than 3 waypoints, there are no free variables for loco.
        if (waypoints.size() < 2)
        {
            return false;
        }
        if (waypoints.size() == 2)
        {
            return PolynomialSmoother::getTrajectoryBetweenWaypoints(waypoints,
                                                                     trajectory);
        }
        mav_trajectory_generation::timing::Timer loco_timer("smoothing/poly_loco");

        // Create a loco object! So Loco!
        mav_trajectory_generation::Trajectory traj_initial;
        if (resample_visibility_)
        {
            // If resampling the visibility graph, then basically divide the whole thing
            // into evenly spaced waypoints on the graph.
            mav_msgs::EigenTrajectoryPoint::Vector resampled_waypoints;
            resampleWaypointsFromVisibilityGraph(num_segments_, constraints_, waypoints,
                                                 &resampled_waypoints);
            PolynomialSmoother::getTrajectoryBetweenWaypoints(resampled_waypoints,
                                                              &traj_initial);
        }
        else
        {
            PolynomialSmoother::getTrajectoryBetweenWaypoints(waypoints, &traj_initial);
        }

        if(only_linear)
        {
            *trajectory = traj_initial;
            return true;
        }


        // Polynomials should always be of degree kPolynomialDegree 
        // Polynomials are need for three dimensions (x, y, z)
        constexpr int N = mav_trajectory_generation::kPolynomialDegree;
        constexpr int D = 3;

        loco_planner::Loco<N> loco(D, loco_config);
        // This is because our initial solution is nearly collision-free.
        // loco.setWd(0.1);

        loco.setRobotRadius(constraints_.robot_radius);
        if (distance_and_gradient_function_)
        {
            loco.setDistanceAndGradientFunction(
                std::bind(&LocoSmoother::getMapDistanceAndGradient, this,
                          std::placeholders::_1, std::placeholders::_2));
        }
        else
        {
            loco.setDistanceFunction(map_distance_func_);
        }

        if (resample_trajectory_ && !resample_visibility_)
        {
            loco.setupFromTrajectoryAndResample(traj_initial, num_segments_);
        }
        else
        {
            loco.setupFromTrajectory(traj_initial);
        }
        if (add_waypoints_)
        {
            loco.setWaypointsFromTrajectory(traj_initial);
        }

        loco.solveProblem();
        loco.getTrajectory(trajectory);

        if (scale_time_)
        {
            trajectory->scaleSegmentTimesToMeetConstraints(constraints_.v_max,
                                                           constraints_.a_max);
        }

        return true;
    }

    bool LocoSmoother::getTrajectoryBetweenTwoPoints(
        const mav_msgs::EigenTrajectoryPoint &start,
        const mav_msgs::EigenTrajectoryPoint &goal,
        mav_trajectory_generation::Trajectory *trajectory) const
    {
        mav_trajectory_generation::timing::Timer loco_timer("smoothing/poly_loco");

        if (trajectory == nullptr)
        {
            fprintf(stderr, "error, trajectory is nullptr: %s\n", __FUNCTION__);
            return false;
        }

        // Polynomials should always be of degree kPolynomialDegree 
        // Polynomials are need for three dimensions (x, y, z)
        constexpr int N = mav_trajectory_generation::kPolynomialDegree;
        constexpr int D = 3;
        loco_planner::Loco<N> loco(D);
        loco.setWd(0.1);

        loco.setRobotRadius(constraints_.robot_radius);
        loco.setMapResolution(min_col_check_resolution_);
        if (distance_and_gradient_function_)
        {
            loco.setDistanceAndGradientFunction(
                std::bind(&LocoSmoother::getMapDistanceAndGradient, this,
                          std::placeholders::_1, std::placeholders::_2));
        }
        else
        {
            loco.setDistanceFunction(map_distance_func_);
        }

        double total_time = mav_trajectory_generation::computeTimeVelocityRamp(
            start.position_W, goal.position_W, constraints_.v_max,
            constraints_.a_max);
        loco.setupFromTrajectoryPoints(start, goal, num_segments_, total_time);
        loco.solveProblem();
        loco.getTrajectory(trajectory);

        if (optimize_time_)
        {
            trajectory->scaleSegmentTimesToMeetConstraints(constraints_.v_max,
                                                           constraints_.a_max);
        }
        return true;
    }

    bool LocoSmoother::getPathBetweenTwoPoints(
        const mav_msgs::EigenTrajectoryPoint &start,
        const mav_msgs::EigenTrajectoryPoint &goal,
        mav_msgs::EigenTrajectoryPoint::Vector *path) const
    {
        if (path == nullptr)
        {
            fprintf(stderr, "path is a nullptr: %s\n", __FUNCTION__);
            return false;
        }
        mav_trajectory_generation::Trajectory trajectory;
        bool success = getTrajectoryBetweenTwoPoints(start, goal, &trajectory);
        if (success)
        {
            mav_trajectory_generation::sampleWholeTrajectory(
                trajectory, constraints_.sampling_dt, path);
            return true;
        }
        return false;
    }

    double LocoSmoother::getMapDistanceAndGradient(
        const Eigen::VectorXd &position, Eigen::VectorXd *gradient) const
    {
        if (distance_and_gradient_function_ == NULL)
        {
            fprintf(stderr, "distance and gradient function is null: %s\n", __FUNCTION__);
            return -1.0;
        }
        if (position.size() != 3)
        {
            fprintf(stderr, "position size is != 3: %s\n", __FUNCTION__);
            return -1.0;
        }
        if (gradient == nullptr)
        {
            return distance_and_gradient_function_(position, nullptr);
        }
        Eigen::Vector3d gradient_3d;
        double distance = distance_and_gradient_function_(position, &gradient_3d);
        *gradient = gradient_3d;
        return distance;
    }

} // namespace mav_planning
