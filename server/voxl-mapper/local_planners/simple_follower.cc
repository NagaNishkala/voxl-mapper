#include "simple_follower.h"

#include "voxl_mapper.h"
#include "rc_transform_ringbuf.h"
#include "timing_utils.h"
#include "voxl_trajectory/voxl_trajectory.h"
#include "mav_planning_msgs/mav_planning_msgs.h"
#include "mav_local_planner/conversions.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "path_vis.h"
#include <unistd.h>

#define LOCAL_PLANNER_RATE 0.5
#define MAX_PLAN_DISTANCE 2.0f
#define PLAN_AHEAD_TIME 0.5
#define REACHED_DISTANCE 0.1f

#define PLAN_NAME "plan_msgs"
#define PLAN_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR PLAN_NAME "/")

SimpleFollower::SimpleFollower(std::shared_ptr<voxblox::EsdfMap> esdf_map, int plan_ch, int render_ch)
    : esdf_map_(esdf_map),
      running_(false),
      cur_waypoint_(1),
      plan_ch_(plan_ch),
      render_ch_(render_ch)
{
}

bool SimpleFollower::sendTrajectory(const mav_trajectory_generation::Trajectory &trajectory, int split_id, double split_time, bool first_plan = false)
{
    static int segment_id = 0;

    trajectory_t out;

    if (!convertMavTrajectoryToVoxlTrajectory(trajectory, out))
        return false;

    out.magic_number = TRAJECTORY_MAGIC_NUMBER;

    // Set segment id for each segment whilst incrementing id
    for (int i = 0; i < out.n_segments; i++)
        out.segments[i].id = segment_id++;

    if (first_plan)
    {
        out.traj_command = TRAJ_CMD_LOAD_AND_START;
        current_traj_ = out;
    }
    else
    {
        if (split_id < 0 || split_time < 0)
        {
            fprintf(stderr, "Invalid split id or split time\n");
            fprintf(stderr, "split id = %d | split time = %f\n", split_id, split_time);
            return false;
        }

        out.traj_command = TRAJ_CMD_INSERT;
        out.segment_split_id = split_id;
        out.segment_split_time = split_time;

        // Trim to make sure we eliminate unneccessary segments
        trim_trajectory(&current_traj_, cur_segment_id_);

        // Attempt to insert, only send if successful
        if (!insert_trajectory(&current_traj_, &out))
            return false;
    }

    printf("Sending trajectory to plan channel\n");
    pipe_server_write(plan_ch_, &out, sizeof(trajectory_t));
    return true;
}

void SimpleFollower::visualizePath(const Point3fVector &target_points)
{
    std::vector<path_vis_t> waypoints;
    for (const Point3f &p : target_points)
    {
        path_vis_t pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        pt.r = 0;
        pt.g = 255;
        pt.b = 0;
        waypoints.push_back(pt);
    }

    generateAndSendPath(render_ch_, waypoints, PATH_VIS_LINE, "SimpleFollower Waypoints");
    usleep(100000);

    std::vector<path_vis_t> traj;
    double sampling_dt = 0.05;

    for (int i = 0; i < current_traj_.n_segments; i++)
    {
        poly_segment_t *s = &current_traj_.segments[i];
        for (double t = 0.0; t < s->duration_s; t += sampling_dt)
        {
            path_vis_t pt;
            pt.x = eval_poly_at_t(s->n_coef, s->cx, t);
            pt.y = eval_poly_at_t(s->n_coef, s->cy, t);
            pt.z = eval_poly_at_t(s->n_coef, s->cz, t);
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            traj.push_back(pt);
        }
    }

    generateAndSendPath(render_ch_, traj, PATH_VIS_TRAJECTORY, "SimpleFollower Smoothed");
}

bool SimpleFollower::runSmoother(const Point3fVector &target_points, mav_trajectory_generation::Trajectory &trajectory)
{
    mav_msgs::EigenTrajectoryPointVector waypoints_for_smoother;
    convertPointsToSmootherFormat(target_points, waypoints_for_smoother);

    return loco_smoother_.getTrajectoryBetweenWaypoints(waypoints_for_smoother, &trajectory);
}

void SimpleFollower::setup()
{
    setupSmootherFromConfig(loco_smoother_);

    loco_smoother_.setInCollisionCallback(std::bind(&SimpleFollower::detectCollision, this, std::placeholders::_1));
    loco_smoother_.setDistanceAndGradientFunction(std::bind(&SimpleFollower::getMapDistanceAndGradient, this, std::placeholders::_1, std::placeholders::_2));
}

void SimpleFollower::start()
{
    // Ignore request to start if already running, otherwise check if previous thread needs cleanup
    if (running_)
        return;
    else if(planning_thread_.joinable())
        planning_thread_.join();

    cur_waypoint_ = 1;
    running_ = true;
    planning_thread_ = std::thread(&SimpleFollower::plannerThread, this);
}

void SimpleFollower::stop()
{
    if (!running_)
        return;

    running_ = false;

    if (planning_thread_.joinable())
        planning_thread_.join();
}

void SimpleFollower::computeSplitPointDetails(Point3f &start_pose, int &split_id, double &split_time)
{
    poly_segment_t *last_segment = &current_traj_.segments[current_traj_.n_segments - 1];
    float x = eval_poly_at_t(last_segment->n_coef, last_segment->cx, last_segment->duration_s);
    float y = eval_poly_at_t(last_segment->n_coef, last_segment->cy, last_segment->duration_s);
    float z = eval_poly_at_t(last_segment->n_coef, last_segment->cz, last_segment->duration_s);

    // Default values will be the end of the segment
    start_pose << x, y, z;
    split_id = last_segment->id;
    split_time = last_segment->duration_s;

    pthread_mutex_lock(&segment_mutex);
    double t = cur_segment_t_;
    int segment_idx = -1;
    for (int i = 0; i < current_traj_.n_segments; i++)
    {
        if (current_traj_.segments[i].id == cur_segment_id_)
        {
            segment_idx = i;
            break;
        }
    }

    if (segment_idx < 0)
    {
        fprintf(stderr, "ERROR: Segment id %d does not exist in current trajectory. Planning from end of trajectory\n", cur_segment_id_);
        pthread_mutex_unlock(&segment_mutex);
        return;
    }

    fprintf(stderr, "starting t = %f | id = %d\n", cur_segment_t_, cur_segment_id_);
    t = current_traj_.segments[segment_idx].duration_s - t;

    if (t > PLAN_AHEAD_TIME)
    {
        split_time = cur_segment_t_ + PLAN_AHEAD_TIME;
        pthread_mutex_unlock(&segment_mutex);
        split_id = current_traj_.segments[segment_idx].id;

        poly_segment_t *s = &current_traj_.segments[current_traj_.n_segments - 1];
        float x = eval_poly_at_t(s->n_coef, s->cx, s->duration_s);
        float y = eval_poly_at_t(s->n_coef, s->cy, s->duration_s);
        float z = eval_poly_at_t(s->n_coef, s->cz, s->duration_s);

        start_pose << x, y, z;
        return;
    }

    for (int i = segment_idx + 1; i < current_traj_.n_segments; i++)
    {
        t += current_traj_.segments[i].duration_s;

        if (t > PLAN_AHEAD_TIME)
        {
            t -= current_traj_.segments[i].duration_s;

            split_id = current_traj_.segments[i].id;
            split_time = PLAN_AHEAD_TIME - t;

            poly_segment_t *s = &current_traj_.segments[i];
            float x = eval_poly_at_t(s->n_coef, s->cx, split_time);
            float y = eval_poly_at_t(s->n_coef, s->cy, split_time);
            float z = eval_poly_at_t(s->n_coef, s->cz, split_time);
            start_pose << x, y, z;
            pthread_mutex_unlock(&segment_mutex);
            return;
        }
    }

    // We will only reach here if PLAN_AHEAD_TIME occurs past end of the current trajectory
    // so the default values are correct
    pthread_mutex_unlock(&segment_mutex);
}

bool SimpleFollower::getInitialPlan()
{
    // Get current pose
    rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
    int ret = rc_tf_ringbuf_get_tf_at_time(&voxblox::TsdfServer::buf, rc_nanos_monotonic_time(), &tf_body_wrt_fixed);
    if (ret < 0)
    {
        fprintf(stderr, "LocalPlanner: ERROR fetching tf from tf ringbuffer\n");
        if (ret == -2)
        {
            printf("there wasn't sufficient data in the buffer\n");
        }
        if (ret == -3)
        {
            printf("the requested timestamp was too new\n");
        }
        if (ret == -4)
        {
            printf("the requested timestamp was too old\n");
        }
        return false;
    }

    Point3f cur_pose(tf_body_wrt_fixed.d[0][3], tf_body_wrt_fixed.d[1][3], tf_body_wrt_fixed.d[2][3]);

    Point3fVector target_points;
    mav_trajectory_generation::Trajectory trajectory;

    if(!runPlanner(cur_pose, target_points, trajectory))
        return false;

    if (!sendTrajectory(trajectory, -1, -1, true))
    {
        fprintf(stderr, "Local Planner failed to send initial trajectory\n");
        return false;
    }

    visualizePath(target_points);

    return true;
}

bool SimpleFollower::isBetween(Point3f a, Point3f b, Point3f c)
{
    Point3f ab = b - a;
    Point3f ac = c - a;

    float dot_ac = ab.dot(ac);
    float dot_ab = ab.dot(ab);

    if (dot_ac < 0 || dot_ac > dot_ab)
        return false;
    else
        return true;
}

bool SimpleFollower::runPlanner(Point3f start_pose, Point3fVector &target_points, mav_trajectory_generation::Trajectory &trajectory)
{
    // Start trajectory from start pose
    target_points.push_back(start_pose);

    // Find closest line segment
    float min_dist = __FLT_MAX__;
    for (size_t i = cur_waypoint_; i < waypoints_.size() - 1; i++)
    {
        Point3f p1 = waypoints_[i];
        Point3f p2 = waypoints_[i + 1];
        Eigen::ParametrizedLine<float, 3> line = Eigen::ParametrizedLine<float, 3>::Through(p1, p2);

        // Check if the projection of start pose is within the bounds of the line
        if (isBetween(p1, p2, line.projection(start_pose)))
        {
            // Save the end of the line we are closest to as our target
            float d = line.squaredDistance(start_pose);
            if (d < min_dist)
            {
                min_dist = d;
                cur_waypoint_ = i + 1;
            }
        }
    }

    target_points.push_back(waypoints_[cur_waypoint_]);

    // Add additional waypoints for a longer planning distance
    float total_dist = (start_pose - waypoints_[cur_waypoint_]).norm();
    float prev_dist = 0;

    for(size_t i = cur_waypoint_ + 1; i < waypoints_.size(); i++)
    {
        prev_dist = total_dist;
        total_dist += (waypoints_[i] - waypoints_[i - 1]).norm();

        if (total_dist < MAX_PLAN_DISTANCE)
        {
            target_points.push_back(waypoints_[i]);
        }
        else
        {
            float remaining_dist = MAX_PLAN_DISTANCE - prev_dist;
            Point3f intermediate_point = waypoints_[i - 1] + (waypoints_[i] - waypoints_[i - 1]).normalized() * remaining_dist;
            target_points.push_back(intermediate_point);
            break;
        }
    }

    if (!runSmoother(target_points, trajectory))
    {
        fprintf(stderr, "Smoother failed in local planner. Skipping..\n");
        return false;
    }

    return true;
}

void SimpleFollower::plannerThread()
{

    if (!getInitialPlan())
    {
        fprintf(stderr, "Failed to find initial plan. Exiting..\n");
        running_ = false;
        return;
    }

    while (running_)
    {
        loop_sleep(LOCAL_PLANNER_RATE);

        // TODO: CHANGE THIS CHECK TO BE DIST TO GOAL
        // If we sent a trajectory to the end goal then exit
        if (cur_waypoint_ == waypoints_.size() - 1)
        {
            fprintf(stderr, "Local Planner finished\n");
            running_ = false;
            return;
        }

        // Proceed with the planner
        Point3f start_pose;
        int split_id = -1;
        double split_time = -1.0;

        computeSplitPointDetails(start_pose, split_id, split_time);

        Point3fVector target_points;
        mav_trajectory_generation::Trajectory trajectory;

        runPlanner(start_pose, target_points, trajectory);

        if (!sendTrajectory(trajectory, split_id, split_time))
        {
            fprintf(stderr, "Local Planner failed to send trajectory\n");
            continue;
        }

        visualizePath(target_points);
    }

    running_ = false;
    return;
}

void SimpleFollower::setPlan(const Point3fVector &waypoints)
{
    // Dont allow setting of waypoints if the local planner is already running
    if (running_)
    {
        fprintf(stderr, "ERROR: Attempted to set new waypoints whilst local planner was running\n");
        return;
    }

    // Copy the waypoints
    waypoints_ = Point3fVector(waypoints);
}

bool SimpleFollower::detectCollision(const Point3d &pos)
{
    return getMapDistance(pos) <= robot_radius;
}

float SimpleFollower::getMapDistance(const Point3d &position)
{
    double dist = 0.0;
    if (!(esdf_map_->getDistanceAtPosition(position, false, &dist)))
    {
        // if we cannot identify a voxel close enough to this location WITHOUT interpolation, it is unknown so reject it
        if (rrt_treat_unknown_as_occupied)
            dist = 0.0;
        else
            dist = esdf_default_distance;
    }

    return dist;
}

double SimpleFollower::getMapDistanceAndGradient(const Point3d &position, Point3d *gradient)
{
    double distance = 0.0;
    if (!(esdf_map_->getDistanceAndGradientAtPosition(position, false, &distance, gradient)))
    {
        return 0.0;
    }
    return distance;
}

SimpleFollower::~SimpleFollower()
{
    stop();
}