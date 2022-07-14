#include "local_a_star.h"

#include "voxl_mapper.h"
#include "rc_transform_ringbuf.h"
#include "timing_utils.h"
#include "voxl_trajectory/voxl_trajectory.h"
#include "mav_planning_msgs/mav_planning_msgs.h"
#include "mav_local_planner/conversions.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "path_vis.h"

// TODO Remove
#include <voxblox/core/esdf_map.h>

#define LOCAL_PLANNER_RATE 1
#define REACHED_DISTANCE 0.05f
#define MAX_TRAVEL_COST 15
#define PLAN_AHEAD_TIME 0.25
#define MAX_PLAN_DISTANCE 2

#define PLAN_NAME "plan_msgs"
#define PLAN_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR PLAN_NAME "/")

LocalAStar::LocalAStar(voxblox::TsdfServer *map, int plan_ch, int render_ch)
    : map_(map),
      running_(false),
      cur_waypoint_(1),
      plan_ch_(plan_ch),
      render_ch_(render_ch)
{
    current_traj_.n_segments = 0;
    current_traj_.traj_command = 0;
    current_traj_.segment_split_id = 0;
    current_traj_.segment_split_time = 0.0;
}

bool LocalAStar::sendTrajectory(const mav_trajectory_generation::Trajectory &trajectory, int split_id, double split_time, bool first_plan = false)
{
    static int segment_id = 0;

    trajectory_t out;

    if (!convertMavTrajectoryToVoxlTrajectory(trajectory, out))
        return false;

    out.magic_number = TRAJECTORY_MAGIC_NUMBER;

    // Set segment id for each segment whilst incrementing id
    for (int i = 0; i < out.n_segments; i++)
        out.segments[i].id = segment_id++;

    // fprintf(stderr, "Before\n");

    // for (int i = 0; i < current_traj_.n_segments; i++)
    // {
    //     fprintf(stderr, "Id = %d | dur = %f\n", current_traj_.segments[i].id, current_traj_.segments[i].duration_s);
    // }
    // fprintf(stderr, "\n--------\n");

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

    // fprintf(stderr, "After\n");

    // for (int i = 0; i < current_traj_.n_segments; i++)
    // {
    //     fprintf(stderr, "Id = %d | dur = %f\n", current_traj_.segments[i].id, current_traj_.segments[i].duration_s);
    // }
    // fprintf(stderr, "\n--------\n");

    printf("Sending trajectory to plan channel\n");
    pipe_server_write(plan_ch_, &out, sizeof(trajectory_t));
    return true;
}

void LocalAStar::visualizePath(const Point3fVector &target_points)
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

    generateAndSendPath(render_ch_, waypoints, PATH_VIS_LINE, "LocalAStar Waypoints");
    nanosleep_for(1e7);

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

    generateAndSendPath(render_ch_, traj, PATH_VIS_TRAJECTORY, "LocalAStar Smoothed");
}

bool LocalAStar::runSmoother(const Point3fVector &target_points, mav_trajectory_generation::Trajectory &trajectory)
{
    mav_msgs::EigenTrajectoryPointVector waypoints_for_smoother;
    convertPointsToSmootherFormat(target_points, waypoints_for_smoother);
    int num_segments = loco_num_segments < target_points.size() ? loco_num_segments : target_points.size();
    loco_smoother_.setNumSegments(num_segments);

    return loco_smoother_.getTrajectoryBetweenWaypoints(waypoints_for_smoother, &trajectory);
}

void LocalAStar::setup()
{
    setupSmootherFromConfig(loco_smoother_, map_->getEsdfMapPtr().get());
}

void LocalAStar::start()
{
    // Ignore request to start if already running, otherwise check if previous thread needs cleanup
    if (running_)
        return;
    else if (planning_thread_.joinable())
        planning_thread_.join();

    cur_waypoint_ = 1;
    running_ = true;
    planning_thread_ = std::thread(&LocalAStar::plannerThread, this);
}

void LocalAStar::stop()
{
    if (!running_)
        return;

    running_ = false;

    if (planning_thread_.joinable())
        planning_thread_.join();
}

void LocalAStar::computeSplitPointDetails(Point3f &start_pose, int &split_id, double &split_time)
{
    poly_segment_t *last_segment = &current_traj_.segments[current_traj_.n_segments - 1];
    float x = eval_poly_at_t(last_segment->n_coef, last_segment->cx, last_segment->duration_s);
    float y = eval_poly_at_t(last_segment->n_coef, last_segment->cy, last_segment->duration_s);
    float z = eval_poly_at_t(last_segment->n_coef, last_segment->cz, last_segment->duration_s);

    // Default values will be the end of the segment
    start_pose << x, y, z;
    split_id = last_segment->id;
    split_time = last_segment->duration_s;

    // Lock and copy the segment variables
    pthread_mutex_lock(&segment_mutex);
    double cur_segment_t = cur_segment_t_;
    int cur_segment_id = cur_segment_id_;
    pthread_mutex_unlock(&segment_mutex);

    int segment_idx = get_segment_idx(&current_traj_, cur_segment_id);

    if (segment_idx < 0)
    {
        fprintf(stderr, "ERROR: Segment id %d does not exist in current trajectory. Planning from end of trajectory\n", cur_segment_id);
        return;
    }

    // Now we need to calculate the time point to start at. We do this by moving PLAN_AHEAD_TIME seconds
    // along the polynomial, starting at the most recently evaluate time point
    double t = current_traj_.segments[segment_idx].duration_s - cur_segment_t;

    // If the split happens on this segment then find the start point and split time
    if (t >= PLAN_AHEAD_TIME)
    {
        split_time = cur_segment_t + PLAN_AHEAD_TIME;
        split_id = cur_segment_id;

        poly_segment_t *s = &current_traj_.segments[segment_idx];
        float x = eval_poly_at_t(s->n_coef, s->cx, split_time);
        float y = eval_poly_at_t(s->n_coef, s->cy, split_time);
        float z = eval_poly_at_t(s->n_coef, s->cz, split_time);

        start_pose << x, y, z;
        return;
    }

    // Otherwise step through the following segments to find where the split point occurs
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
            return;
        }
    }

    // We will only reach here if PLAN_AHEAD_TIME occurs past end of the current trajectory
    // so the default values are correct
}

bool LocalAStar::getInitialPlan()
{
    // Get current pose
    rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
    map_->getRobotPose(tf_body_wrt_fixed, rc_nanos_monotonic_time());

    Point3f cur_pose(tf_body_wrt_fixed.d[0][3], tf_body_wrt_fixed.d[1][3], tf_body_wrt_fixed.d[2][3]);

    Point3fVector target_points;
    mav_trajectory_generation::Trajectory trajectory;

    if (!runPlanner(cur_pose, target_points, trajectory))
        return false;

    if (!sendTrajectory(trajectory, -1, -1, true))
    {
        fprintf(stderr, "Local Planner failed to send initial trajectory\n");
        return false;
    }

    visualizePath(target_points);

    return true;
}

Point3f LocalAStar::computeGoalPoint(Point3f &start_pos)
{
    // Find closest line segment
    float min_dist = __FLT_MAX__;
    for (size_t i = cur_waypoint_; i < waypoints_.size() - 1; i++)
    {
        Point3f p1 = waypoints_[i];
        Point3f p2 = waypoints_[i + 1];
        Eigen::ParametrizedLine<float, 3> line = Eigen::ParametrizedLine<float, 3>::Through(p1, p2);

        // Check if the projection of start pose is within the bounds of the line
        if (isBetween(p1, p2, line.projection(start_pos)))
        {
            // Save the end of the line we are closest to as our target
            float d = line.squaredDistance(start_pos);
            if (d < min_dist)
            {
                min_dist = d;
                cur_waypoint_ = i + 1;
            }
        }
    }

    // Add additional waypoints for a longer planning distance
    float total_dist = 0;
    float prev_dist = 0;
    Point3f prev_wp = start_pos;

    for (size_t i = cur_waypoint_; i < waypoints_.size(); i++)
    {
        prev_dist = total_dist;
        total_dist += (waypoints_[i] - prev_wp).norm();

        if (total_dist > MAX_PLAN_DISTANCE)
        {
            float remaining_dist = MAX_PLAN_DISTANCE - prev_dist;
            return prev_wp + (waypoints_[i] - prev_wp).normalized() * remaining_dist;
        }

        prev_wp = waypoints_[i];
    }

    return waypoints_.back();
}

bool LocalAStar::runPlanner(Point3f start_pos, Point3fVector &target_points, mav_trajectory_generation::Trajectory &trajectory)
{
    const voxblox::Layer<voxblox::EsdfVoxel> *layer = map_->getEsdfMapPtr()->getEsdfLayerConstPtr();

    // Get goal point
    Point3f goal_pos = computeGoalPoint(start_pos);

    // Calculate start and goal idx
    voxblox::GlobalIndex start_idx = voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(start_pos, 1.0f / voxel_size);
    voxblox::GlobalIndex goal_idx = voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(goal_pos, 1.0f / voxel_size);

    std::priority_queue<Node *, std::vector<Node *>, Compare> open;
    voxblox::LongIndexHashMapType<Node *>::type node_lookup;
    Node *start_node = new Node(0, 0, 0, start_idx, nullptr);
    Node *cur_node;
    Node *nbr_node;
    Node *best_node;
    float best_h = std::numeric_limits<float>::max();

    // Insert node into our lookup map
    node_lookup.insert(std::make_pair(start_node->esdf_idx, start_node));

    // int64_t s = rc_nanos_monotonic_time();
    int iteration_count = 0;

    open.push(start_node);

    while (!open.empty())
    {
        iteration_count++;

        cur_node = open.top();
        open.pop();

        // Need to save best heuristic node incase we end early
        if (cur_node != start_node && cur_node->heuristic_cost < best_h)
        {
            best_node = cur_node;
            best_h = cur_node->heuristic_cost;
        }

        // "Insert" node into closed set (less overhead using a variable stored in node)
        cur_node->closed = true;

        // Check if we have reached end condition (reached global goal or max distance away from start point)
        if (cur_node->esdf_idx == goal_idx)
        {
            best_node = cur_node;
            break;
        }
        else if (cur_node->travel_cost >= MAX_TRAVEL_COST)
        {
            break;
        }

        // Get the global indices of neighbors.
        voxblox::Neighborhood<voxblox::Connectivity::kTwentySix>::IndexMatrix neighbor_indices;
        voxblox::Neighborhood<voxblox::Connectivity::kTwentySix>::getFromGlobalIndex(cur_node->esdf_idx, &neighbor_indices);

        // Go through the neighbors and add them to the open set
        for (unsigned int idx = 0; idx < neighbor_indices.cols(); ++idx)
        {

            const voxblox::GlobalIndex &nbr_global_idx = neighbor_indices.col(idx);
            const voxblox::EsdfVoxel *esdf_voxel = layer->getVoxelPtrByGlobalIndex(nbr_global_idx);


            // Skip if no voxel, or voxel has been seen and is in collision
            if (esdf_voxel == nullptr || (esdf_voxel->observed && esdf_voxel->distance < robot_radius))
            {
                continue;
            }

            float change_dir_cost = cur_node->idx_in_parent == idx ? 0 : 1;
            float travel_cost = cur_node->travel_cost + voxblox::Neighborhood<voxblox::Connectivity::kTwentySix>::kDistances[idx];
            travel_cost += change_dir_cost;

            // If nbr is in node_lookup fetch the pointer, otherwise create a new node
            if (node_lookup.count(nbr_global_idx) > 0)
            {
                nbr_node = node_lookup[nbr_global_idx];

                // Skip if node is in closed set
                if (nbr_node->closed)
                {
                    continue;
                }

                // Check if the path through this node is shorter
                if (travel_cost < nbr_node->travel_cost)
                {
                    nbr_node->travel_cost = travel_cost;
                    nbr_node->total_cost = travel_cost + nbr_node->heuristic_cost;
                    nbr_node->parent = cur_node;
                    nbr_node->idx_in_parent = idx;

                    // Hack to reorder heap
                    // TODO: Check if remove then insert is faster
                    std::make_heap(const_cast<Node **>(&open.top()),
                                   const_cast<Node **>(&open.top()) + open.size(),
                                   Compare());
                }
            }
            else
            {
                float heuristic_cost = computeHeuristic(nbr_global_idx, goal_idx, esdf_voxel->distance);
                nbr_node = new Node(travel_cost + heuristic_cost, travel_cost, heuristic_cost, nbr_global_idx, cur_node);
                nbr_node->idx_in_parent = idx;

                // Insert node into our lookup map
                node_lookup.insert(std::make_pair(nbr_node->esdf_idx, nbr_node));

                // Add to queue
                open.push(nbr_node);
            }
        }
    }

    // fprintf(stderr, "A* took = %6.2fms for %d iterations\n", (double)(rc_nanos_monotonic_time() - s) / 1e6, iteration_count);

    cur_node = best_node;
    std::vector<Node *> path;

    // Get the path out and reverse it
    while (cur_node != nullptr)
    {
        path.push_back(cur_node);
        cur_node = cur_node->parent;
    }
    std::reverse(path.begin(), path.end());

    pruneAStarPath(path);

    // Convert path to correct format for smoother
    target_points.push_back(start_pos);
    for (int i = 1; i < path.size(); i++)
    {
        Point3f pos = voxblox::getCenterPointFromGridIndex(path[i]->esdf_idx, voxel_size);
        target_points.push_back(pos);
    }

    // Don't want centre of voxels for our start and end points
    target_points[0] = start_pos;
    target_points.back() = goal_pos;

    // Cleanup nodes
    for (auto it = node_lookup.begin(); it != node_lookup.end(); it++)
        delete it->second;

    // s = rc_nanos_monotonic_time();
    if (!runSmoother(target_points, trajectory))
    {
        fprintf(stderr, "Smoother failed in local planner. Skipping..\n");
        return false;
    }
    // fprintf(stderr, "Smoother took = %6.2fms\n", (double)(rc_nanos_monotonic_time() - s) / 1e6);

    return true;
}

float LocalAStar::computeHeuristic(voxblox::GlobalIndex cur_idx, voxblox::GlobalIndex goal_idx, float obs_dist)
{
    int64_t dx = abs(cur_idx(0) - goal_idx(0));
    int64_t dy = abs(cur_idx(1) - goal_idx(1));
    int64_t dz = abs(cur_idx(2) - goal_idx(2));

    int64_t sorted[3] = {dx, dy, dz};
    int n = sizeof(sorted) / sizeof(int64_t);

    std::sort(sorted, sorted + n);

    float dist_to_goal_heuristic = sqrt(3) * sorted[0] + sqrt(2) * (sorted[1] - sorted[0]) + (sorted[2] - sorted[1]);
    float obstacle_heuristic = -obs_dist;

    return dist_to_goal_heuristic + obstacle_heuristic;
}

bool LocalAStar::isBetween(Point3f a, Point3f b, Point3f c)
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

void LocalAStar::pruneAStarPath(std::vector<Node *> &path)
{
    // Nothing to prune
    if (path.size() <= 2)
    {
        return;
    }

    std::vector<Node *> new_path;
    new_path.reserve(path.size());
    new_path.push_back(path[0]);

    size_t p = 0;
    size_t p_next = 1;

    while (p < path.size() - 1)
    {
        int cur_idx = path[p_next]->idx_in_parent;

        while (p_next + 1 < path.size() && cur_idx == path[p_next + 1]->idx_in_parent)
        {
            p_next++;
        }

        new_path.push_back(path[p_next]);
        p = p_next;
        p_next++;
    }

    path.swap(new_path);
}

void LocalAStar::plannerThread()
{
    int64_t next_time = 0;
    map_->updateEsdf(true);

    if (!getInitialPlan())
    {
        fprintf(stderr, "Failed to find initial plan. Exiting..\n");
        running_ = false;
        return;
    }

    while (running_)
    {
        loop_sleep(LOCAL_PLANNER_RATE, &next_time);
        uint64_t s = rc_nanos_monotonic_time();
        
        // Get current pose to check if we have reached goal
        rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
        map_->getRobotPose(tf_body_wrt_fixed, rc_nanos_monotonic_time());

        Point3f cur_pose(tf_body_wrt_fixed.d[0][3], tf_body_wrt_fixed.d[1][3], tf_body_wrt_fixed.d[2][3]);

        if ((cur_pose - waypoints_.back()).norm() <= REACHED_DISTANCE)
        {
            printf("Reached goal. Stopping local planner..");
            break;
        }

        map_->updateEsdf(true);

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

        fprintf(stderr, "Planner took %f\n", (double)(rc_nanos_monotonic_time() - s)/1e6);
    }

    running_ = false;
    return;
}

void LocalAStar::setPlan(const Point3fVector &waypoints)
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

LocalAStar::~LocalAStar()
{
    stop();
}