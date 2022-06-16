#include "rrt_connect.h"
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <voxblox/core/common.h>
#include <random>
#include <float.h>
#include <unistd.h>

static uint64_t rc_nanos_monotonic_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000) + ts.tv_nsec;
}

RRTConnect::RRTConnect(std::shared_ptr<voxblox::EsdfMap> esdf_map_, int vis_channel)
    : esdf_map_(esdf_map_),
      root_(nullptr),
      node_counter_(-2), // Set to -2 so that -2 is start and -1 is goal
      vis_channel_(vis_channel)
{
    // Set the random seed
    srand(time(nullptr));

    computeMapBounds();

    setupSmoother();
}

bool RRTConnect::computeMapBounds()
{
    if (esdf_map_ == nullptr)
    {
        printf("ERROR: esdf map is nullptr, cannot compute map bounds.\n");
        return false;
    }

    voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(), &lower_bound_, &upper_bound_);

    printf("Map Bounds:\n");
    printf("Lower-> x:%6.2f, y:%6.2f, z:%6.2f\n", lower_bound_.x(), lower_bound_.y(), lower_bound_.z());
    printf("Upper-> x:%6.2f, y:%6.2f, z:%6.2f\n", upper_bound_.x(), upper_bound_.y(), upper_bound_.z());

    ind_lower_[0] = (double)lower_bound_.x() / ((double)voxels_per_side * (double)voxel_size);
    ind_lower_[1] = (double)lower_bound_.y() / ((double)voxels_per_side * (double)voxel_size);
    ind_lower_[2] = (double)lower_bound_.z() / ((double)voxels_per_side * (double)voxel_size);

    double ind_upper[3] = {
        (double)upper_bound_.x() / ((double)voxels_per_side * (double)voxel_size),
        (double)upper_bound_.y() / ((double)voxels_per_side * (double)voxel_size),
        (double)upper_bound_.z() / ((double)voxels_per_side * (double)voxel_size)};

    d_x_ = std::abs(ind_lower_[0] - ind_upper[0]);
    d_y_ = std::abs(ind_lower_[1] - ind_upper[1]);
    d_z_ = std::abs(ind_lower_[2] - ind_upper[2]);

    // nodes max size will then be dx + (dy * dx) + (dz * dy * dz)
    nodes_.resize(d_x_ + (d_y_ * d_x_) + (d_z_ * d_y_ * d_x_));
    return true;
}

void RRTConnect::setupSmoother()
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
    loco_smoother_.setParameters(loco_params, poly_params, physical_constraints, loco_verbose);
    loco_smoother_.setInCollisionCallback(std::bind(&RRTConnect::detectCollision, this, std::placeholders::_1));
    loco_smoother_.setDistanceAndGradientFunction(std::bind(&RRTConnect::getMapDistanceAndGradient, this, std::placeholders::_1, std::placeholders::_2));

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
    loco_smoother_.loco_config.epsilon = 0.5;
    loco_smoother_.loco_config.robot_radius = robot_radius;
    loco_smoother_.loco_config.w_d = loco_smoothness_cost_weight;
    loco_smoother_.loco_config.w_c = loco_collision_cost_weight;
    loco_smoother_.loco_config.w_w = loco_waypoint_cost_weight;
    loco_smoother_.loco_config.min_collision_sampling_dt = loco_min_collision_sampling_dist;
    loco_smoother_.loco_config.map_resolution = voxel_size;
    loco_smoother_.loco_config.verbose = loco_verbose;
}

bool RRTConnect::detectCollisionEdge(const Eigen::Vector3d &start, const Eigen::Vector3d &end, bool is_extend = false)
{
    timer.start("Edge Collision Check");

    double dist;

    if (is_extend)
        dist = rrt_min_distance;
    else
        dist = distance(start, end);

    int num_of_steps = floor(dist / robot_radius);

    if (detectCollision(end))
    {
        timer.stop("Edge Collision Check");
        return true;
    }

    // A direction vector with length of robot radius
    Eigen::Vector3d dir_vec = ((end - start) / dist);
    Eigen::Vector3d pos = start + dir_vec * robot_radius;

    for (int i = 0; i < num_of_steps; i++)
    {
        if (detectCollision(pos))
        {
            timer.stop("Edge Collision Check");
            return true;
        }

        pos += dir_vec * robot_radius;
    }
    timer.stop("Edge Collision Check");
    return false;
}

bool RRTConnect::detectCollision(const Eigen::Vector3d &pos)
{
    return getMapDistance(pos) <= robot_radius * 1.5;
}

double RRTConnect::getMapDistance(const Eigen::Vector3d &position)
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

double RRTConnect::getMapDistanceAndGradient(const Eigen::Vector3d &position, Eigen::Vector3d *gradient)
{
    double distance = 0.0;
    if (!(esdf_map_->getDistanceAndGradientAtPosition(position, false, &distance, gradient)))
    {
        return 0.0;
    }
    return distance;
}

Node *RRTConnect::createNewNode(const Eigen::Vector3d &position, Node *parent)
{
    Node *new_node = new Node{.position = position, .parent = parent, .children = std::vector<Node *>(), .id = node_counter_};
    node_counter_++;
    return new_node;
}

Node *RRTConnect::createRandomNode()
{
    double x = ((upper_bound_.x() - lower_bound_.x()) * (double)rand() / (double)RAND_MAX) + lower_bound_.x();
    double y = ((upper_bound_.y() - lower_bound_.y()) * (double)rand() / (double)RAND_MAX) + lower_bound_.y();
    double z = ((upper_bound_.z() - lower_bound_.z()) * (double)rand() / (double)RAND_MAX) + lower_bound_.z();

    Eigen::Vector3d rand_pt(roundf(x * 100) / 100, roundf(y * 100) / 100, roundf(z * 100) / 100);

    return createNewNode(rand_pt, nullptr);
}

std::pair<Node *, double> RRTConnect::findNearest(const Eigen::Vector3d &point)
{
    double min_dist = DBL_MAX;
    Node *closest = nullptr;

    int actual_index = flatIndexfromPoint(point);

    if (!nodes_[actual_index].empty())
    {

        for (size_t i = 0; i < nodes_[actual_index].size(); i++)
        {
            // TODO: Is this needed?
            if (nodes_[actual_index][i] == nullptr)
                continue;

            double dist = distance(point, nodes_[actual_index][i]->position);

            if (dist < min_dist)
            {
                min_dist = dist;
                closest = nodes_[actual_index][i];
            }
        }
    }

    if (closest == nullptr)
    {
        actual_index = binSelect(actual_index);
        if (!nodes_[actual_index].empty())
        {
            for (size_t i = 0; i < nodes_[actual_index].size(); i++)
            {
                double dist = distance(point, nodes_[actual_index][i]->position);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest = nodes_[actual_index][i];
                }
            }
        }
    }

    return std::make_pair(closest, min_dist);
}

int RRTConnect::binSelect(int index)
{
    if (!nodes_[index].empty())
        return index;

    // check left/right, forward/back, up/down by 1. if those all fail, keep looking around it
    int x_offset = 1;
    int y_offset = d_x_;
    int z_offset = d_y_ * d_x_;

    // independent checks
    bool l_x, l_y, l_z, r_x, r_y, r_z;
    l_x = l_y = l_z = r_x = r_y = r_z = true;

    while (l_x || l_y || l_z || r_x || r_y || r_z)
    {

        if (l_x && index - x_offset < 0)
            l_x = false;
        if (r_x && index + x_offset >= nodes_.size())
            r_x = false;

        if (l_y && index - y_offset < 0)
            l_y = false;
        if (r_y && index + y_offset >= nodes_.size())
            r_y = false;

        if (l_z && index - z_offset < 0)
            l_z = false;
        if (r_z && index + z_offset >= nodes_.size())
            r_z = false;

        if (r_x && !nodes_.at(index + x_offset).empty())
            return index + x_offset;
        if (l_x && !nodes_.at(index - x_offset).empty())
            return index - x_offset;

        if (r_y && !nodes_.at(index + y_offset).empty())
            return index + y_offset;
        if (l_y && !nodes_.at(index - y_offset).empty())
            return index - y_offset;

        if (r_z && !nodes_.at(index + z_offset).empty())
            return index + z_offset;
        if (l_z && !nodes_.at(index - z_offset).empty())
            return index - z_offset;

        x_offset += 1;
        y_offset += d_x_;
        z_offset += d_y_ * d_x_;
    }
    // default. if we fail, return where the root is
    return 0;
}

int RRTConnect::flatIndexfromPoint(const Eigen::Vector3d &point)
{
    voxblox::BlockIndex bi = esdf_map_->getEsdfLayerPtr()->computeBlockIndexFromCoordinates(point.cast<float>());
    return (bi.x() - ind_lower_[0]) + (bi.y() - ind_lower_[1] * d_x_) + ((bi.z() - ind_lower_[2]) * d_y_ * d_x_);
}

void RRTConnect::add(Node *q_nearest, Node *q_new)
{
    q_new->parent = q_nearest;
    q_nearest->children.push_back(q_new);

    int actual_index = flatIndexfromPoint(q_new->position);
    nodes_[actual_index].push_back(q_new);
}

void RRTConnect::deleteNodes(Node *root)
{
    for (size_t i = 0; i < root->children.size(); i++)
    {
        deleteNodes(root->children[i]);
    }
    delete root;
    root = nullptr;
}

void RRTConnect::cleanupPruning()
{
    for (const Node* node : pruning_nodes_)
    {
        delete node;
    }

    pruning_nodes_.clear();
}

void RRTConnect::nodesToEigen(mav_msgs::EigenTrajectoryPointVector &eigen_path)
{
    // sanity checks
    if (rrt_path_.empty())
    {
        fprintf(stderr, "Waypoint path is empty!\n");
        return;
    }

    for (size_t i = 0; i < rrt_path_.size(); i++)
    {
        mav_msgs::EigenTrajectoryPoint curr_pt;
        curr_pt.position_W = rrt_path_[i]->position;
        eigen_path.push_back(curr_pt);
    }
}

bool RRTConnect::locoSmooth(const mav_msgs::EigenTrajectoryPointVector &waypoints, mav_msgs::EigenTrajectoryPointVector &smoothed_path, mav_trajectory_generation::Trajectory &final_trajectory)
{
    bool got = loco_smoother_.getTrajectoryBetweenWaypoints(waypoints, &final_trajectory);

    bool success = false;
    if (got)
    {
        mav_msgs::EigenTrajectoryPoint::Vector states;
        double sampling_interval = 0.05;
        success = mav_trajectory_generation::sampleWholeTrajectory(final_trajectory, sampling_interval, &smoothed_path);
    }

    return got && success;
}

bool RRTConnect::runSmoother(mav_trajectory_generation::Trajectory &trajectory)
{
    printf("Running Smoother\n");
    uint64_t start_time = rc_nanos_monotonic_time();

    mav_msgs::EigenTrajectoryPointVector waypoints;

    nodesToEigen(waypoints);

    bool loco_success = locoSmooth(waypoints, smoothed_path_, trajectory);

    if (loco_success)
    {
        printf("Smoother finished succesfully in %6.2fms\n", (rc_nanos_monotonic_time() - start_time) / 1000000.0);
        return true;
    }
    else
    {
        printf("Smoother failed and took %6.2fms\n", (rc_nanos_monotonic_time() - start_time) / 1000000.0);
        return false;
    }
}

void RRTConnect::pruneRRTPath()
{
    int prune_count = 0;

    // Level 1 pruning
    int index_a;
    int index_b;

    float shift_a;
    float shift_b;

    Eigen::Vector3d candidate_a;
    Eigen::Vector3d candidate_b;

    // Generate between 0.2 and 0.8 otherwise we will have new nodes too close to the start nodes
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.2, 0.8);

    for (int i = 0; i < rrt_prune_iterations; i++)
    {
        // Pick two nodes at random (ensure index b is always greater than index a)
        index_a = rand() % (rrt_path_.size() - 2);
        index_b = rand() % (rrt_path_.size() - 2 - index_a) + index_a + 1;

        // Generate a random shift along each nodes next edge
        shift_a = distribution(generator);
        shift_b = distribution(generator);

        // Calculate new points along the edge using the shift
        candidate_a = (1 - shift_a) * rrt_path_[index_a]->position + shift_a * rrt_path_[index_a + 1]->position;
        candidate_b = (1 - shift_b) * rrt_path_[index_b]->position + shift_b * rrt_path_[index_b + 1]->position;

        // The path between the two new positions is a shortcut. If its collision free, 
        // add it into the path and remove the nodes that it skips
        if (!detectCollisionEdge(candidate_a, candidate_b))
        {
            // Remove all intermediate nodes in rrt_path_ between A and B
            while (index_b > index_a)
            {
                rrt_path_.erase(rrt_path_.begin() + index_b);
                index_b--;
            }

            // Temp nodes only for inserting into the path
            Node *new_a = createNewNode(candidate_a, nullptr);
            Node *new_b = createNewNode(candidate_b, nullptr);

            // Save the nodes so we can clean them up after
            pruning_nodes_.push_back(new_a);
            pruning_nodes_.push_back(new_b);

            rrt_path_.insert(rrt_path_.begin() + index_a + 1, new_b);
            rrt_path_.insert(rrt_path_.begin() + index_a + 1, new_a);

            prune_count++;
        }
    }

    // Level 2 pruning
    Node *start;
    Node *end;

    for (size_t i = 0; i < rrt_path_.size() - 2; i++)
    {
        start = rrt_path_[i];

        for (size_t j = rrt_path_.size() - 1; j > i; j--)
        {
            end = rrt_path_[j];

            if (!detectCollisionEdge(start->position, end->position))
            {
                rrt_path_.erase(rrt_path_.begin() + i + 1, rrt_path_.begin() + j);
                prune_count++;
                break;
            }
        }
    }

    printf("Pruned %d time\n", prune_count);
}

void RRTConnect::visualizePaths()
{
    // Get RRT Tree points
    std::vector<point_xyz> rrt_pc;
    for (const Node* rrt_node : rrt_path_)
    {
        point_xyz pt;
        pt.x = rrt_node->position.x();
        pt.y = rrt_node->position.y();
        pt.z = rrt_node->position.z();
        rrt_pc.push_back(pt);
    }

    // Get Smoothed path points
    std::vector<point_xyz> smooth_path_pc;
    for (const auto &trajectory_point : smoothed_path_)
    {
        point_xyz pt;
        pt.x = trajectory_point.position_W.x();
        pt.y = trajectory_point.position_W.y();
        pt.z = trajectory_point.position_W.z();
        smooth_path_pc.push_back(pt);
    }

    // *NOTE* using ts as format to specify type of path for voxl-portal
    // 0 - raw waypoints
    // 2 - loco smoothed path
    // 6 - rrt star tree as building (not sent here)

    point_cloud_metadata_t waypoints_meta;
    waypoints_meta.magic_number = POINT_CLOUD_MAGIC_NUMBER;
    waypoints_meta.n_points = rrt_pc.size();
    waypoints_meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZ;
    waypoints_meta.timestamp_ns = 0;

    if (waypoints_meta.n_points != 0)
        pipe_server_write_point_cloud(vis_channel_, waypoints_meta, rrt_pc.data());

    // Delay before sending another path
    usleep(100000);

    waypoints_meta.n_points = smooth_path_pc.size();
    waypoints_meta.timestamp_ns = 1;
    if (waypoints_meta.n_points != 0)
        pipe_server_write_point_cloud(vis_channel_, waypoints_meta, smooth_path_pc.data());
}

void RRTConnect::visualizeMap()
{
    std::vector<point_xyz_i> pc;

    voxblox::BlockIndexList block_list;
    voxblox::Layer<voxblox::EsdfVoxel> *layer = esdf_map_->getEsdfLayerPtr();
    layer->getAllAllocatedBlocks(&block_list);
    size_t vps = layer->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    for (const voxblox::BlockIndex &block_index : block_list)
    {
        const voxblox::Block<voxblox::EsdfVoxel> &block = layer->getBlockByIndex(block_index);

        for (size_t i = 0u; i < num_voxels_per_block; ++i)
        {
            const voxblox::EsdfVoxel &voxel = block.getVoxelByLinearIndex(i);
            Eigen::Matrix<float, 3, 1> point = block.computeCoordinatesFromLinearIndex(i);

            if (!voxel.observed)
                continue;

            point_xyz_i pt;
            pt.x = point.x();
            pt.y = point.y();
            pt.z = point.z();
            pt.intensity = voxel.distance;
            pc.push_back(pt);
        }
    }

    point_cloud_metadata_t waypoints_meta;
    waypoints_meta.magic_number = POINT_CLOUD_MAGIC_NUMBER;
    waypoints_meta.n_points = pc.size();
    waypoints_meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZC;
    waypoints_meta.timestamp_ns = 3;

    if (waypoints_meta.n_points != 0)
        pipe_server_write_point_cloud(vis_channel_, waypoints_meta, pc.data());
}

bool RRTConnect::createPlan(const Eigen::Vector3d &startPos, const Eigen::Vector3d &endPos, mav_trajectory_generation::Trajectory &trajectory)
{
    timer.start("RRT Planner");
    printf("Start = (%f, %f, %f), End = (%f, %f, %f)\n", startPos.x(), startPos.y(), startPos.z(), endPos.x(), endPos.y(), endPos.z());

    // Setup start and end nodes
    Node *qStart = createNewNode(startPos, nullptr);
    Node *qGoal = createNewNode(endPos, nullptr);
    root_ = qStart;

    // Insert start into search structure
    int start_node_index = flatIndexfromPoint(qStart->position);
    nodes_[start_node_index].push_back(qStart);

    // Ensure that the start and end point are collision free
    if (detectCollision(qStart->position))
    {
        printf("ERROR: Start point is in collision\n");
        return false;
    }
    else if (detectCollision(qGoal->position))
    {
        printf("ERROR: End point is in collision\n");
        return false;
    }

    // Check if an immediate path from start to end exists
    if (!detectCollisionEdge(qStart->position, qGoal->position))
    {
        printf("Immediate collision free path found!\n");
        rrt_path_.push_back(qStart);
        rrt_path_.push_back(qGoal);

        return runSmoother(trajectory);
    }

    uint64_t start_time = rc_nanos_monotonic_time();
    int attempts = 0;
    Node *q_rand = nullptr;
    Node *q_connect = nullptr;
    Node *q_near = nullptr;
    double dist = 0;
    bool collision_found;

    while (rrt_max_runtime_nanoseconds == -1 || start_time + rrt_max_runtime_nanoseconds > rc_nanos_monotonic_time())
    {
        // Get random node or use goal
        if (attempts % 10 == 0)
            q_rand = qGoal;
        else
            q_rand = createRandomNode();

        if (q_rand)
        {
            collision_found = false;

            // Find nearest node in tree
            std::tie(q_near, dist) = findNearest(q_rand->position);

            Eigen::Vector3d dir_vec = ((q_rand->position - q_near->position) / dist);

            // Continually step towards q_rand by rrt_min_distance and add a node if its collision free
            // Optimization: Use squared distances to avoid having to do the square root which is more expensive
            while ((q_rand->position - q_near->position).squaredNorm() > pow(rrt_min_distance, 2))
            {
                q_connect = createNewNode(q_near->position + rrt_min_distance * dir_vec, nullptr);

                if (!detectCollisionEdge(q_near->position, q_connect->position, true))
                {
                    add(q_near, q_connect);
                    q_near = q_connect;
                }
                else
                {
                    // The current node is in collision so it is not added to the tree and needs to be deleted
                    delete q_connect;
                    q_connect = nullptr;
                    collision_found = true;
                    break;
                }
            }

            // Add q_rand only if the while loop succesfully finished and its collision free
            if (!collision_found && !detectCollisionEdge(q_near->position, q_rand->position))
            {
                add(q_near, q_rand);

                // If we ran goal biasing then the end point would be the goal, so exit
                if (q_rand == qGoal)
                    break;
            }
            else if (q_rand != qGoal)
            {
                // Delete q_rand if we couldnt add to graph and it wasnt the goal node
                delete q_rand;
                q_rand = nullptr;
            }

            // Check if we can reach goal
            std::tie(q_near, dist) = findNearest(qGoal->position);

            if (dist <= rrt_goal_threshold)
            {
                add(q_near, qGoal);
                break;
            }
        }

        attempts++;
    }

    if (qGoal->parent != nullptr)
    {
        printf("RRT solution found in %6.2fms and took %d attempts\n", (rc_nanos_monotonic_time() - start_time) / 1000000.0, attempts);
    }
    else
    {
        printf("RRT solution not found. Planning exceeded %6.2fms and took %d attempts\n", rrt_max_runtime_nanoseconds / 1000000.0, attempts);
        return false;
    }

    // Get the path from qGoal to qStart
    Node *cur = qGoal;

    while (cur->parent != nullptr)
    {
        rrt_path_.push_back(cur);
        cur = cur->parent;
    }

    // Add the last node which should be qStart
    rrt_path_.push_back(cur);

    // Reverse path since we traveresed tree from leaf to root but we want root to leaf
    std::reverse(rrt_path_.begin(), rrt_path_.end());

    timer.start("Pruning");
    pruneRRTPath();
    timer.stop("Pruning");
    
    timer.stop("RRT Planner");

    // Run smoother
    timer.start("Smoother");
    bool smoother_success = runSmoother(trajectory);
    timer.stop("Smoother");

    timer.printAllTimers();

    visualizePaths();

    if (rrt_send_map) 
        visualizeMap();

    return smoother_success;
}

RRTConnect::~RRTConnect()
{
    deleteNodes(root_);
    cleanupPruning();
}