#include "rrt_connect.h"
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <voxblox/core/common.h>
#include <random>
#include <float.h>
#include <unistd.h>
#include "path_vis.h"

static uint64_t rc_nanos_monotonic_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000) + ts.tv_nsec;
}

RRTConnect::RRTConnect(std::shared_ptr<voxblox::EsdfMap> esdf_map, int vis_channel)
    : esdf_map_(esdf_map),
      root_(nullptr),
      node_counter_(-2), // Set to -2 so that -2 is start and -1 is goal
      vis_channel_(vis_channel)
{
    // Set the random seed
    srand(time(nullptr));
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
    printf("Lower-> x:%6.2f, y:%6.2f, z:%6.2f\n", (double)lower_bound_.x(), (double)lower_bound_.y(), (double)lower_bound_.z());
    printf("Upper-> x:%6.2f, y:%6.2f, z:%6.2f\n", (double)upper_bound_.x(), (double)upper_bound_.y(), (double)upper_bound_.z());

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

Node *RRTConnect::createNewNode(const Point3f &position, Node *parent)
{
    Node *new_node = new Node{.position = position, .parent = parent, .children = std::vector<Node *>(), .id = node_counter_};
    node_counter_++;

    if (parent != nullptr)
        parent->children.push_back(new_node);

    return new_node;
}

Node *RRTConnect::createRandomNode()
{
    float x = ((upper_bound_.x() - lower_bound_.x()) * (float)rand() / (float)RAND_MAX) + lower_bound_.x();
    float y = ((upper_bound_.y() - lower_bound_.y()) * (float)rand() / (float)RAND_MAX) + lower_bound_.y();
    float z = ((upper_bound_.z() - lower_bound_.z()) * (float)rand() / (float)RAND_MAX) + lower_bound_.z();

    Point3f rand_pt(roundf(x * 100) / 100, roundf(y * 100) / 100, roundf(z * 100) / 100);

    return createNewNode(rand_pt, nullptr);
}

std::pair<Node *, double> RRTConnect::findNearest(const Point3f &point)
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

int RRTConnect::flatIndexfromPoint(const Point3f &point)
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

void RRTConnect::cleanupTree()
{
    if (root_)
        deleteNodes(root_);

    // Clear out the search structure
    for (std::vector<Node *> &vec : nodes_)
    {
        vec.clear();
    }

    nodes_.clear();
    node_counter_ = -2;
}

void RRTConnect::cleanupPruning()
{
    for (Node *node : pruning_nodes_)
    {
        delete node;
    }

    pruning_nodes_.clear();
}

void RRTConnect::pruneRRTPath(std::vector<Node *> &rrt_path)
{
    int prune_count_l1 = 0;
    int prune_count_l2 = 0;

    // Level 1 pruning
    int index_a;
    int index_b;

    float shift_a;
    float shift_b;

    Point3f candidate_a;
    Point3f candidate_b;

    // Generate between 0.2 and 0.8 otherwise we will have new nodes too close to the start nodes
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.2, 0.8);

    fprintf(stderr, "%ld waypoints before pruning\n", rrt_path.size());

    for (int i = 0; i < rrt_prune_iterations; i++)
    {
        // Pick two nodes at random (ensure index b is always greater than index a)
        index_a = rand() % (rrt_path.size() - 2);
        index_b = rand() % (rrt_path.size() - 2 - index_a) + index_a + 1;

        // Generate a random shift along each nodes next edge
        shift_a = distribution(generator);
        shift_b = distribution(generator);

        // Calculate new points along the edge using the shift
        candidate_a = (1 - shift_a) * rrt_path[index_a]->position + shift_a * rrt_path[index_a + 1]->position;
        candidate_b = (1 - shift_b) * rrt_path[index_b]->position + shift_b * rrt_path[index_b + 1]->position;

        // The path between the two new positions is a shortcut. If its collision free,
        // add it into the path and remove the nodes that it skips
        if (!detectCollisionEdge(esdf_map_.get(), candidate_a, candidate_b))
        {
            // Remove all intermediate nodes in rrt_path between A and B
            while (index_b > index_a)
            {
                rrt_path.erase(rrt_path.begin() + index_b);
                index_b--;
            }

            // Temp nodes only for inserting into the path
            Node *new_a = createNewNode(candidate_a, nullptr);
            Node *new_b = createNewNode(candidate_b, nullptr);

            // Save the nodes so we can clean them up after
            pruning_nodes_.push_back(new_a);
            pruning_nodes_.push_back(new_b);

            rrt_path.insert(rrt_path.begin() + index_a + 1, new_b);
            rrt_path.insert(rrt_path.begin() + index_a + 1, new_a);

            prune_count_l1++;
        }
    }

    // Level 2 pruning
    int p = 0;
    int p_next = 1;
    std::vector<Node*> new_path;
    new_path.reserve(rrt_path.size());
    new_path.push_back(rrt_path[0]);

    while (p != rrt_path.size() - 1)
    {
        Node* node_p = rrt_path[p];

        while (p_next + 1 < rrt_path.size() - 1)
        {
            Node* node_p_next = rrt_path[p_next + 1];
            if(detectCollisionEdge(esdf_map_.get(), node_p->position, node_p_next->position))
            {
                break;
            }

            p_next++;
        }

        new_path.push_back(rrt_path[p_next]);
        p = p_next;
        p_next++;
    }

    prune_count_l2 = rrt_path.size() - new_path.size();
    rrt_path.swap(new_path);

    // printf("Level 1: Pruned %d times\n", prune_count_l1);
    // printf("Level 2: Pruned %d times\n", prune_count_l2);
    // fprintf(stderr, "%ld waypoints after pruning\n", rrt_path.size());
}

void RRTConnect::convertPathToOutput(const std::vector<Node *> &rrt_path, Point3fVector &waypoints)
{
    waypoints.reserve(rrt_path.size());

    for (const Node *node : rrt_path)
    {
        waypoints.push_back(node->position);
    }
}

void RRTConnect::visualizePath(const Point3fVector &waypoints)
{
    std::vector<path_vis_t> rrt_path;
    for (const Point3f &p : waypoints)
    {
        path_vis_t pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        rrt_path.push_back(pt);
    }

    generateAndSendPath(vis_channel_, rrt_path, PATH_VIS_LINE, "RRT Waypoints");
}

bool RRTConnect::fixTree(Node *new_root)
{
    Node *nearest_node;
    double dist;

    // Find nearest node in tree
    std::tie(nearest_node, dist) = findNearest(new_root->position);

    // Need to make sure the nearest node is reachable
    if (detectCollisionEdge(esdf_map_.get(), nearest_node->position, new_root->position))
        return false;

    add(nearest_node, new_root);

    // Root should always have parent as null
    new_root->parent = nullptr;

    Node *prev_node = new_root;
    Node *cur_node = nearest_node;
    Node *next_node = nearest_node->parent;

    while (next_node != nullptr)
    {
        // 1. Change parent of current node to be previous node
        // 2. Delete previous node from children list of current node
        // 3. Add current node to children of previous node

        cur_node->parent = prev_node;
        cur_node->children.erase(std::find(cur_node->children.begin(), cur_node->children.end(), prev_node));
        prev_node->children.push_back(cur_node);

        prev_node = cur_node;
        cur_node = next_node;
        next_node = next_node->parent;
    }

    // Fix the root
    cur_node->parent = prev_node;
    cur_node->children.erase(std::find(cur_node->children.begin(), cur_node->children.end(), prev_node));
    prev_node->children.push_back(cur_node);

    return true;
}

bool RRTConnect::runRRT(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints)
{
    computeMapBounds();

    // Setup start and end nodes
    Node *q_start = createNewNode(start_pos, nullptr);
    Node *q_goal = createNewNode(end_pos, nullptr);

    // TODO: Need to handle case where global map has changed and caused a disconnect somewhere in the tree
    // If root is not null then we already have a tree. Fix up pointers before running the planning
    // if (root_ != nullptr)
    // {
    //     fprintf(stderr, "RRT Tree already exists. Attempting to use for replanning...\n");

    //     timer.start("Tree fixing");
    //     bool success = fixTree(q_start);
    //     timer.stop("Tree fixing");

    //     if (success)
    //         fprintf(stderr, "RRT Tree fixed succesfully. Reusing for planning.\n");
    //     else
    //     {
    //         fprintf(stderr, "Failed to reuse RRT tree for planning. Starting from scratch...\n");

    //         // Delete the tree and start from scratch
    //         cleanupTree();
    //     }
    // }

    root_ = q_start;
    std::vector<Node *> rrt_path;

    // Insert start into search structure
    int start_node_index = flatIndexfromPoint(q_start->position);
    nodes_[start_node_index].push_back(q_start);

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
            q_rand = q_goal;
        else
            q_rand = createRandomNode();

        // Try again if we couldnt get a random node
        if (!q_rand)
            continue;

        collision_found = false;

        // Find nearest node in tree
        std::tie(q_near, dist) = findNearest(q_rand->position);

        Point3f dir_vec = ((q_rand->position - q_near->position) / dist);

        // Continually step towards q_rand by rrt_min_distance and add a node if its collision free
        // Optimization: Use squared distances to avoid having to do the square root which is more expensive
        while ((q_rand->position - q_near->position).squaredNorm() > rrt_min_distance * rrt_min_distance)
        {
            q_connect = createNewNode(q_near->position + rrt_min_distance * dir_vec, nullptr);

            if (!detectCollisionEdge(esdf_map_.get(), q_near->position, q_connect->position, true))
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
        if (!collision_found && !detectCollisionEdge(esdf_map_.get(), q_near->position, q_rand->position))
        {
            add(q_near, q_rand);

            // If we ran goal biasing then the end point would be the goal, so exit
            if (q_rand == q_goal)
                break;
        }
        else if (q_rand != q_goal)
        {
            // Delete q_rand if we couldnt add to graph and it wasnt the goal node
            delete q_rand;
            q_rand = nullptr;
        }

        // Check if we can reach goal
        std::tie(q_near, dist) = findNearest(q_goal->position);

        if (dist <= rrt_goal_threshold)
        {
            add(q_near, q_goal);
            break;
        }

        attempts++;
    }

    if (q_goal->parent != nullptr)
    {
        printf("RRT solution found in %6.2fms and took %d attempts\n", (rc_nanos_monotonic_time() - start_time) / 1000000.0, attempts);
    }
    else
    {
        printf("RRT solution not found. Planning exceeded %6.2fms and took %d attempts\n", rrt_max_runtime_nanoseconds / 1000000.0, attempts);
        return false;
    }

    // Get the path from q_goal to q_start
    Node *cur = q_goal;
    while (cur != nullptr)
    {
        rrt_path.push_back(cur);
        cur = cur->parent;
    }

    // Reverse path since we traveresed tree from leaf to root but we want root to leaf
    std::reverse(rrt_path.begin(), rrt_path.end());

    pruneRRTPath(rrt_path);

    convertPathToOutput(rrt_path, waypoints);

    // Cleanup extra nodes created due to pruning
    cleanupPruning();
    cleanupTree(); // TODO: Remove once replanning is finished

    return true;
}

bool RRTConnect::isPlanningFeasible(const Point3f &start_pos, const Point3f &end_pos)
{
    if (detectCollision(esdf_map_.get(), start_pos))
    {
        printf("ERROR: Start point is in collision\n");
        return false;
    }
    else if (detectCollision(esdf_map_.get(), end_pos))
    {
        printf("ERROR: End point is in collision\n");
        return false;
    }

    return true;
}

bool RRTConnect::checkImmediatePath(const Point3f &start_pos, const Point3f &end_pos)
{
    if (!detectCollisionEdge(esdf_map_.get(), start_pos, end_pos))
    {
        printf("Immediate collision free path found!\n");
        return true;
    }

    return false;
}

bool RRTConnect::createPlan(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints)
{
    bool success = false;
    printf("Starting RRTConnect planner.\n");

    // Check preconditions for planning
    if (isPlanningFeasible(start_pos, end_pos))
    {
        if (checkImmediatePath(start_pos, end_pos))
        {
            waypoints.push_back(start_pos);
            waypoints.push_back(end_pos);

            success = true;
        }
        else
        {
            success = runRRT(start_pos, end_pos, waypoints);
        }

        visualizePath(waypoints);
    }

    return success;
}

void RRTConnect::tearDown()
{
    cleanupTree();
    cleanupPruning();
}