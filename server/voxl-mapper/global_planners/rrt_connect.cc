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
      vis_channel_(vis_channel),
      last_create_idx_(0)
{
    // Set the random seed
    srand(time(nullptr));

    lower_bound_ = Point3f::Zero();
    upper_bound_ = Point3f::Zero();
}

bool RRTConnect::computeMapBounds()
{
    if (esdf_map_ == nullptr)
    {
        printf("ERROR: esdf map is nullptr, cannot compute map bounds.\n");
        return false;
    }

    voxblox::utils::computeObservedMapBoundsFromLayer<voxblox::EsdfVoxel>(*esdf_map_->getEsdfLayerPtr(), &lower_bound_, &upper_bound_);

    printf("Map Bounds:\n");
    printf("Lower-> x:%6.2f, y:%6.2f, z:%6.2f\n", (double)lower_bound_.x(), (double)lower_bound_.y(), (double)lower_bound_.z());
    printf("Upper-> x:%6.2f, y:%6.2f, z:%6.2f\n", (double)upper_bound_.x(), (double)upper_bound_.y(), (double)upper_bound_.z());

    return true;
}

RRTConnect::Node *RRTConnect::createNewNode(const Point3f &position, Node *parent)
{
    Node *new_node = new Node{.position = position, .parent = parent, .children = std::vector<Node *>(), .id = node_counter_};
    node_counter_++;

    if (parent != nullptr)
        parent->children.push_back(new_node);

    return new_node;
}

RRTConnect::Node *RRTConnect::createRandomNode()
{
    float x = ((upper_bound_.x() - lower_bound_.x()) * (float)rand() / (float)RAND_MAX) + lower_bound_.x();
    float y = ((upper_bound_.y() - lower_bound_.y()) * (float)rand() / (float)RAND_MAX) + lower_bound_.y();
    float z = ((upper_bound_.z() - lower_bound_.z()) * (float)rand() / (float)RAND_MAX) + lower_bound_.z();

    Point3f rand_pt(roundf(x * 100) / 100, roundf(y * 100) / 100, roundf(z * 100) / 100);

    return createNewNode(rand_pt, nullptr);
}

std::pair<RRTConnect::Node *, float> RRTConnect::findNearest(const Point3f &point)
{
    Node *closest = nullptr;
    float distance = FLT_MAX;

    // 3000 was chosen because it was the point at which the number of attempts stopped increasing
    // TODO: A smarter way would be to make this adaptive i.e. start at some lowish number and then
    // check when the time to search becomes greater than the time to create the octree
    if (points_.size() - last_create_idx_ > 3000)
    {
        octree_.initialize(points_);

        last_create_idx_ = points_.size() - 1;

        int32_t idx = octree_.findNeighbor<unibn::L2Distance<Point3f>>(point);

        closest = nodes_[idx];
        distance = (point - nodes_[idx]->position).norm();
    }
    else
    {
        int32_t idx = octree_.findNeighbor<unibn::L2Distance<Point3f>>(point);

        if (idx > 0)
        {
            closest = nodes_[idx];
            distance = (point - nodes_[idx]->position).squaredNorm();
        }

        for (int i = last_create_idx_; i < points_.size(); i++)
        {
            float d = (points_[i] - point).squaredNorm();

            if (d < distance)
            {
                distance = d;
                closest = nodes_[i];
            }
        }

        distance = std::sqrt(distance);
    }

    return std::make_pair(closest, distance);
}

void RRTConnect::add(Node *q_nearest, Node *q_new)
{
    q_new->parent = q_nearest;
    q_nearest->children.push_back(q_new);

    nodes_.push_back(q_new);

    points_.push_back(q_new->position);
}

void RRTConnect::deleteNodes(Node *&root)
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

    nodes_.clear();
    points_.clear();
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
    std::vector<Node *> new_path;
    new_path.reserve(rrt_path.size());
    new_path.push_back(rrt_path[0]);

    while (p != rrt_path.size() - 1)
    {
        Node *node_p = rrt_path[p];

        while (p_next + 1 < rrt_path.size() - 1)
        {
            Node *node_p_next = rrt_path[p_next + 1];
            if (detectCollisionEdge(esdf_map_.get(), node_p->position, node_p_next->position))
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
    usleep(1e6);

    // Note: The tree cannot be deleted for this to work
    // (i.e. cleanupTree() should not be called before this line runs)
    if (rrt_send_tree)
    {
        std::vector<path_vis_t> rrt_tree;
        std::vector<Node *> stack;
        stack.push_back(root_);

        while (!stack.empty())
        {
            Node *root = stack.back();
            stack.pop_back();

            path_vis_t root_pt;
            root_pt.x = root->position.x();
            root_pt.y = root->position.y();
            root_pt.z = root->position.z();
            root_pt.r = 0;
            root_pt.g = 255;
            root_pt.b = 0;

            for (int i = 0; i < root->children.size(); i++)
            {
                stack.push_back(root->children[i]);
                Node *child = root->children[i];

                path_vis_t pt;
                pt.x = child->position.x();
                pt.y = child->position.y();
                pt.z = child->position.z();
                pt.r = 255;
                pt.g = 0;
                pt.b = 0;
                rrt_tree.push_back(root_pt);
                rrt_tree.push_back(pt);
            }
        }

        generateAndSendPath(vis_channel_, rrt_tree, PATH_VIS_TREE, "RRT Tree");
    }
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
    int64_t a = rc_nanos_monotonic_time();
    computeMapBounds();
    fprintf(stderr, "compute = %f\n", (double)(rc_nanos_monotonic_time() - a) / 1e9);

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
    nodes_.push_back(q_start);
    points_.push_back(start_pos);

    uint64_t start_time = rc_nanos_monotonic_time();
    int attempts = 0;
    Node *q_rand = nullptr;
    Node *q_connect = nullptr;
    Node *q_near = nullptr;
    float dist = 0;
    bool collision_found;

    while (rrt_max_runtime_nanoseconds == -1 || start_time + rrt_max_runtime_nanoseconds > rc_nanos_monotonic_time())
    {
        // Get random node or use goal
        if (attempts % 50 == 0)
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

    cleanupTree(); // TODO: Remove once replanning is finished

    return success;
}

void RRTConnect::tearDown()
{
    cleanupTree();
    cleanupPruning();
}