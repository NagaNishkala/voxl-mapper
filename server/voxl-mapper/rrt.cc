#include "rrt.h"
#include <cfloat>
#include <unistd.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <voxblox/core/common.h>


/////////////////////////////////////////////////////////////////////
// General Helper Funcs
/////////////////////////////////////////////////////////////////////
inline bool exists_ (const std::string& name)
{
    ifstream f(name.c_str());
    return f.good();
}

static uint64_t rc_nanos_monotonic_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000) + ts.tv_nsec;
}

static bool operator==(const point_xyz& lhs, const point_xyz& rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

// Constructor
RRTSTAR::RRTSTAR(Eigen::Vector3d start, Eigen::Vector3d end, std::shared_ptr<voxblox::EsdfMap> esdf_map_)
{
    startPos = start;
    endPos = end;
    esdf_map = esdf_map_;

    srand((unsigned)time(NULL));
    computeMapBounds();

    root = new Node;
    root->parent = NULL; // only root node should be parentless
    root->position = startPos;
    root->cost = 0.0;
    lastNode = root;
    int actual_index = flatIndexfromPoint(startPos);
    nodes[actual_index].push_back(root);

    // Loco smoother
    mav_planning::locoParams loco_params;
    loco_params.resample_trajectory_ = loco_resample_trajectory;
    loco_params.num_segments_ = loco_num_segments;
    loco_params.add_waypoints_ = loco_add_waypoints;
    loco_params.scale_time_ = loco_scale_time;
    loco_params.split_at_collisions_ = loco_split_at_collisions;

    loco_smoother_.setParameters(loco_params);
    // loco_smoother_.setMapDistanceCallback(std::bind(&RRTSTAR::getMapDistance, this, std::placeholders::_1));
    loco_smoother_.setInCollisionCallback(std::bind(&RRTSTAR::checkCollisionWithRobot, this, std::placeholders::_1));
    loco_smoother_.setDistanceAndGradientFunction(std::bind(&RRTSTAR::getMapDistanceAndGradient, this, std::placeholders::_1, std::placeholders::_2));

    loco_smoother_.loco_config.polynomial_degree = loco_poly_degree;
    loco_smoother_.loco_config.derivative_to_optimize = loco_derivative_to_optimize;
    loco_smoother_.loco_config.robot_radius = (robot_radius*2.0);
    loco_smoother_.loco_config.w_d = loco_smoothness_cost_weight;
    loco_smoother_.loco_config.w_c = loco_collision_cost_weight;
    loco_smoother_.loco_config.w_w = loco_waypoint_cost_weight;

    std::string benchmark_path = BENCHMARK_FILE;
    if (!exists_(benchmark_path)){
        std::ofstream outfile (benchmark_path);
        // header
        outfile << "Version,Collision,Distance,Total Time,Random Node Time,Nearest Node Time,Check Motion Time,Neighborhood Time,Children Update Time,Attempts,Nearest Fails,Random to Nearest Fails,Partial Segments,Tree Rewires\n";
        outfile.close();
    }
}

/////////////////////////////////////////////////////////////////////
// RRT* core functions
/////////////////////////////////////////////////////////////////////
bool RRTSTAR::computeMapBounds()
{
    if(esdf_map == nullptr){
        fprintf(stderr, "ERROR: esdf map is nullptr, cannot compute map bounds.\n");
        return false;
    }

    voxblox::utils::computeMapBoundsFromLayer(*esdf_map->getEsdfLayerPtr(), &lower_bound, &upper_bound);

    printf("Map Bounds:\n");
    printf("Lower-> x:%6.2f, y:%6.2f, z:%6.2f\n", lower_bound.x(), lower_bound.y(), lower_bound.z());
    printf("Upper-> x:%6.2f, y:%6.2f, z:%6.2f\n", upper_bound.x(), upper_bound.y(), upper_bound.z());

    ind_lower[0] = (double)lower_bound.x()/((double)voxels_per_side * (double)voxel_size);
    ind_lower[1] = (double)lower_bound.y()/((double)voxels_per_side * (double)voxel_size);
    ind_lower[2] = (double)lower_bound.z()/((double)voxels_per_side * (double)voxel_size);

    double ind_upper[3] = {
        (double)upper_bound.x()/((double)voxels_per_side * (double)voxel_size),
        (double)upper_bound.y()/((double)voxels_per_side * (double)voxel_size),
        (double)upper_bound.z()/((double)voxels_per_side * (double)voxel_size)
    };

    d_x = std::abs(ind_lower[0] - ind_upper[0]);
    d_y = std::abs(ind_lower[1] - ind_upper[1]);
    d_z = std::abs(ind_lower[2] - ind_upper[2]);

    // nodes max size will then be dx + (dy * dx) + (dz * dy * dz)
    nodes.resize(d_x + (d_y * d_x) + (d_z * d_y * d_x));
    return true;
}

double RRTSTAR::Cost(Node *q)
{
    return q->cost;
}

double RRTSTAR::PathCost(Node *qFrom, Node *qTo)
{
    double totalDistance = 0;

    for(Node *cur = qFrom; cur != qTo; cur = cur->parent){
        if(cur->parent == NULL){
            throw("ERROR: PathCost reached a node without a parent");
        }
        if (cur->parent->position == endPos) break;
        totalDistance += distance(cur, cur->parent);
    }

    return totalDistance;
}

double RRTSTAR::distance(Eigen::Vector3d p, Eigen::Vector3d q)
{
    Eigen::Vector3d dif = p - q;

    return sqrt(powf(dif.x(), 2) + powf(dif.y(), 2) + powf(dif.z(), 2));
}

double RRTSTAR::distance(Node* p, Node *q)
{
    return distance(p->position, q->position);
}

double RRTSTAR::man_dist(Eigen::Vector3d p, Eigen::Vector3d q)
{
    Eigen::Vector3d dif = p - q;
    return (std::abs(dif.x()) + std::abs(dif.y()) + std::abs(dif.z()));
}

int RRTSTAR::binSelect(int index)
{
    if (!nodes[index].empty()) return index;

    // check left/right, forward/back, up/down by 1. if those all fail, keep looking around it
    int x_offset = 1;
    int y_offset = d_x;
    int z_offset = d_y * d_x;

    // independent checks
    bool l_x, l_y, l_z, r_x, r_y, r_z;
    l_x = l_y = l_z = r_x = r_y = r_z = true;

    while (l_x || l_y || l_z || r_x || r_y || r_z){

        if (l_x && index-x_offset < 0) l_x = false;
        if (r_x && index+x_offset >= nodes.size()) r_x = false;

        if (l_y && index-y_offset < 0) l_y = false;
        if (r_y && index+y_offset >= nodes.size()) r_y = false;

        if (l_z && index-z_offset < 0) l_z = false;
        if (r_z && index+z_offset >= nodes.size()) r_z = false;

        if (r_x && !nodes.at(index+x_offset).empty()) return index+x_offset;
        if (l_x && !nodes.at(index-x_offset).empty()) return index-x_offset;

        if (r_y && !nodes.at(index+y_offset).empty()) return index+y_offset;
        if (l_y && !nodes.at(index-y_offset).empty()) return index-y_offset;

        if (r_z && !nodes.at(index+z_offset).empty()) return index+z_offset;
        if (l_z && !nodes.at(index-z_offset).empty()) return index-z_offset;

        x_offset += 1;
        y_offset += d_x;
        z_offset += d_y * d_x;
    }
    // default. if we fail, return where the root is
    return 0;
}

void RRTSTAR::near(Eigen::Vector3d point, vector<Node *>& out_nodes)
{
    int index = flatIndexfromPoint(point);

    // check left/right, forward/back, up/down by 1. if those all fail, keep looking around it
    int x_offset = 1;
    int y_offset = d_x;
    int z_offset = d_y * d_x;

    // independent checks
    bool l_x, l_y, l_z, r_x, r_y, r_z;
    l_x = l_y = l_z = r_x = r_y = r_z = true;

    while ((l_x || l_y || l_z || r_x || r_y || r_z) && out_nodes.size() < 10){

        if (l_x && index-x_offset < 0) l_x = false;
        if (r_x && index+x_offset >= nodes.size()) r_x = false;

        if (l_y && index-y_offset < 0) l_y = false;
        if (r_y && index+y_offset >= nodes.size()) r_y = false;

        if (l_z && index-z_offset < 0) l_z = false;
        if (r_z && index+z_offset >= nodes.size()) r_z = false;

        if (r_x && !nodes.at(index+x_offset).empty()) out_nodes.insert(out_nodes.end(), nodes[index+x_offset].begin(), nodes[index+x_offset].end());
        if (l_x && !nodes.at(index-x_offset).empty()) out_nodes.insert(out_nodes.end(), nodes[index-x_offset].begin(), nodes[index-x_offset].end());

        if (r_y && !nodes.at(index+y_offset).empty()) out_nodes.insert(out_nodes.end(), nodes[index+y_offset].begin(), nodes[index+y_offset].end());
        if (l_y && !nodes.at(index-y_offset).empty()) out_nodes.insert(out_nodes.end(), nodes[index-y_offset].begin(), nodes[index-y_offset].end());

        if (r_z && !nodes.at(index+z_offset).empty()) out_nodes.insert(out_nodes.end(), nodes[index+z_offset].begin(), nodes[index+z_offset].end());
        if (l_z && !nodes.at(index-z_offset).empty()) out_nodes.insert(out_nodes.end(), nodes[index-z_offset].begin(), nodes[index-z_offset].end());

        x_offset += 1;
        y_offset += d_x;
        z_offset += d_y * d_x;
    }

    return;
    // // return the whole container
    // int actual_index = flatIndexfromPoint(point);

    // if(!nodes[actual_index].empty()){
    //     out_nodes.insert(out_nodes.end(), nodes[actual_index].begin(), nodes[actual_index].end());
    // }
    // else {
    //     actual_index = binSelect(actual_index);
    //     out_nodes.insert(out_nodes.end(), nodes[actual_index].begin(), nodes[actual_index].end());
    // }
    // return;
}

int RRTSTAR::flatIndexfromPoint(Eigen::Vector3d point)
{
    voxblox::BlockIndex bi = voxblox::getGridIndexFromPoint<voxblox::BlockIndex>(point.cast<float>(), (1.0f/(voxels_per_side * voxel_size)));
    return (bi.x()-ind_lower[0]) + (bi.y()-ind_lower[1] * d_x) + ((bi.z()-ind_lower[2])*d_y*d_x);
}

Node* RRTSTAR::nearest(Eigen::Vector3d point, bool end)
{
    double minDist = DBL_MAX;
    Node *closest = NULL;

    int actual_index = flatIndexfromPoint(point);

    if (!nodes[actual_index].empty()){

        for (int i = 0; i < nodes[actual_index].size(); i++){

            if (nodes[actual_index][i] == NULL) continue;
            if (nodes[actual_index][i]->position == endPos && nodes[actual_index][i]->parent) continue;
            double dist = distance(point, nodes[actual_index][i]->position);
            if (dist <= 0.01){
                if (end) closest = nodes[actual_index][i];
                else closest = NULL;
                return closest;
            }
            else if (dist < minDist) {
                minDist = dist;
                closest = nodes[actual_index][i];
            }
        }
    }

    if (closest == NULL) {
    // else {
        actual_index = binSelect(actual_index);
        if (!nodes[actual_index].empty()){
            for (int i = 0; i < nodes[actual_index].size(); i++){
                double dist = distance(point, nodes[actual_index][i]->position);
                if (dist <= 0.01){
                    if (end) closest = nodes[actual_index][i];
                    else closest = NULL;
                    return closest;
                }
                else if (dist < minDist) {
                    minDist = dist;
                    closest = nodes[actual_index][i];
                }
            }
        }
    }
    return closest;
}

void RRTSTAR::trimNodes(int index)
{
    int index_to_drop = -1;
    int least_children = INT32_MAX;
    for (int i = 0; i < nodes[index].size(); i++){
        int curr_children = nodes[index][i]->children.size();
        if (curr_children < least_children){
            least_children = curr_children;
            index_to_drop = i;
        }
    }
    if (index_to_drop != -1) nodes[index].erase(nodes[index].begin() + index_to_drop);
}

void RRTSTAR::add(Node *qNearest, Node *qNew)
{
    if(qNew->parent){
        qNew->parent->children.erase(std::remove(qNew->parent->children.begin(), qNew->parent->children.end(), qNew), qNew->parent->children.end());
    }
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + distance(qNew, qNearest);
    qNearest->children.push_back(qNew);

    int actual_index = flatIndexfromPoint(qNew->position);
    if (nodes[actual_index].size() > SINGLE_GRID_CAPACITY) trimNodes(actual_index);
    nodes[actual_index].push_back(qNew);

    lastNode = qNew;
}

void RRTSTAR::updateChildren(Node* n, double cost_diff){
    for (int l = 0; l < n->children.size(); l++){
        n->children[l]->cost -= cost_diff;
        updateChildren(n->children[l], cost_diff);
    }
}

Node* RRTSTAR::getRandomNode(bool two_d)
{
    double x = ((upper_bound.x() - lower_bound.x()) * (double)rand()/(double)RAND_MAX) + lower_bound.x();
    double y = ((upper_bound.y() - lower_bound.y()) * (double)rand()/(double)RAND_MAX) + lower_bound.y();
    double z = ((upper_bound.z() - lower_bound.z()) * (double)rand()/(double)RAND_MAX) + lower_bound.z();

    Eigen::Vector3d rand_pt;
    if (two_d) rand_pt << roundf(x * 100) / 100, roundf(y * 100) / 100, startPos.z();
    else rand_pt << roundf(x * 100) / 100, roundf(y * 100) / 100, roundf(z * 100) / 100;

    Node* ret = new Node;
    ret->position = rand_pt;
    ret->parent = NULL;

    return ret;
}

bool RRTSTAR::reached()
{
    if (distance(lastNode->position, endPos) <= END_DIST_THRESHOLD)
        return true;
    return false;
}

void RRTSTAR::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// RRT* collision checking
///////////////////////////////////////////////////////////////////////////////////////////////////

double RRTSTAR::getMapDistance(Eigen::Vector3d position)
{
    double distance = 0.0;
    if (!(esdf_map->getDistanceAtPosition(position, false, &distance))) {
        // if we cannot identify a voxel close enough to this location WITHOUT interpolation, it is unknown so reject it
        if (rrt_treat_unknown_as_occupied){
            return 0.0;
        }
        else return esdf_default_distance;
    }
    return distance;
}

double RRTSTAR::getMapDistanceAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient)
{
    double distance = 0.0;
    if (!(esdf_map->getDistanceAndGradientAtPosition(position, false, &distance, gradient))) {
        // if we cannot identify a voxel close enough to this location WITHOUT interpolation, it is unknown so reject it
        // if (rrt_treat_unknown_as_occupied){
            return 0.0;
        // }
        // else return esdf_default_distance;
    }
    return distance;
}

bool RRTSTAR::checkCollisionWithRobot(Eigen::Vector3d robot_position)
{
    static double radius__ = robot_radius * 1.75;
    float distance = getMapDistance(robot_position);
    return radius__ >= (double)distance;
}

bool RRTSTAR::checkMotion(Eigen::Vector3d start, Eigen::Vector3d end)
{
    // make sure it is a nullptr
    std::pair<Eigen::Vector3d*, double> unused = std::pair<Eigen::Vector3d*, double>(nullptr, -1);
    return checkMotion(start, end, unused);
}

bool RRTSTAR::checkMotion(Eigen::Vector3d start, Eigen::Vector3d end, std::pair<Eigen::Vector3d*, double>& last_valid)
{
    voxblox::Point start_scaled, goal_scaled;
    voxblox::AlignedVector<voxblox::GlobalIndex> indices;

    Eigen::Vector3d direction_vec = end - start;
    double path_length = distance(start, end);

    // int dist_val = path_length;
    static const int max_jump = 0.75;

    for (double i = 0.1; i < path_length; i+=0.1){
        Eigen::Vector3d new_pt = start + (direction_vec * i);
        bool collision = checkCollisionWithRobot(new_pt);
        if (collision) return false;
        else {
            if (last_valid.first != nullptr){
                memcpy(last_valid.first, &new_pt, sizeof(new_pt));
                last_valid.second = static_cast<double>(i);
                i+=getMapDistance(new_pt);
            }
        }
        if (i >= max_jump) return false;
    }
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// smoothing extra functions
///////////////////////////////////////////////////////////////////////////////////////////////////

void RRTSTAR::nodesToEigen(mav_msgs::EigenTrajectoryPointVector* eigen_path)
{
    // sanity checks
    if (eigen_path == nullptr){
        fprintf(stderr, "Output path is a nullptr!\n");
        return;
    }
    else if (path.empty()){
        fprintf(stderr, "Waypoint path is empty!\n");
        return;
    }

     for (int i = 0; i < path.size(); i++){
        mav_msgs::EigenTrajectoryPoint curr_pt;
        curr_pt.position_W = path[i]->position;
        eigen_path->push_back(curr_pt);
    }
}

int RRTSTAR::simplifyPath()
{
    if (path.empty() || path.size() <= 3) return 0;
    int removals = 0;

    for(int j = path.size() - 1; j > 1 ; j--){
        for(int i = 0; i < j - 1; i ++){
            if(checkMotion(path[j]->position, path[i]->position)){
                path.erase(path.begin() + i + 1, path.begin() + j - 1);
                j -= j-i+1;
                removals++;
            }
        }
        if (path.empty() || path.size() <= 3) return removals;
    }
    return removals;
}

bool RRTSTAR::checkPathForCollisions(mav_msgs::EigenTrajectoryPointVector& path) {
    for (const mav_msgs::EigenTrajectoryPoint& point : path) {
        if (checkCollisionWithRobot(point.position_W)) {
            return true;
        }
    }
    return false;
}

bool RRTSTAR::locoSmooth(mav_msgs::EigenTrajectoryPointVector& coordinate_path, mav_msgs::EigenTrajectoryPointVector* path, mav_trajectory_generation::Trajectory* last_trajectory_)
{
    // loco_smoother_.setResampleTrajectory(true);
    // loco_smoother_.setAddWaypoints(false);

    // turi -- test
    // loco_smoother_.setPoly(coordinate_path.size(), loco_derivative_to_optimize);

    bool got = loco_smoother_.getTrajectoryBetweenWaypoints(coordinate_path, last_trajectory_);

    bool success = false;
    if (got){
        mav_msgs::EigenTrajectoryPoint::Vector states;
        double sampling_interval = 0.05;
        success = mav_trajectory_generation::sampleWholeTrajectory(*last_trajectory_, sampling_interval, path);
    }
    return got && success;
}

void RRTSTAR::drawTreeLayer(Node* root, std::vector<point_xyz> *tree)
{
    for (int i = 0; i < root->children.size(); i++){
        point_xyz curr_pt;
        curr_pt.x = (float)root->position.x();
        curr_pt.y = (float)root->position.y();
        curr_pt.z = (float)root->position.z();
        if (root->children[i] != nullptr){
            point_xyz curr_child;
            curr_child.x = (float)root->children[i]->position.x();
            curr_child.y = (float)root->children[i]->position.y();
            curr_child.z = (float)root->children[i]->position.z();
            tree->push_back(curr_pt);
            tree->push_back(curr_child);
            drawTreeLayer(root->children[i], tree);
        }
    }
}

void RRTSTAR::Solve(mav_trajectory_generation::Trajectory* last_trajectory_)
{
    Node* end = new Node;
    end->position = endPos;
    end->parent = NULL;

    // check if valid before starting exploration
    if (checkMotion( root->position, end->position )){
        fprintf(stderr, "collision free path!\n");
        path.push_back(root);
        path.push_back(end);

        mav_msgs::EigenTrajectoryPointVector base_path;
        nodesToEigen(&base_path);
        locoSmooth(base_path, &loco_path, last_trajectory_);
        return;
    }

    bool solved = false;    // flag for when we do not want to use first solution
    bool use_two_d = true;  // flag for checking 2d space only, will switch to 3d after condition is met
    int rand_ind = 0;

    /// TIMING VARIABLES ///
    double random_node_t, nearest_t, check_motion_t, neighborhood_t, children_t;
    random_node_t = nearest_t = check_motion_t = neighborhood_t = children_t = 0.0;
    double random_node_count, nearest_count, motion_count, neighborhood_count, children_count;
    random_node_count = nearest_count = motion_count = neighborhood_count = children_count = 0.0;
    uint64_t time_start, time_end;

    // for timing out entire solve loop
    uint64_t start_time = rc_nanos_monotonic_time();
    uint64_t end_time = 0;

    /// TREE SENDING - DEBUG ///
    std::vector<point_xyz> rrt_star_tree;   // tree "pointcloud"
    point_cloud_metadata_t tree_meta;
    tree_meta.magic_number = POINT_CLOUD_MAGIC_NUMBER;

    // fail counts
    int two_d_fails = 0;
    int qnearest_fails = 0;
    int random_to_nearest_fails = 0;
    int partial_segments_added = 0;
    int rewires = 0;
    int attempts = 0;

    // used for checking if the 2d search is actually going anywhere
    double dist_to_goal = 0.0;
    while (rrt_max_runtime_nanoseconds == -1 || start_time + rrt_max_runtime_nanoseconds > rc_nanos_monotonic_time()){
        attempts++;

        if (use_two_d){
            if (two_d_fails >= 25){
                fprintf(stderr, "Switching to 3D checks\n");
                use_two_d = false;
            }
            else if (dist_to_goal <= man_dist(lastNode->position, endPos)) two_d_fails++;
            else{
                dist_to_goal = man_dist(lastNode->position, endPos);
                two_d_fails = 0;
            }
        }

        time_start = rc_nanos_monotonic_time();
        Node *qRand = getRandomNode(use_two_d);
        if (attempts % 5 == 0){         // try to pull towards goal indirectly
            // qRand->position = Eigen::Vector3d( end->position.x(),  end->position.y(), qRand->position.z());
            switch(rand_ind){
                case 0:
                    qRand->position = Eigen::Vector3d( qRand->position.x(),  end->position.y(), end->position.z());
                    break;
                case 1:
                    qRand->position = Eigen::Vector3d( end->position.x(),  qRand->position.y(), end->position.z());
                    break;
                case 2:
                    qRand->position = Eigen::Vector3d( end->position.x(),  end->position.y(), qRand->position.z());
                    break;
            }
            if (rand_ind < 2) rand_ind++;
            else rand_ind = 0;
        }
        time_end = rc_nanos_monotonic_time();
        if (time_end > time_start){
            random_node_t += (time_end-time_start);
            random_node_count++;
        }

        if (qRand) {
            // qNearest will be a node from our tree - a previously inserted vertex
            time_start = rc_nanos_monotonic_time();

            Node *qNearest = nearest(qRand->position, false);

            time_end = rc_nanos_monotonic_time();
            if (time_end > time_start){
                nearest_t += (time_end-time_start);
                nearest_count++;
            }

            // make sure that the random pt is at least step_size away from the nearest point in the tree
            if (qNearest && distance(qRand->position, qNearest->position) > rrt_min_distance){ // testing
                // setup our last valid state
                Eigen::Vector3d temp;
                std::pair<Eigen::Vector3d*, double> last_valid_state = std::pair<Eigen::Vector3d*, double>(&temp, -1);

                // check if the path to the nearest pt is valid
                time_start = rc_nanos_monotonic_time();
                bool val = checkMotion(qNearest->position, qRand->position, last_valid_state);
                time_end = rc_nanos_monotonic_time();
                if (time_end > time_start){
                    check_motion_t += (time_end-time_start);
                    motion_count++;
                }

                if(!val){
                    if (last_valid_state.second != -1){
                        qRand->position = *last_valid_state.first;
                        partial_segments_added++;
                    }
                    else {
                        random_to_nearest_fails++;
                        continue;
                    }
                }

                // get our neighborhood around qRand
                vector<Node *> Qnear;

                time_start = rc_nanos_monotonic_time();
                near(qRand->position, Qnear);
                time_end = rc_nanos_monotonic_time();
                if (time_end > time_start){
                    neighborhood_t += (time_end-time_start);
                    neighborhood_count++;
                }
                if (end->parent) std::remove(Qnear.begin(), Qnear.end(), end);
                end->children.clear();
                Node *qMin = qNearest;

                // add the edge between qMin(lowest cost out of nodes in Qnear) and qRand
                add(qMin, qRand);

                // check if we can just straight up jump to the end
                if(checkMotion(endPos, qRand->position)){
                    if(end->parent){
                        if(PathCost(end, root) < distance(endPos, qRand->position) + PathCost(qRand, root)){
                            end->children.clear();
                            add(qRand, end);
                        }
                    }
                    else add(qRand, end);
                }

                // tree rewiring -----------------------------------------------------------------------------
                for(int j = 0; j < Qnear.size(); j++){
                    Node *qNear = Qnear[j];

                    time_start = rc_nanos_monotonic_time();
                    val = checkMotion(qRand->position, qNear->position);
                    time_end = rc_nanos_monotonic_time();
                    if (time_end > time_start){
                        check_motion_t += (time_end-time_start);
                        motion_count++;
                    }
                    if(val){
                        if(Cost(qRand) + distance(qRand, qNear) < Cost(qNear)){
                            rewires++;
                            Node *qParent = qNear->parent;
                            double cost_diff = Cost(qNear) - (Cost(qRand) + distance(qRand, qNear));

                            // Remove edge between qParent and qNear
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());

                            // Add edge between qRand and qNear
                            qNear->cost = Cost(qRand) + distance(qRand, qNear);
                            qNear->parent = qRand;
                            qRand->children.push_back(qNear);

                            time_start = rc_nanos_monotonic_time();
                            updateChildren(qNear, cost_diff);
                            time_end = rc_nanos_monotonic_time();
                            if (time_end > time_start){
                                children_t += (time_end-time_start);
                                children_count++;
                            }
                        }
                    }
                }
            } else qnearest_fails++;
        }
        if (rrt_send_tree){
            drawTreeLayer(root, &rrt_star_tree);
            tree_meta.timestamp_ns = 2; // from portal, TREE_FORMAT = 2;
            tree_meta.n_points = rrt_star_tree.size();
            tree_meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZ;
            if (tree_meta.n_points != 0) pipe_server_write_point_cloud(RENDER_CH, tree_meta, rrt_star_tree.data());
            // necessary so websocket can handle data stream, should handle portal side eventually
            rrt_star_tree.clear();
            usleep(75000);
        }
        if (!solved && reached()){
            solved = true;
            end_time = rc_nanos_monotonic_time();
            fprintf(stderr, "RRTSTAR Reached Destination\n");
            if(rrt_use_first_solution) break;
            else fprintf(stderr, "Attempting to refine current path");
        }
    }
    if (!solved) end_time = rc_nanos_monotonic_time();

    printf("\n------------------------------------------\n");
    printf("TIMING STATS\n");
    printf("------------------------------------------\n");
    printf("Solve time             ->          %6.2fms\n", (double)(end_time - start_time)/1000000.0 );
    printf("Total time             ->          %6.2fms\n", (double)(rc_nanos_monotonic_time() - start_time)/1000000.0);
    printf("Random node grab time  -> Average: %6.2fms, TOT: %6.2fms\n", (double)((random_node_t/(1000000* random_node_count))), random_node_t/1000000.0);
    printf("Nearest node time      -> Average: %6.2fms, TOT: %6.2fms\n", (double)((nearest_t/(1000000* nearest_count))), nearest_t/1000000.0);
    printf("Check motion time      -> Average: %6.2fms, TOT: %6.2fms\n", (double)((check_motion_t/(1000000* motion_count))), check_motion_t/1000000.0);
    printf("Neighborhood grab time -> Average: %6.2fms, TOT: %6.2fms\n", (double)((neighborhood_t/(1000000* neighborhood_count))), neighborhood_t/1000000.0);
    printf("Children update time   -> Average: %6.2fms, TOT: %6.2fms\n", (double)((children_t/(1000000* children_count))), children_t/1000000.0);
    printf("\n------------------------------------------\n");
    printf("GENERAL STATS\n");
    printf("------------------------------------------\n");
    printf("PATH LENGTH: %6.2f\n", distance(startPos, endPos));
    printf("ATTEMPTS: %d\n", attempts);
    printf("QNEAREST FAILS: %d\n", qnearest_fails);
    printf("RANDOM TO NEAREST FAILS: %d\n", random_to_nearest_fails);
    printf("PARTIAL SEGMENTS ADDED: %d\n", partial_segments_added);
    printf("TREE REWIRES: %d\n", rewires);
    printf("------------------------------------------\n");

    Node *q;
    if (!solved) printf("RRTSTAR FAILED: Exceeded max iterations. Outputting closest path\n");
    if (rrt_use_first_solution) q = lastNode;
    else if (end->parent) q = end;
    else q = nearest(endPos, true);

    // generate shortest path to destination.
    while (q->parent != NULL && !(std::count(path.begin(), path.end(), q))) {
        path.push_back(q);
        q = q->parent;
    }

    // if root node (start) or end node not in graph, add em in before we smooth
    if (std::find(path.begin(), path.end(), root) == path.end()) path.push_back(root);
    if (std::find(path.begin(), path.end(), end) == path.end())  path.insert(path.begin(), end);

    time_start = rc_nanos_monotonic_time();
    int removals = simplifyPath();
    time_end = rc_nanos_monotonic_time();
    fprintf(stderr, "Simplifying Path took: %6.2fms\nRemoved %d nodes\n", (time_end - time_start)/ 1000000.0, removals);

    // reverse!!!
    std::reverse(path.begin(), path.end());

    mav_msgs::EigenTrajectoryPointVector base_path;
    nodesToEigen(&base_path);

    bool collisions_on_base = checkPathForCollisions(base_path);
    fprintf(stderr, "Collisions along base path-> %s\n", collisions_on_base ? "yes" : "no");

    std::ofstream myfile;
    myfile.open (BENCHMARK_FILE, ios_base::app);
    myfile << RRT_VERSION << ", " <<  (collisions_on_base ? "no" : "yes") << ", " << distance(startPos, endPos) << ", " << (double)(rc_nanos_monotonic_time() - start_time)/1000000.0 << ", " << random_node_t/1000000.0 << ", " << nearest_t/1000000.0 << ", " << check_motion_t/1000000.0 << ", " << neighborhood_t/1000000.0 << ", " << children_t/1000000.0 << ", " << attempts << ", " << qnearest_fails << ", " << random_to_nearest_fails << ", " << partial_segments_added << ", " << rewires << "\n";
    myfile.close();

    time_start = rc_nanos_monotonic_time();
    bool loco_success = locoSmooth(base_path, &loco_path, last_trajectory_);
    time_end = rc_nanos_monotonic_time();
    fprintf(stderr, "Loco Smooth took: %6.2fms\nSuccess-> %s\n", (time_end - time_start)/ 1000000.0, loco_success ? "yes" : "no");

    deleteNodes(root);
    return;
}
