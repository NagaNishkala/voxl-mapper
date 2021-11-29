#ifndef RRTSTAR_H
#define RRTSTAR_H

#include "voxl_mapper.h"
#include "config_file.h"
#include "ptcloud_vis.h" // for point_xyz def
#include <iostream>
#include <fstream>

#define RRT_VERSION "0.0.2"
#define BENCHMARK_FILE "/data/voxl_mapper/benchmark.csv"

#define END_DIST_THRESHOLD 0.75
#define SINGLE_GRID_CAPACITY 16

#define RENDER_CH 5

using namespace std;

typedef struct Node {
    vector<Node *> children;
    Node *parent = NULL;
    Eigen::Vector3d position;
    double cost = -1.0;
}Node;

class RRTSTAR
{
public:
    RRTSTAR(Eigen::Vector3d startPos, Eigen::Vector3d endPos, shared_ptr<voxblox::EsdfMap> esdf_map_);

    /// solves RRT*, generates rough waypoint path
    void Solve(mav_trajectory_generation::Trajectory* last_trajectory_);

    /// returns true if our lastNode is closer than END_DIST_THRESHOLD, otherwise false
    bool reached();

    /// smoother - T/F if successful
    bool locoSmooth(mav_msgs::EigenTrajectoryPointVector& coordinate_path, mav_msgs::EigenTrajectoryPointVector* path,
    mav_trajectory_generation::Trajectory* last_trajectory_);

    /// rough waypoint path
    vector<Node *> path;
    /// smoothed loco path
    mav_msgs::EigenTrajectoryPointVector loco_path;

private:
    /// sets the lower and upper bounds for the problem, also sets up our "grids"
    bool computeMapBounds();

    /// returns q->cost
    double Cost(Node *q);

    /// returns cost of path from qFrom to qTo represented by distance
    double PathCost(Node *qFrom, Node *qTo);

    /// distance funcs from position p to q
    double distance(Eigen::Vector3d p, Eigen::Vector3d q);
    /// helper overload w/Nodes
    double distance(Node* p, Node *q);
    double man_dist(Eigen::Vector3d p, Eigen::Vector3d q);

    /// fills the out_nodes vector with the neighborhood around a given position
    void near(Eigen::Vector3d point, vector<Node *>& out_nodes);

    /// returns a ptr to the closest node in our tree to a given position, null if the node is already in our tree
    Node* nearest(Eigen::Vector3d point, bool end);

    /// inserts our new node qNew into the tree with an edge to qNearest
    void add(Node *qNearest, Node *qNew);

    /// given a cost difference, will update the cost of each child of n recursively
    void updateChildren(Node* n, double cost_diff);

    /// generates a random position within map bounds that is NOT in collision, returns a node with this position
    Node* getRandomNode(bool two_d);

    /// directly checks esdf map for closest voxl to position (w/interpolation) and returns its distance
    double getMapDistance(Eigen::Vector3d position);

    /// same as above + gradient, for loco smoother
    double getMapDistanceAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient);

    /// returns true if the robot_position is in a collision, false otherwise
    bool checkCollisionWithRobot(Eigen::Vector3d robot_position);

    /// returns true if a valid motion, false otherwise. If last_valid is setup, will the pair with the last valid
    /// position and percentage along the path without collision
    bool checkMotion(Eigen::Vector3d start, Eigen::Vector3d end, std::pair<Eigen::Vector3d*, double>& last_valid);
    /// helper overload w/out pair
    bool checkMotion(Eigen::Vector3d start, Eigen::Vector3d end);

    /// converts the class member path to a trajectory message
    void nodesToEigen(mav_msgs::EigenTrajectoryPointVector* eigen_path);

    /// performs vertex connecting to reduce waypoints
    int simplifyPath();

    /// will check each waypoint of a path, and return true if in collision, false otherwise
    bool checkPathForCollisions(mav_msgs::EigenTrajectoryPointVector& path);

    /// will recursively draw out the entire tree, given a Node* to start at
    void drawTreeLayer(Node* root, std::vector<point_xyz> *tree);

    /// cleanup, deletes each node in tree starting from root
    void deleteNodes(Node *root);

    /// returns a "flat" grid index for our array given a position
    int flatIndexfromPoint(Eigen::Vector3d point);

    /// finds the closest bin to index that has at least 1 point in it
    int binSelect(int index);

    /// called by add(), will remove the node with the least children in a given bin
    void trimNodes(int index);

    shared_ptr<voxblox::EsdfMap> esdf_map;

    Eigen::Vector3d lower_bound;
    Eigen::Vector3d upper_bound;

    Node *root, *lastNode;
    Eigen::Vector3d startPos, endPos;
    vector<vector<Node*>> nodes;
    mav_planning::LocoSmoother loco_smoother_;

    int d_x, d_y, d_z;
    double ind_lower[3];
};

#endif // RRTSTAR_H
