#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include "global_planner.h"
#include <config_file.h>
#include <memory>
#include <voxblox/utils/planning_utils.h>
#include <voxblox/core/esdf_map.h>
#include <modal_pipe.h>
#include <modal_pipe_interfaces.h>
#include <pthread.h>

#include "timer.h"

struct Node
{
    Point3f position;
    Node *parent;
    std::vector<Node *> children;
    int id;
};

class RRTConnect : public GlobalPlanner
{

public:
    RRTConnect(std::shared_ptr<voxblox::EsdfMap> esdf_map, int vis_channel);

    bool createPlan(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints);

    void tearDown();

private:

    bool runRRT(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints);

    bool isPlanningFeasible(const Point3f &start_pos, const Point3f &end_pos);

    bool checkImmediatePath(const Point3f &start_pos, const Point3f &end_pos);

    /**
     * @brief Computes the map bounds and initializes the map variables
     * 
     * @return true the ESDF map exists and map variables were able to be computed
     * @return false ESDF map does not exist
     */
    bool computeMapBounds();

    /**
     * @brief Helper function to create a new node
     * 
     * @param position 3D point of node
     * @param parent parent of node
     * @return Node* newly created node
     */
    Node *createNewNode(const Point3f& position, Node* parent);

    /**
     * @brief Creates a new node at a random location inside map bounds
     * 
     * @return Node* new node with a random position
     */
    Node *createRandomNode();

    /**
     * @brief Returns nearest node to point
     * 
     * @param point position to find the nearest node to
     * @return std::pair<Node *, double> the nearest node and distance to node
     */
    std::pair<Node *, double> findNearest(const Point3f &point);

    /**
     * @brief Calculates corresponding bin of the node at index in nodes_.
     *        Helper function for finding nearest node
     * 
     * @param index index of node in nodes_
     * @return int bin in which to search
     */
    int binSelect(int index);

    /**
     * @brief Converts point to a bin index in nodes_
     * 
     * @param point point to convert
     * @return int bin index in nodes_
     */
    int flatIndexfromPoint(const Point3f &point);

    /**
     * @brief Add a node to the RRT tree
     * 
     * @param q_nearest the parent node
     * @param q_new the new node to add
     */
    void add(Node *q_nearest, Node *q_new);

    /**
     * @brief Cleanup nodes by recursively traversing nodes and deleting
     * 
     * @param root the starting point of the RRT tree
     */
    void deleteNodes(Node *&root);

    /**
     * @brief Cleanup the RRT tree and all associated search structures
     * 
     */
    void cleanupTree();

    bool fixTree(Node* new_root);

    /**
     * @brief Prunes the found RRT path in two levels.
     * 
     * Level 1: Pick two random edges and two random points along those edges.
     *          Attempt to connect these two points. If collision free then 
     *          remove all nodes along the path between these two points to
     *          create a shorter path. If not collision free, pick again.
     *          Run this for as many times as desired.
     * 
     * Level 2: Iterate over the path from start to end. For each node check
     *          all other nodes from end to start to see if the path is collision 
     *          free. If collision free then remove any nodes that this would skip.
     * 
     * 
     * Level 1 helps to create an overall shorter path. Level 2 helps to "thin out"
     * the path created by level 1's randomness. Level 2 also helps to further
     * shorten the path.
     */
    void pruneRRTPath(std::vector<Node*> &rrt_path);

    void convertPathToOutput(const std::vector<Node*> &rrt_path, Point3fVector &waypoints);

    /**
     * @brief Deletes intermediate nodes created by Level 1 pruning
     * 
     */
    void cleanupPruning();

    /**
     * @brief Sends both RRT and smoothed trajectory path points to voxl-portal
     * 
     */
    void visualizePath(const Point3fVector &waypoints);

    inline double distance(const Eigen::Vector3f &start, const Eigen::Vector3f &end)
    {
        Point3f dif = end - start;
        return dif.norm();
    }

    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    Point3f lower_bound_;
    Point3f upper_bound_;
    int d_x_, d_y_, d_z_;
    double ind_lower_[3];

    std::vector<std::vector<Node *>> nodes_;
    std::vector<Node *> pruning_nodes_;
    Node *root_;
    int node_counter_;

    int vis_channel_;

    Timer timer;
};

#endif