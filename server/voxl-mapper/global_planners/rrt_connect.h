#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include "global_planner.h"
#include <config_file.h>
#include <memory>
#include <voxblox/utils/planning_utils.h>
#include <voxblox/core/esdf_map.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <modal_pipe.h>
#include <modal_pipe_interfaces.h>
#include <ptcloud_vis.h>
#include <pthread.h>

#include "timer.h"

struct Node
{
    Eigen::Vector3d position;
    Node *parent;
    std::vector<Node *> children;
    int id;
};

class RRTConnect : public GlobalPlanner
{

public:
    RRTConnect(std::shared_ptr<voxblox::EsdfMap> esdf_map, int vis_channel);

    bool createPlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &end_pos, mav_trajectory_generation::Trajectory &trajectory);

    void tearDown();

private:
    /**
     * @brief Computes the map bounds and initializes the map variables
     * 
     * @return true the ESDF map exists and map variables were able to be computed
     * @return false ESDF map does not exist
     */
    bool computeMapBounds();

    /**
     * @brief Sets the loco smoother parameters
     * 
     */
    void setupSmoother();

    /**
     * @brief Detects if a collision occurs along an edge (includes start and end point in checks)
     * 
     * @param start the start point of the edge
     * @param end the end point of the edge
     * @param is_extend if the edge is computed as part of the Extend portion of RRT
     * @return true a collision was found along edge from start to end
     * @return false no collision found along edge from start to end
     */
    bool detectCollisionEdge(const Eigen::Vector3d &start, const Eigen::Vector3d &end, bool is_extend);

    /**
     * @brief Detects if the robot would be in collision at pos
     * 
     * @param pos the position to check for colission
     * @return true robot is in collision at pos
     * @return false robot not in collision as pos
     */
    bool detectCollision(const Eigen::Vector3d &pos);

    /**
     * @brief Get the distance to the closest obstacle at position
     * 
     * @param position position to check distance to obstalce for
     * @return double distance to nearest obstacle
     */
    double getMapDistance(const Eigen::Vector3d &position);\

    /**
     * @brief Get the distance and gradient of the closest obstacle at position
     * 
     * @param position position to check distance/gradient to obstalce for
     * @param gradient [output] gradient vector
     * @return double distance to nearest obstacle
     */
    double getMapDistanceAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient);

    /**
     * @brief Helper function to create a new node
     * 
     * @param position 3D point of node
     * @param parent parent of node
     * @return Node* newly created node
     */
    Node *createNewNode(const Eigen::Vector3d& position, Node* parent);

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
    std::pair<Node *, double> findNearest(const Eigen::Vector3d &point);

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
    int flatIndexfromPoint(const Eigen::Vector3d &point);

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
    void deleteNodes(Node *root);

    /**
     * @brief Cleanup the RRT tree and all associated search structures
     * 
     */
    void cleanupTree();

    bool fixTree(Node* new_root);

    /**
     * @brief Convert the rrt path to the appropriate eigen type for the smoother to process
     * 
     * @param eigen_path [output] path to be fed to smoother
     */
    void nodesToEigen(const std::vector<Node*> &rrt_path, mav_msgs::EigenTrajectoryPointVector &eigen_path);

    /**
     * @brief Runs the loco smoother on the path returned from RRT
     * 
     * @param waypoints points found by RRT planner
     * @param smoothed_path [output] points sampled along the smoothed path
     * @param final_trajectory [output] smoothed final trajectory
     * @return true smoother ran succesfully
     * @return false smoother failed
     */
    bool locoSmooth(const mav_msgs::EigenTrajectoryPointVector &waypoints, mav_msgs::EigenTrajectoryPointVector &smoothed_path, mav_trajectory_generation::Trajectory &final_trajectory);

    /**
     * @brief Handles preprocessing needed for running loco smoother
     * 
     * @param trajectory [output] smoothed final trajectory
     * @return true smoother ran succesfully
     * @return false smoother failed
     */
    bool runSmoother(const std::vector<Node*> &rrt_path, mav_trajectory_generation::Trajectory &trajectory);

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

    /**
     * @brief Deletes intermediate nodes created by Level 1 pruning
     * 
     */
    void cleanupPruning();

    /**
     * @brief Sends both RRT and smoothed trajectory path points to voxl-portal
     * 
     */
    void visualizePaths(const std::vector<Node*> &rrt_path);

    /**
     * @brief Sends ESDF data to voxl-portal to allow for visualizing of the map
     * 
     */
    void visualizeMap();

    inline double distance(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
    {
        Eigen::Vector3d dif = end - start;
        return dif.norm();
    }

    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    Eigen::Vector3d lower_bound_;
    Eigen::Vector3d upper_bound_;
    int d_x_, d_y_, d_z_;
    double ind_lower_[3];

    std::vector<std::vector<Node *>> nodes_;
    std::vector<Node *> pruning_nodes_;
    mav_msgs::EigenTrajectoryPointVector smoothed_path_;
    Node *root_;
    int node_counter_;

    int vis_channel_;

    mav_planning::LocoSmoother loco_smoother_;

    Timer timer;
};

#endif