#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include "global_planner.h"
#include <config_file.h>
#include <memory>
#include <voxblox/core/esdf_map.h>
#include <mav_local_planner/mav_local_planner.h>

#include <modal_pipe.h>
#include <modal_pipe_interfaces.h>
#include <ptcloud_vis.h>

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
    RRTConnect(std::shared_ptr<voxblox::EsdfMap> esdf_map_, int vis_channel);

    bool createPlan(const Eigen::Vector3d &startPos, const Eigen::Vector3d &endPos, mav_trajectory_generation::Trajectory &trajectory);

    ~RRTConnect();

private:
    bool computeMapBounds();
    void setupSmoother();

    bool detectCollisionEdge(const Eigen::Vector3d &start, const Eigen::Vector3d &end, bool isConnected);
    bool detectCollision(const Eigen::Vector3d &pos);
    double getMapDistance(const Eigen::Vector3d &position);
    double getMapDistanceAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient);

    Node *createNewNode(const Eigen::Vector3d& position, Node* parent);
    Node *getRandomNode();

    std::pair<Node *, double> findNearest(const Eigen::Vector3d &point);
    int binSelect(int index);
    int flatIndexfromPoint(const Eigen::Vector3d &point);

    void add(Node *qNearest, Node *qNew);
    void deleteNodes(Node *root);

    void nodesToEigen(mav_msgs::EigenTrajectoryPointVector &eigen_path);
    bool locoSmooth(const mav_msgs::EigenTrajectoryPointVector &coordinate_path, mav_msgs::EigenTrajectoryPointVector &path, mav_trajectory_generation::Trajectory &last_trajectory);
    bool runSmoother(mav_trajectory_generation::Trajectory &trajectory);

    void pruneRRTPath();
    void cleanupPruning();

    void visualizePaths();
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
    std::vector<Node *> rrt_path_;
    std::vector<Node *> pruning_nodes_;
    mav_msgs::EigenTrajectoryPointVector smoothed_path_;
    Node *root_;
    int node_counter_;

    int vis_channel_;

    mav_planning::LocoSmoother loco_smoother_;

    Timer timer;
};

#endif