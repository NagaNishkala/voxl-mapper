#ifndef LOCAL_A_STAR_H_
#define LOCAL_A_STAR_H_

#include <mav_path_smoothing/loco_smoother.h>
#include <stdio.h>
#include "voxl_mapper.h"

#include <memory>
#include <thread>

#include "local_planner.h"

class LocalAStar : public LocalPlanner
{
public:
    LocalAStar(voxblox::TsdfServer *map, int plan_ch, int render_ch);
    ~LocalAStar();

    void setPlan(const Point3fVector &waypoints);
    void setup();
    void start();
    void stop();

private:

    typedef struct Node
    {
        float total_cost;
        float travel_cost;
        float heuristic_cost;
        voxblox::GlobalIndex esdf_idx;
        Node *parent;
        bool closed;
        int steps;
        int idx_in_parent = 0;

        bool operator<(const Node &b)
        {
            return total_cost > b.total_cost;
        }

        Node(float total_cost, float travel_cost, float heuristic_cost, const voxblox::GlobalIndex &esdf_idx, Node *parent)
            : total_cost(total_cost),
              travel_cost(travel_cost),
              heuristic_cost(heuristic_cost),
              esdf_idx(esdf_idx),
              parent(parent),
              closed(false)
        {
            if(parent)
                steps = parent->steps + 1;
            else
                steps = 0;
        }
    } Node;

    struct Compare
    {
        bool operator()(Node *lhs, Node *rhs)
        {
            return ((lhs->total_cost) >= (rhs->total_cost));
        }
    };

    void plannerThread();
    bool getInitialPlan();
    bool runPlanner(const Point3f &start_pose, Point3fVector &target_points, mav_trajectory_generation::Trajectory &trajectory);
    bool runSmoother(const Point3fVector &target_points,
                     mav_trajectory_generation::Trajectory &trajectory);
    bool sendTrajectory(
        const mav_trajectory_generation::Trajectory &trajectory, int split_id, double split_time, bool first_plan = false);
    void visualizePath(const Point3fVector &target_points);

    bool computeSplitPointDetails(Point3f &start_pose, int &split_id, double &split_time);

    float computeHeuristic(voxblox::GlobalIndex cur_idx, voxblox::GlobalIndex goal_idx);
    bool isBetween(Point3f a, Point3f b, Point3f c);

    void pruneAStarPath(std::vector<Node *> &path);

    Point3f computeGoalPosition(const Point3f& start_pos);

    Point3fVector waypoints_;
    mav_planning::LocoSmoother loco_smoother_;

    voxblox::TsdfServer* map_;
    pthread_mutex_t map_mutex_;

    std::thread planning_thread_;
    std::atomic<bool> running_;

    trajectory_t current_traj_;

    Point3f start_vel;
    Point3f start_acc;

    int next_segment_id_;

    int plan_ch_;
    int render_ch_;

    bool en_debug_ = false;
};

#endif