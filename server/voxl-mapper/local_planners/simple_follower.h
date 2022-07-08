#ifndef SIMPLE_FOLLOWER_H_
#define SIMPLE_FOLLOWER_H_

#include <mav_path_smoothing/loco_smoother.h>
#include <stdio.h>
#include <voxblox/core/esdf_map.h>

#include <memory>
#include <thread>

#include "local_planner.h"

class SimpleFollower : public LocalPlanner {
   public:
    SimpleFollower(std::shared_ptr<voxblox::EsdfMap> esdf_map, int plan_ch, int render_ch);
    ~SimpleFollower();

    void setPlan(const Point3fVector &waypoints);
    void setup();
    void start();
    void stop();

   private:
    void plannerThread();
    bool getInitialPlan();
    bool runPlanner(Point3f start_pose, Point3fVector &target_points, mav_trajectory_generation::Trajectory &trajectory);
    bool runSmoother(const Point3fVector &target_points,
                     mav_trajectory_generation::Trajectory &trajectory);
    bool sendTrajectory(
        const mav_trajectory_generation::Trajectory &trajectory, int split_id, double split_time, bool first_plan);
    void visualizePath(const Point3fVector &target_points);

    bool detectCollision(const Point3d &pos);
    float getMapDistance(const Point3d &position);
    double getMapDistanceAndGradient(const Point3d &position,
                                     Point3d *gradient);

    void computeSplitPointDetails(Point3f &start_pose, int &split_id, double &split_time);

    bool isBetween(Point3f a, Point3f b, Point3f c);

    Point3fVector waypoints_;
    mav_planning::LocoSmoother loco_smoother_;

    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    pthread_mutex_t map_mutex_;

    std::thread planning_thread_;
    std::atomic<bool> running_;

    int cur_waypoint_;

    trajectory_t current_traj_;

    int plan_ch_;
    int render_ch_;

    bool en_debug_ = false;
};

#endif