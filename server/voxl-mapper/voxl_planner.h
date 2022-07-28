#ifndef VOXL_PLANNER_H_
#define VOXL_PLANNER_H_

#include "global_planners/global_planner.h"
#include "local_planners/local_planner.h"
#include <mav_trajectory_generation/trajectory.h>
#include "planner_utils.h"
#include <voxblox/core/esdf_map.h>

namespace voxblox
{
    class TsdfServer;
}

enum class ControlMessageType
{
    PlanCommand,
    TrajectoryProtocol,
    Unknown
};

class VoxlPlanner
{
public:
    VoxlPlanner(){};
    ~VoxlPlanner();

    void setMap(voxblox::TsdfServer *mapper);
    void initMPA();
    bool planTo(const Point3f &start_pos, const Point3f &end_pos);
    void startFollowPath();
    void stopFollowPath();
    void sendStopCmd();
    void resetEstop();

private:
    void setGlobalPlanner(GlobalPlanner *global_planner);
    void setLocalPlanner(LocalPlanner *local_planner);

    bool getRobotPose(Point3f &cur_pose);

    static void controlPipeCallback(int ch, char *msg, int bytes, void *context);
    ControlMessageType getMessageType(char *msg, int bytes);
    void handlePlanCmd(char *msg, VoxlPlanner *planner);
    void handleProtocolMsg(char *msg, int bytes, VoxlPlanner *planner);

    GlobalPlanner *global_planner_;
    LocalPlanner *local_planner_;

    Point3fVector waypoints_;
    int render_ch_;
    int plan_ch_;

    voxblox::TsdfServer *mapper_;

    bool en_debug_ = false;
};

#endif