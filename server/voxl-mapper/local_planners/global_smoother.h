#ifndef GLOBAL_SMOOTHER_H
#define GLOBAL_SMOOTHER_H

#include <mav_path_smoothing/loco_smoother.h>
#include "voxl_mapper.h"
#include "local_planner.h"

class GlobalSmoother : public LocalPlanner
{
    public:
    GlobalSmoother(voxblox::TsdfServer *map, int plan_ch, int render_ch);
    ~GlobalSmoother();

    void setPlan(const Point3fVector &waypoints);
    void setup();
    void start();
    void stop();

    private:
    Point3fVector waypoints_;
    mav_planning::LocoSmoother loco_smoother_;
    voxblox::TsdfServer* map_;

    int plan_ch_;
    int render_ch_;
};

#endif