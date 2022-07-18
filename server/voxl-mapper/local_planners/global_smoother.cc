#include "global_smoother.h"

GlobalSmoother::GlobalSmoother(voxblox::TsdfServer *map, int plan_ch, int render_ch)
    : map_(map),
      plan_ch_(plan_ch),
      render_ch_(render_ch)
{
}

void GlobalSmoother::setup()
{
    setupSmootherFromConfig(loco_smoother_, map_->getEsdfMapPtr().get());
}

void GlobalSmoother::setPlan(const Point3fVector &waypoints)
{
    // Copy the waypoints
    waypoints_ = Point3fVector(waypoints);
}

void GlobalSmoother::start()
{
    mav_msgs::EigenTrajectoryPointVector waypoints_for_smoother;
    convertPointsToSmootherFormat(waypoints_, waypoints_for_smoother);

    mav_trajectory_generation::Trajectory trajectory;

    if (loco_smoother_.getTrajectoryBetweenWaypoints(waypoints_for_smoother, &trajectory))
    {
        int segment_id = 0;

        trajectory_t out;

        if (!convertMavTrajectoryToVoxlTrajectory(trajectory, out))
        {
            printf("Failed to convert trajectory to expected type\n");
            return;
        }

        out.magic_number = TRAJECTORY_MAGIC_NUMBER;

        // Set segment id for each segment whilst incrementing id
        for (int i = 0; i < out.n_segments; i++)
            out.segments[i].id = segment_id++;

        out.traj_command = TRAJ_CMD_LOAD_AND_START;

        printf("Sending trajectory to plan channel\n");
        pipe_server_write(plan_ch_, &out, sizeof(trajectory_t));
    }
    else
    {
        printf("Smoother failed to find a path.\n");
    }
}

void GlobalSmoother::stop()
{
}

GlobalSmoother::~GlobalSmoother()
{
}