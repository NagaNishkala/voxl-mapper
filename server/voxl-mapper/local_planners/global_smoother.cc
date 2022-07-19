#include "global_smoother.h"
#include "path_vis.h"

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

    // int num_segments = loco_num_segments < waypoints_.size() ? loco_num_segments : waypoints_.size();
    // loco_smoother_.setNumSegments(num_segments);

    if (loco_smoother_.getTrajectoryBetweenWaypoints(waypoints_for_smoother, &trajectory, false))
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

        visualizePath(out);

        pipe_server_write(plan_ch_, &out, sizeof(trajectory_t));
    }
    else
    {
        printf("Smoother failed to find a path.\n");
    }
}

void GlobalSmoother::stop()
{
    waypoints_.clear();
}

void GlobalSmoother::visualizePath(trajectory_t &current_traj)
{
    std::vector<path_vis_t> traj;
    double sampling_dt = 0.05;

    for (int i = 0; i < current_traj.n_segments; i++)
    {
        poly_segment_t *s = &current_traj.segments[i];
        for (double t = 0.0; t < s->duration_s; t += sampling_dt)
        {
            path_vis_t pt;
            pt.x = eval_poly_at_t(s->n_coef, s->cx, t);
            pt.y = eval_poly_at_t(s->n_coef, s->cy, t);
            pt.z = eval_poly_at_t(s->n_coef, s->cz, t);
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            traj.push_back(pt);
        }
    }

    generateAndSendPath(render_ch_, traj, PATH_VIS_TRAJECTORY, "GlobalSmoother");
}

GlobalSmoother::~GlobalSmoother()
{
}