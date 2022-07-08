#include "voxl_planner.h"
#include <modal_pipe.h>
#include "global_planners/rrt_connect.h"
#include "local_planners/simple_follower.h"
#include "voxl_mapper.h"
#include "timing_utils.h"
#include "voxl_trajectory.h"

#define PROCESS_NAME "voxl-mapper"
#define PLAN_NAME "plan_msgs"
#define PLAN_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR PLAN_NAME "/")

#define RENDER_NAME "voxl_planner_render"
#define RENDER_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR RENDER_NAME "/")

#define PLAN_NAME "plan_msgs"
#define PLAN_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR PLAN_NAME "/")

#define PLAN_HOME "plan_home"
#define PLAN_TO "plan_to"
#define FOLLOW_PATH "follow_path"
#define STOP_FOLLOWING "stop_following"
#define CONTROL_COMMANDS (PLAN_HOME "," PLAN_TO "," FOLLOW_PATH "," STOP_FOLLOWING)

void VoxlPlanner::initMPA()
{
    pipe_info_t render_info = {
        RENDER_NAME,
        RENDER_LOCATION,
        "js_render",
        PROCESS_NAME,
        1024 * 1024 * 64,
        0};

    pipe_info_t plan_info = {
        PLAN_NAME,
        PLAN_LOCATION,
        "trajectory_t",
        PROCESS_NAME,
        1024 * 1024 * 64,
        0};

    render_ch_ = pipe_server_get_next_available_channel();
    if (pipe_server_create(render_ch_, render_info, 0))
    {
        printf("FAILED TO START RENDER SERVER PIPE\n");
    }

    plan_ch_ = pipe_server_get_next_available_channel();
    if (pipe_server_create(plan_ch_, plan_info, SERVER_FLAG_EN_CONTROL_PIPE))
    {
        printf("FAILED TO START PLAN SERVER PIPE\n");
    }

    pipe_server_set_available_control_commands(plan_ch_, CONTROL_COMMANDS);
    pipe_server_set_control_cb(plan_ch_, controlPipeCallback, this);
}

void VoxlPlanner::setMap(voxblox::TsdfServer *mapper)
{
    mapper_.reset(mapper);

    setGlobalPlanner(new RRTConnect(mapper->getEsdfMapPtr(), render_ch_));
    setLocalPlanner(new SimpleFollower(mapper->getEsdfMapPtr(), plan_ch_, render_ch_));
}

void VoxlPlanner::setGlobalPlanner(GlobalPlanner *global_planner)
{
    global_planner_ = global_planner;

    if (global_planner_)
        global_planner_->setup();
}

void VoxlPlanner::setLocalPlanner(LocalPlanner *local_planner)
{
    local_planner_ = local_planner;

    if (local_planner_)
        local_planner_->setup();
}

bool VoxlPlanner::planTo(const Point3f &start_pos, const Point3f &end_pos)
{
    if (!local_planner_ || !global_planner_)
    {
        fprintf(stderr, "ERROR: Planners did not initialize correctly\n");
    }

    local_planner_->stop();

    if (en_debug_)
        fprintf(stderr, "Starting planner\n");

    waypoints_.clear();
    bool success = global_planner_->createPlan(start_pos, end_pos, waypoints_);

    return success;
}

void VoxlPlanner::startFollowPath()
{
    if(!waypoints_.empty())
    {
        local_planner_->setPlan(waypoints_);
        local_planner_->start();
    }
    else
        fprintf(stderr, "No path to follow\n");

}

void VoxlPlanner::stopFollowPath()
{
    waypoints_.clear();
    local_planner_->stop();
}

void VoxlPlanner::sendEstopCmd()
{
    trajectory_t estop;
    estop.magic_number = TRAJECTORY_MAGIC_NUMBER;
    estop.traj_command = TRAJ_CMD_ESTOP;
    pipe_server_write(plan_ch_, &estop, sizeof(estop));
    fprintf(stderr, "Planner sent Estop command.\n");
}

void VoxlPlanner::controlPipeCallback(__attribute__((unused)) int ch, char *msg, __attribute__((unused)) int bytes, __attribute__((unused)) void *context)
{
    VoxlPlanner *planner = (VoxlPlanner *)context;

    switch (planner->getMessageType(msg, bytes))
    {
    case ControlMessageType::PlanCommand:
        planner->handlePlanCmd(msg, planner);
        break;
    case ControlMessageType::TrajectoryProtocol:
        planner->handleProtocolMsg(msg, bytes, planner);
        break;
    case ControlMessageType::Unknown:
        fprintf(stderr, "Unknown message type received via control pipe to planner\n");
        break;
    }

    return;
}

ControlMessageType VoxlPlanner::getMessageType(char *msg, int bytes)
{
    if (bytes < 0 || msg == NULL)
        return ControlMessageType::Unknown;

    traj_protocol_t *new_ptr = (traj_protocol_t *)msg;

    if (new_ptr->magic_number == TRAJ_PROTOCOL_MAGIC_NUMBER)
        return ControlMessageType::TrajectoryProtocol;

    if (msg[0] > 32 && msg[0] < 127)
        return ControlMessageType::PlanCommand;

    return ControlMessageType::Unknown;
}

void VoxlPlanner::handlePlanCmd(char *msg, VoxlPlanner *planner)
{
    voxblox::TsdfServer *mapper = planner->mapper_.get();
    std::string cmd(msg);

    if (cmd == PLAN_HOME)
    {
        printf("Client requested plan home.\n");
        Eigen::Vector3d start_pose, goal_pose;

        rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
        if (!mapper->getRobotPose(tf_body_wrt_fixed, rc_nanos_monotonic_time()))
            return;

        start_pose << tf_body_wrt_fixed.d[0][3], tf_body_wrt_fixed.d[1][3], tf_body_wrt_fixed.d[2][3];
        goal_pose << 0.0, 0.0, -1.5;

        mapper->addNewRobotPositionToEsdf(start_pose.x(), start_pose.y(), start_pose.z());
        mapper->updateEsdf(true);

        fprintf(stderr, "Using start pose of: x: %6.2f, y: %6.2f, z: %6.2f\n", start_pose.x(), start_pose.y(), start_pose.z());
        fprintf(stderr, "Using goal pose of: x: %6.2f, y: %6.2f, z: %6.2f\n", goal_pose.x(), goal_pose.y(), goal_pose.z());

        planner->planTo(start_pose.cast<float>(), goal_pose.cast<float>());
        return;
    }
    else if (cmd.compare(0, 7, PLAN_TO) == 0)
    {
        printf("Client requested plan to location\n");

        Eigen::Vector3d start_pose, goal_pose;

        // Get drones position
        rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
        if (!mapper->getRobotPose(tf_body_wrt_fixed, rc_nanos_monotonic_time()))
            return;

        start_pose << tf_body_wrt_fixed.d[0][3], tf_body_wrt_fixed.d[1][3], tf_body_wrt_fixed.d[2][3];

        char *goal_ptr;
        goal_ptr = strtok(msg, ":");
        goal_ptr = strtok(NULL, ":");

        std::string goal_str(goal_ptr);
        std::string::size_type sz; // alias of size_t

        double x = std::stod(goal_str, &sz);
        double y = std::stod(goal_str.substr(sz + 1));
        double z = start_pose.z();

        goal_pose << x, y, z;

        mapper->addNewRobotPositionToEsdf(start_pose.x(), start_pose.y(), start_pose.z());
        mapper->updateEsdf(true);

        printf("Using start pose of: x: %6.2f, y: %6.2f, z: %6.2f\n", start_pose.x(), start_pose.y(), start_pose.z());
        printf("Using goal pose of: x: %6.2f, y: %6.2f, z: %6.2f\n", goal_pose.x(), goal_pose.y(), goal_pose.z());

        planner->planTo(start_pose.cast<float>(), goal_pose.cast<float>());
        return;
    }
    else if (strcmp(msg, FOLLOW_PATH) == 0)
    {
        printf("Client requested to follow last path\n");
        planner->startFollowPath();
        return;
    }
    else if (strcmp(msg, STOP_FOLLOWING) == 0)
    {
        // This will cause the local planner to stop twice.
        // Once when we call stopFollowPath and a second time when we
        // send out the estop trajectory command and receive an estop protocol msg
        // as a reply.
        fprintf(stderr, "Stop following path request received\n");
        planner->stopFollowPath();
        planner->sendEstopCmd();
    }
}

void VoxlPlanner::handleProtocolMsg(char *msg, int bytes, VoxlPlanner *planner)
{
    int n_packets;
    traj_protocol_t *protocol_msg = pipe_validate_traj_protocol_t(msg, bytes, &n_packets);

    if (protocol_msg == NULL)
        return;

    int last_eval_packet = -1;

    for (int i = 0; i < n_packets; i++)
    {
        switch (protocol_msg->type)
        {
        case msg_type::ACK:
            printf("Received ACK traj protocol msg\n");
            break;

        case msg_type::NACK:
            printf("Received NACK traj protocol msg\n");
            break;

        case msg_type::EVALUATED:
            last_eval_packet = i;
            break;

        case msg_type::ESTOP:
            printf("Received Estop traj protocol msg. Stopped following path\n");
            planner->stopFollowPath();
            break;

        default:
            fprintf(stderr, "ERROR: Unknown protocol message type");
            break;
        }
    }

    if (last_eval_packet >= 0)
    {
        // Use the latest evaluation data to set our current segment
        protocol_msg = protocol_msg + last_eval_packet;
        planner->local_planner_->setCurrentSegment(protocol_msg->segment_id, protocol_msg->segment_t);
    }
}

VoxlPlanner::~VoxlPlanner()
{
    if (local_planner_)
    {
        local_planner_->stop();
        local_planner_->tearDown();
    }

    if (global_planner_)
        global_planner_->tearDown();

    delete local_planner_;
    delete global_planner_;
}