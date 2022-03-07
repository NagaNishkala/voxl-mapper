#include "voxl_mapper.h"
#include "conversions.h"
#include "mesh_vis.h"
#include "ptcloud_vis.h"
#include "rrt.h"
#include "obs_pc_filter.h"
#include "trajectory_interface.h"
#include <unistd.h>
#include <fcntl.h>
#include <unordered_map>
#include <mav_local_planner/conversions.h>
#include <mav_trajectory_generation/trajectory_sampling.h>



#define PROCESS_NAME "voxl-mapper"

#define RAINBOW_REPEAT_DIST 3

// server pipes
#define COSTMAP_NAME                    "voxl_mapper_costmap"
#define COSTMAP_LOCATION                (MODAL_PIPE_DEFAULT_BASE_DIR COSTMAP_NAME "/")
#define COSTMAP_CH                      0

#define MESH_NAME                       "voxl_mesh"
#define MESH_LOCATION                   (MODAL_PIPE_DEFAULT_BASE_DIR MESH_NAME "/")
#define MESH_CH                         1

#define ALIGNED_PTCLOUD_NAME            "voxl_mapper_aligned_ptcloud"
#define ALIGNED_PTCLOUD_LOCATION        (MODAL_PIPE_DEFAULT_BASE_DIR ALIGNED_PTCLOUD_NAME "/")
#define ALIGNED_PTCLOUD_CH              2

#define PLAN_NAME                       "plan_msgs"
#define PLAN_LOCATION                   (MODAL_PIPE_DEFAULT_BASE_DIR PLAN_NAME "/")
#define PLAN_CH                         3

#define RENDER_NAME                     "voxl_planner_render"
#define RENDER_LOCATION                 (MODAL_PIPE_DEFAULT_BASE_DIR RENDER_NAME "/")
#define RENDER_CH                       5

// client pipes
#define BODY_WRT_FIXED_POSE_PATH    MODAL_PIPE_DEFAULT_BASE_DIR "vvpx4_body_wrt_fixed/"
#define QVIO_SIMPLE_LOCATION	    MODAL_PIPE_DEFAULT_BASE_DIR "qvio/"

#define MPA_VVPX4_CH       7
#define MPA_TOF_CH         8
#define MPA_DEPTH_0_CH     9
#define MPA_DEPTH_1_CH     10
#define MPA_DEPTH_2_CH     11
#define MPA_DEPTH_3_CH     12

// control stuff
#define PLAN_HOME       "plan_home"
#define RESET_VIO       "reset_vio"
#define SAVE_MAP        "save_map"
#define LOAD_MAP        "load_map"
#define CLEAR_MAP       "clear_map"
#define PLAN_TO         "plan_to"
#define FOLLOW_PATH     "follow_path"


#define CONTROL_COMMANDS (PLAN_HOME "," RESET_VIO "," SAVE_MAP "," LOAD_MAP "," CLEAR_MAP "," PLAN_TO "," FOLLOW_PATH)

pthread_mutex_t pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t esdf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tsdf_mutex = PTHREAD_MUTEX_INITIALIZER;

rc_tfv_ringbuf_t buf = RC_TF_RINGBUF_INITIALIZER;


namespace voxblox {

TsdfServer::TsdfServer(const TsdfMap::Config &config, const TsdfIntegratorBase::Config &integrator_config,
                       const MeshIntegratorConfig &mesh_config, bool debug, bool timing){

    tsdf_map_.reset(new TsdfMap(config));
    // fast, merged, or simple
    tsdf_integrator_.reset(new FastTsdfIntegrator(integrator_config, tsdf_map_->getTsdfLayerPtr()));
    // mesh params
    mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));
    mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

    str_esdf_save_path.assign(esdf_save_path, BUF_LEN);
    str_tsdf_save_path.assign(tsdf_save_path, BUF_LEN);
    str_mesh_save_path.assign(mesh_save_path, BUF_LEN);

    en_debug = debug;
    en_timing = timing;
    costmap_updates_only = false;

    // tof has a seperate cb over regular depth ptc
    if (tof_enable){
        pipe_client_set_simple_helper_cb(MPA_TOF_CH, _pc_helper_cb, this);
        pipe_client_set_connect_cb(MPA_TOF_CH, _pc_connect_cb, this);
        pipe_client_set_disconnect_cb(MPA_TOF_CH, _pc_disconnect_cb, this);
    }
    if (depth_pipe_0_enable){
        pipe_client_set_point_cloud_helper_cb(MPA_DEPTH_0_CH, _stereo_pc_helper_cb, this);
        pipe_client_set_connect_cb(MPA_DEPTH_0_CH, _pc_connect_cb, this);
        pipe_client_set_disconnect_cb(MPA_DEPTH_0_CH, _pc_disconnect_cb, this);
    }
    if (depth_pipe_1_enable){
        pipe_client_set_point_cloud_helper_cb(MPA_DEPTH_1_CH, _stereo_pc_helper_cb, this);
        pipe_client_set_connect_cb(MPA_DEPTH_1_CH, _pc_connect_cb, this);
        pipe_client_set_disconnect_cb(MPA_DEPTH_1_CH, _pc_disconnect_cb, this);
    }
    if (depth_pipe_2_enable){
        pipe_client_set_point_cloud_helper_cb(MPA_DEPTH_2_CH, _stereo_pc_helper_cb, this);
        pipe_client_set_connect_cb(MPA_DEPTH_2_CH, _pc_connect_cb, this);
        pipe_client_set_disconnect_cb(MPA_DEPTH_2_CH, _pc_disconnect_cb, this);
    }
    if (depth_pipe_3_enable){
        pipe_client_set_point_cloud_helper_cb(MPA_DEPTH_3_CH, _stereo_pc_helper_cb, this);
        pipe_client_set_connect_cb(MPA_DEPTH_3_CH, _pc_connect_cb, this);
        pipe_client_set_disconnect_cb(MPA_DEPTH_3_CH, _pc_disconnect_cb, this);
    }

    pipe_server_set_available_control_commands(PLAN_CH, CONTROL_COMMANDS);
    pipe_server_set_control_cb(PLAN_CH, _control_pipe_cb, this);
}

uint64_t TsdfServer::rc_nanos_monotonic_time(){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000) + ts.tv_nsec;
}

rc_tf_t TsdfServer::get_rc_tf_t(int ch){
    switch (ch)
    {
    case MPA_TOF_CH:
        return tf_tof_wrt_body;

    case MPA_DEPTH_0_CH:
        return tf_depth0_wrt_body;
    
    case MPA_DEPTH_1_CH:
        return tf_depth1_wrt_body;

    case MPA_DEPTH_2_CH:
        return tf_depth2_wrt_body;
    
    case MPA_DEPTH_3_CH:
        return tf_depth3_wrt_body;

    default:
        fprintf(stderr, "INVALID CHANNEL NUMBER FOR TFS\n");
        return tf_tof_wrt_body;
    }
}

int64_t TsdfServer::get_dif_per_frame(int ch){
    switch (ch)
    {
    case MPA_TOF_CH:
        return 1000000000/tof_rate;

    case MPA_DEPTH_0_CH:
        return 1000000000/depth0_rate;
    
    case MPA_DEPTH_1_CH:
        return 1000000000/depth1_rate;

    case MPA_DEPTH_2_CH:
        return 1000000000/depth2_rate;
    
    case MPA_DEPTH_3_CH:
        return 1000000000/depth3_rate;

    default:
        fprintf(stderr, "INVALID CHANNEL NUMBER FOR DATA RATE\n");
        return 1000000000/30.0;
    }
}

int TsdfServer::get_index_by_ch(int ch){
    switch (ch)
    {
    case MPA_TOF_CH:
        return 0;

    case MPA_DEPTH_0_CH:
        return 1;
    
    case MPA_DEPTH_1_CH:
        return 2;

    case MPA_DEPTH_2_CH:
        return 3;
    
    case MPA_DEPTH_3_CH:
        return 4;

    default:
        fprintf(stderr, "INVALID CHANNEL NUMBER FOR INDEX\n");
        return 0;
    }
}

void TsdfServer::_pc_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context){
    printf("Connected to depth pipe\n");
    return;
}

void TsdfServer::_pc_helper_cb(__attribute__((unused)) int ch, char *data, int bytes, void *context){
    static int64_t prev_ts = 0;
    //class instance
    TsdfServer *server = (TsdfServer *)context;

    static rc_tf_t tf_cam_wrt_body = server->get_rc_tf_t(ch);
    static int64_t fixed_ts_dif = server->get_dif_per_frame(ch);
    static int aligned_index = server->get_index_by_ch(ch);

    if (server->planning) return;

    //check if falling behind
    if (pipe_client_bytes_in_pipe(ch) > 0){
        fprintf(stderr, "WARNING bytes left in tof point cloud pipe\n");
    }

    // if we are receiving data too fast, drop it
    if (prev_ts && server->rc_nanos_monotonic_time() - prev_ts < fixed_ts_dif)
        return;

    prev_ts = server->rc_nanos_monotonic_time();

    // validate data
    int n_packets;
    tof_data_t *data_array = pipe_validate_tof_data_t(data, bytes, &n_packets);
    if (data_array == NULL)
        return;
    if (n_packets > 1){
        fprintf(stderr, "skipped %d point clouds\n", n_packets - 1);
    }

    // grab the latest packet if we got more than 2 point clouds in 1 pipe read
    tof_data_t tof_data = data_array[n_packets - 1];
    int64_t curr_ts = tof_data.timestamp_ns;

    rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
    int ret = rc_tf_ringbuf_get_tf_at_time(&buf, curr_ts, &tf_body_wrt_fixed);
    if (ret < 0){
        fprintf(stderr, "ERROR fetching tf from tf ringbuffer\n");
        if (ret == -2){
            printf("there wasn't sufficient data in the buffer\n");
        }
        if (ret == -3){
            printf("the requested timestamp was too new\n");
        }
        if (ret == -4){
            printf("the requested timestamp was too old\n");
        }
        return;
    }

    // setup our voxblox structs for pointcloud and colors
    voxblox::Pointcloud ptcloud;
    voxblox::Colors _colors;

    tof_pc_downsample(MPA_TOF_SIZE, tof_data.points, tof_data.confidences, 4.0, 0.15, 4, &ptcloud);
    _colors.resize(ptcloud.size());

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // now combine tfs
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    rc_tf_t tf_tof_wrt_fixed;
    rc_tf_combine_two(tf_body_wrt_fixed, tf_cam_wrt_body, &tf_tof_wrt_fixed);

    Eigen::Matrix<float, 4, 4> mat_tof_to_fixed;
    mat_tof_to_fixed << tf_tof_wrt_fixed.d[0][0], tf_tof_wrt_fixed.d[0][1], tf_tof_wrt_fixed.d[0][2], tf_tof_wrt_fixed.d[0][3],
        tf_tof_wrt_fixed.d[1][0], tf_tof_wrt_fixed.d[1][1], tf_tof_wrt_fixed.d[1][2], tf_tof_wrt_fixed.d[1][3],
        tf_tof_wrt_fixed.d[2][0], tf_tof_wrt_fixed.d[2][1], tf_tof_wrt_fixed.d[2][2], tf_tof_wrt_fixed.d[2][3],
        0.0, 0.0, 0.0, 1.0;

    const voxblox::Transformation vb_tof_to_fixed(mat_tof_to_fixed);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DEBUG - aligned pointcloud
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    voxblox::Pointcloud _pub_ptcloud;
    voxblox::transformPointcloud(vb_tof_to_fixed, ptcloud, &_pub_ptcloud); // transform it to what we will be inserting

    point_cloud_metadata_t aligned_ptc_meta;
    aligned_ptc_meta.magic_number = POINT_CLOUD_MAGIC_NUMBER;
    aligned_ptc_meta.timestamp_ns = aligned_index;
    aligned_ptc_meta.n_points = ptcloud.size();
    aligned_ptc_meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZ;

    pipe_server_write_point_cloud(ALIGNED_PTCLOUD_CH, aligned_ptc_meta, _pub_ptcloud.data());

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // pointcloud gradient coloring
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (int i=0; i<ptcloud.size();  i++){
        float height = _pub_ptcloud[i].z();

        //floating point modulus
        int mod = 0;
        int intheight = (int)height;
        while((mod+1)*RAINBOW_REPEAT_DIST <= intheight) mod++;
        while((mod)  *RAINBOW_REPEAT_DIST > intheight) mod--;

        height -= mod*RAINBOW_REPEAT_DIST;
        height /= RAINBOW_REPEAT_DIST;

        float a=height*5;
        int X=(int)(a);
        int Y=(int)(255*(a-X));
        int r,g,b;
        switch(X)
        {
            case 0: r=255;g=Y;b=0;break;
            case 1: r=255-Y;g=255;b=0;break;
            case 2: r=0;g=255;b=Y;break;
            case 3: r=0;g=255-Y;b=255;break;
            case 4: r=Y;g=0;b=255;break;
            case 5: r=255;g=0;b=255;break;
        }

        _colors[i].r = r;
        _colors[i].g = g;
        _colors[i].b = b;
        _colors[i].a = 127;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // drone position
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Vector3d _curr_pose;
    _curr_pose << tf_tof_wrt_fixed.d[0][3], tf_tof_wrt_fixed.d[1][3], tf_tof_wrt_fixed.d[2][3];
    pthread_mutex_lock(&pose_mutex);
    server->curr_pose = _curr_pose;
    pthread_mutex_unlock(&pose_mutex);

    // PHEW, finally, send in the point cloud to TSDF
    uint64_t start_time = server->rc_nanos_monotonic_time();
    server->integratePointcloud(vb_tof_to_fixed, ptcloud, _colors, false);
    uint64_t end_time = server->rc_nanos_monotonic_time();

    if (server->en_timing){
        printf("Integrating Pointcloud Took: %0.1f ms\n", (end_time - start_time) / 1000000.0);
    }

    // clear small sphere (0.2m radius) around our drones pose in the esdf map
    start_time = server->rc_nanos_monotonic_time();
    server->esdf_integrator_->addNewRobotPosition(Point(_curr_pose.x(), _curr_pose.y(), _curr_pose.z()));
    end_time = server->rc_nanos_monotonic_time();
    if (server->en_timing){
        printf("Clearing Sphere Took: %0.1f ms\n", (end_time - start_time) / 1000000.0);
    }
    return;
}

void TsdfServer::_stereo_pc_helper_cb(__attribute__((unused)) int ch, point_cloud_metadata_t meta, void* data, void* context){
    static int64_t prev_ts = 0;
    //class instance
    TsdfServer *server = (TsdfServer *)context;

    if (server->planning) return;

    static rc_tf_t tf_cam_wrt_body = server->get_rc_tf_t(ch);
    static int64_t fixed_ts_dif = server->get_dif_per_frame(ch);
    static int aligned_index = server->get_index_by_ch(ch);

    //check if falling behind
    if (pipe_client_bytes_in_pipe(ch) > 0){
        fprintf(stderr, "WARNING bytes left in tof point cloud pipe\n");
    }

    // if we are receiving data too fast, drop it
    if (prev_ts && server->rc_nanos_monotonic_time() - prev_ts < fixed_ts_dif)
        return;

    prev_ts = server->rc_nanos_monotonic_time();

    float new_data [meta.n_points][3];
    memcpy( new_data, data, sizeof(float) * meta.n_points * 3);
    int64_t curr_ts = meta.timestamp_ns;

    rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
    int ret = rc_tf_ringbuf_get_tf_at_time(&buf, curr_ts, &tf_body_wrt_fixed);
    if (ret < 0){
        fprintf(stderr, "ERROR fetching tf from tf ringbuffer\n");
        if (ret == -2){
            printf("there wasn't sufficient data in the buffer\n");
        }
        if (ret == -3){
            printf("the requested timestamp was too new\n");
        }
        if (ret == -4){
            printf("the requested timestamp was too old\n");
        }
        return;
    }

    // setup our voxblox structs for pointcloud and colors
    voxblox::Pointcloud ptcloud;
    voxblox::Colors _colors;

    dfs_pc_downsample(meta.n_points, new_data, 4.0, 0.08, 3, &ptcloud);
    _colors.resize(ptcloud.size());

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // now combine tfs
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    rc_tf_t tf_tof_wrt_fixed;
    rc_tf_combine_two(tf_body_wrt_fixed, tf_cam_wrt_body, &tf_tof_wrt_fixed);

    Eigen::Matrix<float, 4, 4> mat_tof_to_fixed;
    mat_tof_to_fixed << tf_tof_wrt_fixed.d[0][0], tf_tof_wrt_fixed.d[0][1], tf_tof_wrt_fixed.d[0][2], tf_tof_wrt_fixed.d[0][3],
        tf_tof_wrt_fixed.d[1][0], tf_tof_wrt_fixed.d[1][1], tf_tof_wrt_fixed.d[1][2], tf_tof_wrt_fixed.d[1][3],
        tf_tof_wrt_fixed.d[2][0], tf_tof_wrt_fixed.d[2][1], tf_tof_wrt_fixed.d[2][2], tf_tof_wrt_fixed.d[2][3],
        0.0, 0.0, 0.0, 1.0;

    const voxblox::Transformation vb_tof_to_fixed(mat_tof_to_fixed);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DEBUG - aligned pointcloud
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    voxblox::Pointcloud _pub_ptcloud;
    voxblox::transformPointcloud(vb_tof_to_fixed, ptcloud, &_pub_ptcloud); // transform it to what we will be inserting

    point_cloud_metadata_t aligned_ptc_meta;
    aligned_ptc_meta.magic_number = POINT_CLOUD_MAGIC_NUMBER;
    aligned_ptc_meta.timestamp_ns = aligned_index;
    aligned_ptc_meta.n_points = ptcloud.size();
    aligned_ptc_meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZ;

    pipe_server_write_point_cloud(ALIGNED_PTCLOUD_CH, aligned_ptc_meta, _pub_ptcloud.data());

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // pointcloud gradient coloring
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (int i=0; i<ptcloud.size();  i++){
        float height = _pub_ptcloud[i].z();

        //floating point modulus
        int mod = 0;
        int intheight = (int)height;
        while((mod+1)*RAINBOW_REPEAT_DIST <= intheight) mod++;
        while((mod)  *RAINBOW_REPEAT_DIST > intheight) mod--;

        height -= mod*RAINBOW_REPEAT_DIST;
        height /= RAINBOW_REPEAT_DIST;

        float a=height*5;
        int X=(int)(a);
        int Y=(int)(255*(a-X));
        int r,g,b;
        switch(X)
        {
            case 0: r=255;g=Y;b=0;break;
            case 1: r=255-Y;g=255;b=0;break;
            case 2: r=0;g=255;b=Y;break;
            case 3: r=0;g=255-Y;b=255;break;
            case 4: r=Y;g=0;b=255;break;
            case 5: r=255;g=0;b=255;break;
        }

        _colors[i].r = r;
        _colors[i].g = g;
        _colors[i].b = b;
        _colors[i].a = 127;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // drone position
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Vector3d _curr_pose;
    _curr_pose << tf_tof_wrt_fixed.d[0][3], tf_tof_wrt_fixed.d[1][3], tf_tof_wrt_fixed.d[2][3];
    pthread_mutex_lock(&pose_mutex);
    server->curr_pose = _curr_pose;
    pthread_mutex_unlock(&pose_mutex);

    // PHEW, finally, send in the point cloud to TSDF
    pthread_mutex_lock(&tsdf_mutex);
    uint64_t start_time = server->rc_nanos_monotonic_time();
    server->integratePointcloud(vb_tof_to_fixed, ptcloud, _colors, false);
    uint64_t end_time = server->rc_nanos_monotonic_time();
    pthread_mutex_unlock(&tsdf_mutex);

    if (server->en_timing){
        printf("Integrating Pointcloud Took: %0.1f ms\n", (end_time - start_time) / 1000000.0);
    }

    // clear small sphere (0.2m radius) around our drones pose in the esdf map
    pthread_mutex_lock(&esdf_mutex);
    start_time = server->rc_nanos_monotonic_time();
    server->esdf_integrator_->addNewRobotPosition(Point(_curr_pose.x(), _curr_pose.y(), _curr_pose.z()));
    end_time = server->rc_nanos_monotonic_time();
    pthread_mutex_unlock(&esdf_mutex);

    if (server->en_timing){
        printf("Clearing Sphere Took: %0.1f ms\n", (end_time - start_time) / 1000000.0);
    }
    return;
}


void TsdfServer::_pc_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context){
    fprintf(stderr, "\r" CLEAR_LINE FONT_BLINK "Disconnected from Ptcloud server\n" RESET_FONT);
}

void TsdfServer::_vio_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context){
    printf("Connected to VIO server\n");
}

void TsdfServer::_vio_helper_cb(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context){
    // validate data
    int n_packets;
    pose_vel_6dof_t *d = pipe_validate_pose_vel_6dof_t(data, bytes, &n_packets);

    // if there was an error OR no packets received, just return;
    if (d == NULL)
        return;
    if (n_packets <= 0)
        return;

    for (int i = 0; i < n_packets; i++){
        rc_tf_ringbuf_insert_pose(&buf, d[i]);
    }
    return;
}

void TsdfServer::_vio_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context){
    printf("\r" CLEAR_LINE FONT_BLINK "Disconnected from VIO server\n");
}

int TsdfServer::initMPA(){
    rc_tf_ringbuf_alloc(&buf, 5000);

    // server side pipes
    int flags = 0;

    pipe_info_t costmap_info = { \
        COSTMAP_NAME,
        COSTMAP_LOCATION,
        "costmap",
        PROCESS_NAME,
        1024*1024*64,
        0
    };

    pipe_info_t mesh_info = { \
        MESH_NAME,
        MESH_LOCATION,
        "mesh_metadata_t",
        PROCESS_NAME,
        1024*1024*64,
        0
    };

    pipe_info_t aligned_info = { \
        ALIGNED_PTCLOUD_NAME,
        ALIGNED_PTCLOUD_LOCATION,
        "point_cloud_metadata_t",
        PROCESS_NAME,
        1024*1024,
        0
    };

    pipe_info_t plan_info = { \
        PLAN_NAME,
        PLAN_LOCATION,
        "trajectory_t",
        PROCESS_NAME,
        1024*1024*64,
        0
    };

    pipe_info_t render_info = { \
        RENDER_NAME,
        RENDER_LOCATION,
        "js_render",
        PROCESS_NAME,
        1024*1024*64,
        0
    };

    if (pipe_server_create(COSTMAP_CH, costmap_info, flags)){
        printf("FAILED TO START COSTMAP SERVER PIPE\n");
    }

    if (pipe_server_create(MESH_CH, mesh_info, flags)){
        printf("FAILED TO START MESH SERVER PIPE\n");
    }

    if (pipe_server_create(ALIGNED_PTCLOUD_CH, aligned_info, flags)){
        printf("FAILED TO START ALIGNED PTCLOUD SERVER PIPE\n");
    }

    if (pipe_server_create(PLAN_CH, plan_info, SERVER_FLAG_EN_CONTROL_PIPE)){
        printf("FAILED TO START PLAN SERVER PIPE\n");
    }

    if (pipe_server_create(RENDER_CH, render_info, flags)){
        printf("FAILED TO START RENDER SERVER PIPE\n");
    }

    // VIO
    pipe_client_set_simple_helper_cb(MPA_VVPX4_CH, _vio_helper_cb, NULL);
    pipe_client_set_connect_cb(MPA_VVPX4_CH, _vio_connect_cb, NULL);
    pipe_client_set_disconnect_cb(MPA_VVPX4_CH, _vio_disconnect_cb, NULL);

    printf("waiting for server at %s\n", BODY_WRT_FIXED_POSE_PATH);

    pipe_client_open(MPA_VVPX4_CH, BODY_WRT_FIXED_POSE_PATH, PROCESS_NAME,
                                EN_PIPE_CLIENT_SIMPLE_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                                POSE_6DOF_RECOMMENDED_READ_BUF_SIZE);

    // request these pipes per config file defs
    if (tof_enable){
        printf("waiting for server at %s\n", tof_pipe);
        pipe_client_open(MPA_TOF_CH, tof_pipe, PROCESS_NAME,
                         EN_PIPE_CLIENT_SIMPLE_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                         sizeof(tof_data_t) * 10);

    }
    if (depth_pipe_0_enable){
        printf("waiting for server at %s\n", depth_pipe_0);
        pipe_client_open(MPA_DEPTH_0_CH, depth_pipe_0, PROCESS_NAME,
                         EN_PIPE_CLIENT_POINT_CLOUD_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                         sizeof(tof_data_t) * 10);
    }
    if (depth_pipe_1_enable){
        printf("waiting for server at %s\n", depth_pipe_1);
        pipe_client_open(MPA_DEPTH_1_CH, depth_pipe_1, PROCESS_NAME,
                         EN_PIPE_CLIENT_POINT_CLOUD_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                         sizeof(tof_data_t) * 10);
    }
    if (depth_pipe_2_enable){
        printf("waiting for server at %s\n", depth_pipe_2);
        pipe_client_open(MPA_DEPTH_2_CH, depth_pipe_2, PROCESS_NAME,
                         EN_PIPE_CLIENT_POINT_CLOUD_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                         sizeof(tof_data_t) * 10);
    }
    if (depth_pipe_3_enable){
        printf("waiting for server at %s\n", depth_pipe_3);
        pipe_client_open(MPA_DEPTH_3_CH, depth_pipe_3, PROCESS_NAME,
                         EN_PIPE_CLIENT_POINT_CLOUD_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                         sizeof(tof_data_t) * 10);
    }

    printf("Initializing ESDF structs\n");

    voxblox::EsdfIntegrator::Config esdf_int_config;
    esdf_int_config.inner_sphere_radius = esdf_inner_sphere_radius;
    esdf_int_config.outer_sphere_radius = esdf_outer_sphere_radius;
    esdf_int_config.min_distance_m = esdf_min_distance;
    esdf_int_config.max_distance_m = esdf_max_distance;
    esdf_int_config.default_distance_m = esdf_default_distance;

    voxblox::EsdfMap::Config esdf_map_config;
    esdf_map_config.esdf_voxel_size = voxel_size;
    esdf_map_config.esdf_voxels_per_side = voxels_per_side;

    esdf_map_.reset(new EsdfMap(esdf_map_config));
    esdf_integrator_.reset(new EsdfIntegrator(esdf_int_config, tsdf_map_->getTsdfLayerPtr(), esdf_map_->getEsdfLayerPtr()));

    keep_updating = true;
    visual_updates_thread = std::thread(&TsdfServer::visual_updates_thread_worker, this);

    return 0;
}

void TsdfServer::closeMPA(){
    printf("Closing Pipes\n");
    keep_updating = false;
    if (visual_updates_thread.joinable()) visual_updates_thread.join();
    pipe_client_close_all();
    pipe_server_close_all();
}

void TsdfServer::integratePointcloud(const Transformation &T_G_C, const Pointcloud &ptcloud_C, const Colors &colors,
                                     const bool is_freespace_pointcloud){
    if (ptcloud_C.size() != colors.size()){
        fprintf(stderr, "Pointcloud and Colors are not of the same size: %s\n", __FUNCTION__);
    }
    tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors, is_freespace_pointcloud);
}

void TsdfServer::publish2DCostmap()
{
    pthread_mutex_lock(&pose_mutex); // lock pose mutex, get last fully integrated pose
    float height = curr_pose.z();
    pthread_mutex_unlock(&pose_mutex); // return mutex lock
    if (planning) return; //|| keep_checking) return;

    if (en_debug) printf("Generating CostMap\n");
    uint64_t start_time = rc_nanos_monotonic_time();
    create2DCostmap(esdf_map_->getEsdfLayer(), height, 0.20, cost_map, costmap_updates_only);
    uint64_t end_time = rc_nanos_monotonic_time();
    if (en_timing){
        printf("Generating CostMap Took: %0.1f ms\n", (end_time - start_time) / 1000000.0);
    }

    costmap_updates_only = true;

    std::vector<point_xyz_i> cost_map_ptc;

    for (auto it: cost_map){
        point_xyz_i pt;
        pt.x = it.first.first;
        pt.y = it.first.second;
        pt.z = 0.0;
        pt.intensity = it.second;
        cost_map_ptc.push_back(pt);
    }

    point_xyz_i pt_;
    pthread_mutex_lock(&pose_mutex); // lock pose mutex, get last fully integrated pose
    pt_.x = curr_pose.x();
    pt_.y = curr_pose.y();
    pthread_mutex_unlock(&pose_mutex); // return mutex lock
    pt_.z = 0.0;
    pt_.intensity = 0;
    cost_map_ptc.push_back(pt_);

    // intensity ptcloud for occupied (gradient)
    point_cloud_metadata_t costmap_meta;
    costmap_meta.magic_number = POINT_CLOUD_MAGIC_NUMBER;
    costmap_meta.timestamp_ns = rc_nanos_monotonic_time();
    costmap_meta.n_points = cost_map_ptc.size();
    costmap_meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZC;

    pipe_server_write_point_cloud(COSTMAP_CH, costmap_meta, cost_map_ptc.data());
}

void TsdfServer::updateEsdf(bool clear_updated_flag)
{
    if (planning) return;
    if (en_debug) printf("Updating ESDF Map\n");
    uint64_t start_time = rc_nanos_monotonic_time();
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
    uint64_t end_time = rc_nanos_monotonic_time();
    if (en_timing) printf("Updating ESDF Map Took: %0.1f ms\n", (end_time - start_time) / 1000000.0);
}

void TsdfServer::updateMesh()
{
    if (planning) return;// || keep_checking) return;
    if (en_debug) printf("Updating mesh\n");

    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;

    uint64_t start_time = rc_nanos_monotonic_time();
    mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
    uint64_t end_time = rc_nanos_monotonic_time();

    if (en_timing) printf("Generating Mesh Took: %0.1f ms\n", (end_time - start_time) / 1000000.0);

    //Send out through MPA
    voxblox_msgs::Mesh mesh_msg;
    generateVoxbloxMeshMsg(MESH_CH, mesh_layer_.get(), &mesh_msg);
}

bool TsdfServer::saveMesh(){
    updateMesh();

    if (!str_mesh_save_path.empty()){
        const bool success = outputMeshLayerAsPly(mesh_save_path, *mesh_layer_);
        if (success){
            printf("Output file as PLY: %s", str_mesh_save_path.c_str());
        }
        else{
            printf("Failed to output mesh as PLY: %s", str_mesh_save_path.c_str());
        }
        return success;
    }
    return false;
}

void TsdfServer::visual_updates_thread_worker(){
    static int64_t start_time;
    while(keep_updating){
        if (!planning){
            start_time = rc_nanos_monotonic_time();
            updateMesh();
            // ***WARN***
            // cannot update the esdf map when it is being used for collision checking in the planner
            // after a path plan completes, updating the esdf map will take SIGNIFICANTLY longer after
            // multiple skipped updates
            // **********
            updateEsdf(true);
            publish2DCostmap();
        }
        // update at most every two seconds for now
        if ((rc_nanos_monotonic_time() - start_time)/1000 < 2000000)
            usleep(2000000 - ((rc_nanos_monotonic_time() - start_time)/1000));
    }
}

void TsdfServer::clear(){
    planning = true;
    usleep(250000);
    tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
    esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
    mesh_layer_->clear();
    planning = false;
    cost_map.clear();
    costmap_updates_only = false;
}

// control listens for reset commands
void TsdfServer::_control_pipe_cb(__attribute__((unused)) int ch, char* string, \
							 int bytes, __attribute__((unused)) void* context)
{
    TsdfServer *server = (TsdfServer *)context;
	// remove the trailing newline from echo
	if(bytes>1 && string[bytes-1]=='\n'){
		string[bytes-1]=0;
	}
	if(strcmp(string, PLAN_HOME)==0){
		printf("Client requested plan home.\n");
        server->planning = true;
        Eigen::Vector3d start_pose, goal_pose;

        pthread_mutex_lock(&pose_mutex); // lock pose mutex, get last fully integrated pose

        start_pose << server->curr_pose.x(), server->curr_pose.y(), server->curr_pose.z();

        goal_pose << 0.0, 0.0, -1.5;

        server->esdf_integrator_->addNewRobotPosition(Point(start_pose.x(), start_pose.y(), start_pose.z()));
        usleep(125000);
        server->esdf_integrator_->addNewRobotPosition(Point(goal_pose.x(), goal_pose.y(), goal_pose.z()));

        server->updateEsdf(true);

        fprintf(stderr, "Using start pose of: x: %6.2f, y: %6.2f, z: %6.2f\n", start_pose.x(), start_pose.y(), start_pose.z());
        fprintf(stderr, "using goal pose of: x: %6.2f, y: %6.2f, z: %6.2f\n", goal_pose.x(), goal_pose.y(), goal_pose.z());

        pthread_mutex_unlock(&pose_mutex); // return mutex lock
        server->maiRRT(start_pose, goal_pose, server->getEsdfMapPtr(), &(server->path_to_follow));
        server->planning = false;
		return;
	}
	else if(strcmp(string, RESET_VIO)==0){
		printf("Client requested vio reset.\n");
        int fd = open(QVIO_SIMPLE_LOCATION "control", O_WRONLY);
        if(fd<0){
    		fprintf(stderr, "make sure voxl-qvio-server is running\n");
    	    return;
        }

        int ret = write(fd, RESET_VIO_HARD, strlen(RESET_VIO_HARD)+1);
        if (ret <=0) fprintf(stderr, "failed to write to control pipe\n");
        close(fd);
		return;
	}
	else if(strncmp(string, SAVE_MAP, 8)==0){
		printf("Client requested save map.\n");
        server->updateMesh();

        char* f_name;
        f_name = strtok (string, ":");
        f_name = strtok (NULL, ":");

        // if msg includes the filepath
        if (f_name != NULL){
            std::string esdf_path = "/data/voxl_mapper/esdf_";
            std::string tsdf_path = "/data/voxl_mapper/tsdf_";

            esdf_path.append(f_name);
            tsdf_path.append(f_name);

            remove(esdf_path.begin(), esdf_path.end(), ' ');
            remove(tsdf_path.begin(), tsdf_path.end(), ' ');

            esdf_path.pop_back();
            tsdf_path.pop_back();

            if (server->en_debug){
                fprintf(stderr, "full file name: %s\n", esdf_path.c_str());
                fprintf(stderr, "full file name: %s\n", tsdf_path.c_str());
            }

            server->saveMap(tsdf_path, esdf_path);
        }
        // using default paths
        else server->saveMap(server->str_tsdf_save_path, server->str_esdf_save_path);
        return;
	}
	else if(strncmp(string, LOAD_MAP, 8)==0){
		printf("Client requested load map.\n");

        char* f_name;
        f_name = strtok (string, ":");
        f_name = strtok (NULL, ":");

        // if msg includes the filepath
        if (f_name != NULL){
            std::string esdf_path = "/data/voxl_mapper/esdf_";
            std::string tsdf_path = "/data/voxl_mapper/tsdf_";

            esdf_path.append(f_name);
            tsdf_path.append(f_name);

            remove(esdf_path.begin(), esdf_path.end(), ' ');
            remove(tsdf_path.begin(), tsdf_path.end(), ' ');

            esdf_path.pop_back();
            tsdf_path.pop_back();

            if (server->en_debug){
                fprintf(stderr, "full file name: %s\n", esdf_path.c_str());
                fprintf(stderr, "full file name: %s\n", tsdf_path.c_str());
            }

            server->loadMap(tsdf_path, esdf_path);
        }
        // using default paths
        else server->loadMap(server->str_tsdf_save_path, server->str_esdf_save_path);
		return;
	}
	else if(strcmp(string, CLEAR_MAP)==0){
		printf("Client requested clear map.\n");
	    server->clear();
		return;
	}
    else if(strncmp(string, "plan_to", 7)==0){
        server->planning = true;

		printf("Client requested plan to location\n");

        Eigen::Vector3d start_pose, goal_pose;

        pthread_mutex_lock(&pose_mutex); // lock pose mutex, get last fully integrated pose

        start_pose << server->curr_pose.x(), server->curr_pose.y(), server->curr_pose.z();

        char* goal_ptr;
        goal_ptr = strtok (string, ":");
        goal_ptr = strtok (NULL, ":");

        std::string goal_str(goal_ptr);
        std::string::size_type sz;     // alias of size_t

        double x = std::stod (goal_str,&sz);
        double y = std::stod (goal_str.substr(sz+1));
        double z = start_pose.z();

        goal_pose << x, y, z;

        server->esdf_integrator_->addNewRobotPosition(Point(start_pose.x(), start_pose.y(), start_pose.z()));
        usleep(125000);
        server->esdf_integrator_->addNewRobotPosition(Point(goal_pose.x(), goal_pose.y(), goal_pose.z()));

        server->updateEsdf(true);

        fprintf(stderr, "Using start pose of: x: %6.2f, y: %6.2f, z: %6.2f\n", start_pose.x(), start_pose.y(), start_pose.z());
        fprintf(stderr, "using goal pose of: x: %6.2f, y: %6.2f, z: %6.2f\n", goal_pose.x(), goal_pose.y(), goal_pose.z());

        pthread_mutex_unlock(&pose_mutex); // return mutex lock
        server->maiRRT( start_pose, goal_pose, server->getEsdfMapPtr(), &(server->path_to_follow));

        server->planning = false;
		return;
	}
	else if(strcmp(string, FOLLOW_PATH)==0){
        fprintf(stderr, "Client requested to follow last path\n");
        server->followPath();
        return;
    }
    else if (server->en_debug){
        printf("WARNING: Server received unknown command through the control pipe!\n");
	    printf("got %d bytes. Command is: %s\n", bytes, string);
    }

	return;
}

bool TsdfServer::maiRRT(Eigen::Vector3d start_pose, Eigen::Vector3d goal_pose, std::shared_ptr<EsdfMap> esdf_map_ptr, mav_trajectory_generation::Trajectory* path_to_follow){
    RRTSTAR* path_gen = new RRTSTAR(start_pose, goal_pose, esdf_map_ptr);

    if (en_debug) fprintf(stderr, "STARTING SOLVE\n");

    path_gen->Solve(path_to_follow);

    esdf_map_ptr.reset();

    std::vector<point_xyz> raw_waypoints;
    for (int i = 0; i < path_gen->path.size(); i++){
        point_xyz pt;
        pt.x = path_gen->path[i]->position.x();
        pt.y = path_gen->path[i]->position.y();
        pt.z = path_gen->path[i]->position.z();
        raw_waypoints.push_back(pt);
    }

    std::vector<point_xyz> ptcloud_loco;
    for (const auto& trajectory_point : path_gen->loco_path) {
        point_xyz pt;
        pt.x = trajectory_point.position_W.x();
        pt.y = trajectory_point.position_W.y();
        pt.z = trajectory_point.position_W.z();
        ptcloud_loco.push_back(pt);
    }

    // *NOTE* using ts as format to specify type of path for voxl-portal
    // 0 - raw waypoints
    // 2 - loco smoothed path
    // 6 - rrt star tree as building (not sent here)

    point_cloud_metadata_t waypoints_meta;
    waypoints_meta.magic_number = POINT_CLOUD_MAGIC_NUMBER;
    waypoints_meta.n_points = raw_waypoints.size();
    waypoints_meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZ;
    waypoints_meta.timestamp_ns = 0;

    if (waypoints_meta.n_points != 0) pipe_server_write_point_cloud(RENDER_CH, waypoints_meta, raw_waypoints.data());

    waypoints_meta.n_points = ptcloud_loco.size();
    waypoints_meta.timestamp_ns = 1;
    if (waypoints_meta.n_points != 0) pipe_server_write_point_cloud(RENDER_CH, waypoints_meta, ptcloud_loco.data());

    int ret = path_gen->reached();
    delete path_gen;
    path_gen = NULL;

    return ret;
}

bool TsdfServer::followPath()
{
    mav_planning_msgs::PolynomialTrajectory4D msg;

    if (!mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
        path_to_follow, &msg)){
            fprintf(stderr, "CONVERSION FAILED\n");
            return false;
    }

    trajectory_t out;
    if (msg.segments.size() > TRAJ_MAX_SEGMENTS){
        printf("Path segments too long\n");
        return false;
    }
    out.magic_number = TRAJECTORY_MAGIC_NUMBER;
    out.creation_time_ns = rc_nanos_monotonic_time();
    out.n_segments = msg.segments.size();

    for(int i = 0; i < msg.segments.size(); i++){
        if (msg.segments[i].num_coeffs > TRAJ_MAX_COEFFICIENTS){
            printf("Path coefficients too large\n");
            return false;
        }
        out.segments[i].n_coef = msg.segments[i].num_coeffs;
        out.segments[i].duration_s = msg.segments[i].segment_time / 1000000000.0;
        if (en_debug) fprintf(stderr, "duration of segment %d: %6.5f\n", i, out.segments[i].duration_s);

        for(int j = 0; j < out.segments[i].n_coef; j++){
            out.segments[i].cx[j] = msg.segments[i].x[j];
            out.segments[i].cy[j] = msg.segments[i].y[j];
            out.segments[i].cz[j] = msg.segments[i].z[j];
            if (en_debug) fprintf(stderr, "segment %d-> x: %6.5f, y: %6.5f, z: %6.5f\n", i, out.segments[i].cx[j], out.segments[i].cy[j], out.segments[i].cz[j]);
        }
    }
    printf("sending trajectory to plan channel\n");
    pipe_server_write(PLAN_CH, (char*)&out, sizeof(trajectory_t));
    return true;
}

bool TsdfServer::saveMap(std::string tsdf_path, std::string esdf_path)
{
    bool tsdf_saved = tsdf_map_->getTsdfLayer().saveToFile(tsdf_path, true);
    bool esdf_saved = esdf_map_->getEsdfLayer().saveToFile(esdf_path, true);
    return (tsdf_saved && esdf_saved);
}

bool TsdfServer::loadMap(std::string tsdf_path, std::string esdf_path)
{
    planning = true;
    constexpr bool mult_layer_support = true;
    bool tsdf_loaded = LoadBlocksFromFile(
        tsdf_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
        mult_layer_support, tsdf_map_->getTsdfLayerPtr());
    if (tsdf_loaded){
        printf("Successfully loaded TSDF layer.\n");
    }
    else return false;

    bool esdf_loaded = LoadBlocksFromFile(
        esdf_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
        mult_layer_support, esdf_map_->getEsdfLayerPtr());
    if (esdf_loaded){
        printf("Successfully loaded ESDF layer.\n");
    }
    costmap_updates_only = false;
    // updateEsdf(false);
    usleep(250000);
    planning = false;

    return (tsdf_loaded && esdf_loaded);
}

} // namespace voxblox
