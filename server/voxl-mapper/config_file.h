/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/


#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include "rc_transform.h"
#include <voxl_common_config.h>
#include <string>


#define CONF_FILE "/etc/modalai/voxl-mapper.conf"
#define BUF_LEN 64

// INPUTS
extern char tof_pipe[BUF_LEN];
extern bool tof_enable;
extern rc_tf_t tf_tof_wrt_body;
extern float tof_rate;

extern char depth_pipe_0[BUF_LEN];
extern bool depth_pipe_0_enable;
extern rc_tf_t tf_depth0_wrt_body;
extern char extrinsics0_name[BUF_LEN];
extern float depth0_rate;

extern char depth_pipe_1[BUF_LEN];
extern bool depth_pipe_1_enable;
extern rc_tf_t tf_depth1_wrt_body;
extern char extrinsics1_name[BUF_LEN];
extern float depth1_rate;

extern char depth_pipe_2[BUF_LEN];
extern bool depth_pipe_2_enable;
extern rc_tf_t tf_depth2_wrt_body;
extern char extrinsics2_name[BUF_LEN];
extern float depth2_rate;

extern char depth_pipe_3[BUF_LEN];
extern bool depth_pipe_3_enable;
extern rc_tf_t tf_depth3_wrt_body;
extern char extrinsics3_name[BUF_LEN];
extern float depth3_rate;


// *ALL DISTANCES/SIZES ARE IN METERS* //
extern float robot_radius;
extern float voxel_size;
extern int voxels_per_side;
extern char esdf_save_path[BUF_LEN];
extern char tsdf_save_path[BUF_LEN];
extern char mesh_save_path[BUF_LEN];

extern float esdf_max_distance;
extern float esdf_min_distance;
extern float esdf_default_distance;
extern float esdf_inner_sphere_radius;
extern float esdf_outer_sphere_radius;

extern float rrt_min_distance;
extern double rrt_max_runtime_nanoseconds;
extern double rrt_goal_threshold;
extern int rrt_prune_iterations;
extern int rrt_send_tree;
extern bool treat_unknown_as_occupied;

extern int loco_num_segments;
extern double loco_smoothness_cost_weight;
extern double loco_collision_cost_weight;
extern double loco_waypoint_cost_weight;
extern double loco_min_collision_sampling_dist;
extern bool loco_add_waypoints;
extern bool loco_scale_time;
extern bool loco_split_at_collisions;
extern bool loco_resample_trajectory;
extern bool loco_resample_visibility;
extern bool loco_verbose;

extern bool loco_optimize_time;
extern double loco_v_max; 
extern double loco_a_max; 
extern double loco_yaw_rate_max;
extern double loco_sampling_dt;

// read only our own config file without printing the contents
int config_file_read(void);

// prints the current configuration values to the screen.
int config_file_print(void);

// load the common extrinsics config files
// prints cam_wrt_body matrix if debug=true
int load_extrinsics_file(bool debug);

#endif // end #define CONFIG_FILE_H