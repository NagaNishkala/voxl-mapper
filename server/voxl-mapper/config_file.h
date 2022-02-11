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

enum depth_modes {
    tof = 0,
    dfs
};

// *ALL DISTANCES/SIZES ARE IN METERS* //
extern int depth_mode;

extern double robot_radius;
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

extern double rrt_min_distance;
extern double rrt_max_runtime_nanoseconds;
extern bool rrt_use_first_solution;
extern bool rrt_treat_unknown_as_occupied;
extern bool rrt_send_tree;

extern int loco_num_segments;
extern int loco_derivative_to_optimize;
extern int loco_poly_degree;
extern double loco_smoothness_cost_weight;
extern double loco_collision_cost_weight;
extern double loco_waypoint_cost_weight;
extern double loco_min_collision_sampling_dist;
extern bool loco_add_waypoints;
extern bool loco_scale_time;
extern bool loco_split_at_collisions;
extern bool loco_resample_trajectory;
extern bool loco_verbose;

extern rc_tf_t tf_cam_wrt_body;

// read only our own config file without printing the contents
int config_file_read(void);

// prints the current configuration values to the screen.
int config_file_print(void);

// load the common extrinsics config files
// prints cam_wrt_body matrix if debug=true
int load_extrinsics_file(bool debug, depth_modes dmode);

#endif // end #define CONFIG_FILE_H
