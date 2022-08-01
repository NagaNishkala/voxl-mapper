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

#include <modal_json.h>
#include <stdio.h>
#include "config_file.h"


#define CONFIG_FILE_HEADER "\
/**\n\
 * This file contains configuration that's specific to voxl-mapper.\n\
 * \n\
 * RRT Parameters\n\
 * rrt_min_distance:           			  Minimum edge distance in the RRT. Should be larger than voxel size otherwise\n\
 * 										  there will be wasted computation. Default: 0.2 \n\
 * rrt_max_runtime_nanoseconds:           Maximum amount of time allowed for the RRT planner to run. Default: 1000000000 \n\
 * rrt_goal_threshold:                    Minimum distance to goal before a point is considered to be at the goal. Default: 0.05m \n\
 * rrt_prune_iterations:                  Number of iterations to run pruning on the RRT path for. Default: 100 \n\
 * rrt_send_tree:		                  Whether to send the RRT tree for visualization in portal. Default: false \n\
 * treat_unknown_as_occupied:         	  Whether to treat unknown cells as occupied. Default: true \n\
 * \n\
 * Smoother Parameters\n\
 * loco_resample_trajectory: 	          If true will take the initial guess from the linear solver and resample the\n\
 *                                        trajectory to get a new trajectory with num_segments in it that is then\n\
 *                                        passed to the nonlinear solver. Default: true \n\
 * loco_resample_visibility:              If true will resample before running the linear solver. Uses the visiblity \n\
 *                                        graph and a time estimation of the entire path to resample points. the \n\
 *                                        visibility graph is essentially the graph made up of the waypoints passed to \n\
 *                                        the smoother. Default: false\n\
 * loco_num_segments:                     The number of segments that the resampled trajectory will have (only applies \n\
 *                                        if resample_trajectory is true. Default: 5\n\
 * loco_add_waypoints:                    Adds waypoints into the nonlinear smoother to optimize passing through each \n\
 *                                        waypoint. If disabled then waypoint cost weight has no effect. Default: false\n\
 * loco_scale_time:                       Scales the segment times evenly to ensure that the trajectory is feasible \n\
 *                                        given the provided v_max and a_max. Does not change the shape of the trajectory,\n\
 *                                        and only increases segment times. Default: true\n\
 * loco_optimize_time:                    Runs an additional optimization step (using nlopt) to optimize the segment \n\
 *                                        times in order to better meet the dynamic constraints. Default: true\n\
 * loco_split_at_collisions:              Adds additional points to the trajectory if any portion of the initial linear \n\
 *                                        solvers trajectory is in collision. Default: true\n\
 * loco_v_max:                            Max velocity of robot. Default: 1.0\n\
 * loco_a_max:                            Max acceleration of robot. Default: 2.0\n\
 * loco_yaw_rate_max:                     Max yaw rate of robot. Default: PI/4\n\
 * loco_sampling_dt:                      Time step delta at which to sample points from the trajectory to check for collisions\n\
 *                                        this is used in the linear solver and generally only when split at collisions is true\n\
 *                                        Default: 0.1\n\
 * loco_smoothness_cost_weight:           Weighting for smoothness of derivative we are optimizing for. Default: 0.1\n\
 * loco_collision_cost_weight:            Weighting for collisions. Default: 10.0\n\
 * loco_waypoint_cost_weight:             Weighting for waypoints (has no effect if add_waypoints_ is false). Default: 1.0\n\
 * loco_min_collision_sampling_dist:      Time step delta at which to evaluate cost/gradient for collisions. Default: 0.1\n\
 * loco_verbose:                          Whether to print debug statements or not. Default: false\n\
 */\n"


// define all the "Extern" variables from config_file.h
char tof_pipe[BUF_LEN];
bool tof_enable;
rc_tf_t tf_tof_wrt_body;
float tof_rate;

char depth_pipe_0[BUF_LEN];
bool depth_pipe_0_enable;
rc_tf_t tf_depth0_wrt_body;
char extrinsics0_name[BUF_LEN];
float depth0_rate;

char depth_pipe_1[BUF_LEN];
bool depth_pipe_1_enable;
rc_tf_t tf_depth1_wrt_body;
char extrinsics1_name[BUF_LEN];
float depth1_rate;

char depth_pipe_2[BUF_LEN];
bool depth_pipe_2_enable;
rc_tf_t tf_depth2_wrt_body;
char extrinsics2_name[BUF_LEN];
float depth2_rate;

char depth_pipe_3[BUF_LEN];
bool depth_pipe_3_enable;
rc_tf_t tf_depth3_wrt_body;
char extrinsics3_name[BUF_LEN];
float depth3_rate;

float robot_radius;
float voxel_size;
int voxels_per_side;
char esdf_save_path[BUF_LEN];
char tsdf_save_path[BUF_LEN];
char mesh_save_path[BUF_LEN];

float esdf_max_distance;
float esdf_min_distance;
float esdf_default_distance;
float esdf_inner_sphere_radius;
float esdf_outer_sphere_radius;

float rrt_min_distance;
double rrt_max_runtime_nanoseconds;
double rrt_goal_threshold;
int rrt_prune_iterations;
int rrt_send_tree;
bool treat_unknown_as_occupied;

bool loco_resample_trajectory;
bool loco_resample_visibility;
int loco_num_segments;
bool loco_add_waypoints;
bool loco_scale_time;
bool loco_optimize_time;
bool loco_split_at_collisions;
double loco_v_max;
double loco_a_max;
double loco_yaw_rate_max;
double loco_sampling_dt;

double loco_smoothness_cost_weight;
double loco_collision_cost_weight;
double loco_waypoint_cost_weight;
double loco_min_collision_sampling_dist;
bool loco_verbose;


int load_extrinsics_file(bool debug)
{
	vcc_extrinsic_t t[VCC_MAX_EXTRINSICS_IN_CONFIG];
	vcc_extrinsic_t tmp;

	// now load in extrinsics
	int n_extrinsics;
	if(vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, t, &n_extrinsics, VCC_MAX_EXTRINSICS_IN_CONFIG)){
		fprintf(stderr, "ERROR: Unable to read extrinsics conf at %s\n", VCC_EXTRINSICS_PATH);
		return -1;
	}

	// Pick out Cam to Body. Can be stereo_l or tof
	if (tof_enable){
		if(vcc_find_extrinsic_in_array("body", "tof", t, n_extrinsics, &tmp)){
			fprintf(stderr, "ERROR: %s missing tof to body transform\n", VCC_EXTRINSICS_PATH);
			return -1;
		}
		if (debug) printf("tof_wrt_body:\n");
		for(int j=0; j<3; j++){
			for(int k=0; k<4; k++){
				if (k==3) tf_tof_wrt_body.d[j][k] = tmp.T_child_wrt_parent[j];
				else tf_tof_wrt_body.d[j][k] = tmp.R_child_to_parent[j][k];
				if (debug) printf("%6.3f ", tf_tof_wrt_body.d[j][k]);
			}
			if (debug) printf("\n");
		}
		if (debug) printf("\n");
	}
	if (depth_pipe_0_enable){
		if (extrinsics0_name == NULL){
			fprintf(stderr, "extrinsics0 is empty\n");
			return -1;
		}
		if(vcc_find_extrinsic_in_array("body", extrinsics0_name, t, n_extrinsics, &tmp)){
			fprintf(stderr, "ERROR: %s missing %s to body transform\n", extrinsics0_name, VCC_EXTRINSICS_PATH);
			return -1;
		}
		if (debug) printf("%s_wrt_body:\n", extrinsics0_name);
		for(int j=0; j<3; j++){
			for(int k=0; k<4; k++){
				if (k==3) tf_depth0_wrt_body.d[j][k] = tmp.T_child_wrt_parent[j];
				else tf_depth0_wrt_body.d[j][k] = tmp.R_child_to_parent[j][k];
				if (debug) printf("%6.3f ", tf_depth0_wrt_body.d[j][k]);
			}
			if (debug) printf("\n");
		}
		if (debug) printf("\n");
	}
	if (depth_pipe_1_enable){
		if (extrinsics1_name == NULL){
			fprintf(stderr, "extrinsics1 is empty\n");
			return -1;
		}
		if(vcc_find_extrinsic_in_array("body", extrinsics1_name, t, n_extrinsics, &tmp)){
			fprintf(stderr, "ERROR: %s missing %s to body transform\n", extrinsics1_name, VCC_EXTRINSICS_PATH);
			return -1;
		}
		if (debug) printf("%s_wrt_body:\n", extrinsics1_name);
		for(int j=0; j<3; j++){
			for(int k=0; k<4; k++){
				if (k==3) tf_depth1_wrt_body.d[j][k] = tmp.T_child_wrt_parent[j];
				else tf_depth1_wrt_body.d[j][k] = tmp.R_child_to_parent[j][k];
				if (debug) printf("%6.3f ", tf_depth1_wrt_body.d[j][k]);
			}
			if (debug) printf("\n");
		}
		if (debug) printf("\n");
	}
	if (depth_pipe_2_enable){
		if (extrinsics2_name == NULL){
			fprintf(stderr, "extrinsics0 is empty\n");
			return -1;
		}
		if(vcc_find_extrinsic_in_array("body", extrinsics2_name, t, n_extrinsics, &tmp)){
			fprintf(stderr, "ERROR: %s missing %s to body transform\n", extrinsics2_name, VCC_EXTRINSICS_PATH);
			return -1;
		}
		if (debug) printf("%s_wrt_body:\n", extrinsics2_name);
		for(int j=0; j<3; j++){
			for(int k=0; k<4; k++){
				if (k==3) tf_depth2_wrt_body.d[j][k] = tmp.T_child_wrt_parent[j];
				else tf_depth2_wrt_body.d[j][k] = tmp.R_child_to_parent[j][k];
				if (debug) printf("%6.3f ", tf_depth2_wrt_body.d[j][k]);
			}
			if (debug) printf("\n");
		}
		if (debug) printf("\n");
	}
	if (depth_pipe_3_enable){
		if (extrinsics0_name == NULL){
			fprintf(stderr, "extrinsics0 is empty\n");
			return -1;
		}
		if(vcc_find_extrinsic_in_array("body", extrinsics3_name, t, n_extrinsics, &tmp)){
			fprintf(stderr, "ERROR: %s missing %s to body transform\n", extrinsics3_name, VCC_EXTRINSICS_PATH);
			return -1;
		}
		if (debug) printf("%s_wrt_body:\n", extrinsics3_name);
		for(int j=0; j<3; j++){
			for(int k=0; k<4; k++){
				if (k==3) tf_depth3_wrt_body.d[j][k] = tmp.T_child_wrt_parent[j];
				else tf_depth3_wrt_body.d[j][k] = tmp.R_child_to_parent[j][k];
				if (debug) printf("%6.3f ", tf_depth3_wrt_body.d[j][k]);
			}
			if (debug) printf("\n");
		}
		if (debug) printf("\n");
	}

	return 0;
}

/**
 * @brief      prints the current configuration values to the screen
 *
 *             this includes all of the extern variables listed above. If this
 *             is called before config_file_load then it will print the default
 *             values.
 */
int config_file_print(void){
    printf("============================INPUTS===============================\n");
    printf("tof_pipe:                   	  %s\n", tof_pipe);
    printf("tof_enable:		                  %s\n", tof_enable == 0 ? "false" : "true");
    printf("tof_rate:             		      %0.3f\n", (double)tof_rate);

	printf("depth_pipe_0:                     %s\n", depth_pipe_0);
    printf("depth_pipe_0_enable:              %s\n", depth_pipe_0_enable  == 0 ? "false" : "true");
    printf("extrinsics0_name:                 %s\n", extrinsics0_name);
    printf("depth0_rate:             		  %0.3f\n", (double)depth0_rate);

	printf("depth_pipe_1:                     %s\n", depth_pipe_1);
    printf("depth_pipe_1_enable:              %s\n", depth_pipe_1_enable  == 0 ? "false" : "true");
    printf("extrinsics1_name:                 %s\n", extrinsics1_name);
    printf("depth1_rate:             		  %0.3f\n", (double)depth1_rate);

	printf("depth_pipe_2:                     %s\n", depth_pipe_2);
    printf("depth_pipe_2_enable:              %s\n", depth_pipe_2_enable  == 0 ? "false" : "true");
    printf("extrinsics2_name:                 %s\n", extrinsics2_name);
    printf("depth2_rate:             		  %0.3f\n", (double)depth2_rate);

	printf("depth_pipe_3:                     %s\n", depth_pipe_3);
    printf("depth_pipe_3_enable:              %s\n", depth_pipe_3_enable  == 0 ? "false" : "true");
    printf("extrinsics3_name:                 %s\n", extrinsics3_name);
    printf("depth3_rate:             		  %0.3f\n", (double)depth3_rate);

    printf("============================GENERAL==============================\n");
    printf("robot_radius:                     %0.3f\n", robot_radius);
    printf("voxel_size:                       %0.3f\n", (double)voxel_size);
    printf("voxels_per_side:                  %d\n", voxels_per_side);
    printf("esdf_save_path:                   %s\n", esdf_save_path);
    printf("tsdf_save_path:                   %s\n", tsdf_save_path);
    printf("mesh_save_path:                   %s\n", mesh_save_path);
    printf("============================ESDF=================================\n");
    printf("esdf_max_distance:                %0.3f\n", (double)esdf_max_distance);
    printf("esdf_min_distance:                %0.3f\n", (double)esdf_min_distance);
    printf("esdf_default_distance:            %0.3f\n", (double)esdf_default_distance);
    printf("esdf_inner_sphere_radius:         %0.3f\n", (double)esdf_inner_sphere_radius);
    printf("esdf_outer_sphere_radius:         %0.3f\n", (double)esdf_outer_sphere_radius);
    printf("============================RRT==================================\n");
    printf("rrt_min_distance:                 %0.3f\n", rrt_min_distance);
    printf("rrt_max_runtime_nanoseconds:      %0.3f\n", rrt_max_runtime_nanoseconds);
    printf("rrt_goal_threshold:		          %0.3f\n", rrt_goal_threshold);
    printf("rrt_prune_iterations:		      %d\n", rrt_prune_iterations);
    printf("rrt_send_tree:				      %s\n", rrt_send_tree ? "true" : "false");
    printf("treat_unknown_as_occupied:    	  %s\n", treat_unknown_as_occupied ? "true" : "false");
    printf("============================LOCO==================================\n");
    printf("loco_num_segments:                %d\n", loco_num_segments);
    printf("loco_smoothness_cost_weight:      %0.3f\n", loco_smoothness_cost_weight);
    printf("loco_collision_cost_weight:       %0.3f\n", loco_collision_cost_weight);
    printf("loco_waypoint_cost_weight:        %0.3f\n", loco_waypoint_cost_weight);
    printf("loco_min_collision_sampling_dist: %0.3f\n", loco_min_collision_sampling_dist);
    printf("loco_add_waypoints:               %s\n", loco_add_waypoints ? "true" : "false");
    printf("loco_scale_time:                  %s\n", loco_scale_time ? "true" : "false");
    printf("loco_split_at_collisions:         %s\n", loco_split_at_collisions ? "true" : "false");
    printf("loco_resample_trajectory:         %s\n", loco_resample_trajectory ? "true" : "false");
    printf("loco_resample_visibility:         %s\n", loco_resample_visibility ? "true" : "false");
    printf("loco_verbose:                     %s\n", loco_verbose ? "true" : "false");
    printf("loco_optimize_time:               %s\n", loco_optimize_time ? "true" : "false");
    printf("loco_v_max:                       %0.3f\n", loco_v_max);
    printf("loco_a_max:                       %0.3f\n", loco_a_max);
    printf("loco_yaw_rate_max:		          %0.3f\n", loco_yaw_rate_max);
    printf("loco_sampling_dt:                 %0.3f\n", loco_sampling_dt);
	printf("=================================================================\n\n");
	return 0;
}

/**
 * load the config file and populate the above extern variables
 *
 * @return     0 on success, -1 on failure
 */
int config_file_read(void)
{
	int ret = json_make_empty_file_with_header_if_missing(CONF_FILE, CONFIG_FILE_HEADER);
	if(ret < 0) return -1;
	else if(ret>0) fprintf(stderr, "Creating new config file: %s\n", CONF_FILE);

	cJSON* parent = json_read_file(CONF_FILE);
	if(parent==NULL) return -1;

	json_fetch_string_with_default(parent, "tof_pipe", tof_pipe, BUF_LEN, "/run/mpa/tof");
    json_fetch_bool_with_default(parent, "tof_enable", (int*)&tof_enable, 1);
	json_fetch_float_with_default(parent, "tof_rate", &tof_rate, 10.0);

	json_fetch_string_with_default(parent, "depth_pipe_0", depth_pipe_0, BUF_LEN, "/run/mpa/dfs_point_cloud");
    json_fetch_bool_with_default(parent, "depth_pipe_0_enable", (int*)&depth_pipe_0_enable, 0);
	json_fetch_string_with_default(parent, "extrinsics0_name", extrinsics0_name, BUF_LEN, "stereo_l");
	json_fetch_float_with_default(parent, "depth0_rate", &depth0_rate, 10.0);

	json_fetch_string_with_default(parent, "depth_pipe_1", depth_pipe_1, BUF_LEN, "/run/mpa/stereo_front_pc");
    json_fetch_bool_with_default(parent, "depth_pipe_1_enable", (int*)&depth_pipe_1_enable, 0);
	json_fetch_string_with_default(parent, "extrinsics1_name", extrinsics1_name, BUF_LEN, "stereo_front_l");
	json_fetch_float_with_default(parent, "depth1_rate", &depth1_rate, 10.0);

	json_fetch_string_with_default(parent, "depth_pipe_2", depth_pipe_2, BUF_LEN, "/run/mpa/stereo_rear_pc");
    json_fetch_bool_with_default(parent, "depth_pipe_2_enable", (int*)&depth_pipe_2_enable, 0);
	json_fetch_string_with_default(parent, "extrinsics2_name", extrinsics2_name, BUF_LEN, "stereo_rear_l");
	json_fetch_float_with_default(parent, "depth2_rate", &depth2_rate, 10.0);

	json_fetch_string_with_default(parent, "depth_pipe_3", depth_pipe_3, BUF_LEN, "/run/mpa/dfs_point_cloud");
    json_fetch_bool_with_default(parent, "depth_pipe_3_enable", (int*)&depth_pipe_3_enable, 0);
	json_fetch_string_with_default(parent, "extrinsics3_name", extrinsics3_name, BUF_LEN, "stereo_l");
	json_fetch_float_with_default(parent, "depth3_rate", &depth3_rate, 10.0);

	json_fetch_float_with_default(parent, "robot_radius", &robot_radius, 0.3);
	json_fetch_float_with_default(parent, "voxel_size", &voxel_size, 0.2);
    json_fetch_int_with_default(parent, "voxels_per_side", &voxels_per_side, 16u);
	json_fetch_string_with_default(parent, "esdf_save_path", esdf_save_path, BUF_LEN, "/data/voxl-mapper/esdf_map");
	json_fetch_string_with_default(parent, "tsdf_save_path", tsdf_save_path, BUF_LEN, "/data/voxl-mapper/tsdf_map");
	json_fetch_string_with_default(parent, "mesh_save_path", mesh_save_path, BUF_LEN, "/data/voxl-mapper/mesh.ply");

	json_fetch_float_with_default(parent, "esdf_max_distance", &esdf_max_distance, 4.0);
	json_fetch_float_with_default(parent, "esdf_min_distance", &esdf_min_distance, 0.1);
	json_fetch_float_with_default(parent, "esdf_default_distance", &esdf_default_distance, 2.0);
	json_fetch_float_with_default(parent, "esdf_inner_sphere_radius", &esdf_inner_sphere_radius, 0.2);
	json_fetch_float_with_default(parent, "esdf_outer_sphere_radius", &esdf_outer_sphere_radius, 0.6);

	json_fetch_float_with_default(parent, "rrt_min_distance", &rrt_min_distance, 0.2);
	json_fetch_double_with_default(parent, "rrt_max_runtime_nanoseconds", &rrt_max_runtime_nanoseconds, 1000000000);
	json_fetch_double_with_default(parent, "rrt_goal_threshold", &rrt_goal_threshold, 0.05);
	json_fetch_int_with_default(parent, "rrt_prune_iterations", &rrt_prune_iterations, 100);
	json_fetch_bool_with_default(parent, "rrt_send_tree", (int*)&rrt_send_tree, 0);
	json_fetch_bool_with_default(parent, "treat_unknown_as_occupied", (int*)&treat_unknown_as_occupied, 1);

    json_fetch_int_with_default(parent, "loco_num_segments", &loco_num_segments, 10);
	json_fetch_double_with_default(parent, "loco_smoothness_cost_weight", &loco_smoothness_cost_weight, 0.1);
	json_fetch_double_with_default(parent, "loco_collision_cost_weight", &loco_collision_cost_weight, 10.0);
	json_fetch_double_with_default(parent, "loco_waypoint_cost_weight", &loco_waypoint_cost_weight, 1.0);
	json_fetch_double_with_default(parent, "loco_min_collision_sampling_dist", &loco_min_collision_sampling_dist, 0.1);
	json_fetch_bool_with_default(parent, "loco_add_waypoints", (int*)&loco_add_waypoints, 0);
	json_fetch_bool_with_default(parent, "loco_scale_time", (int*)&loco_scale_time, 1);
	json_fetch_bool_with_default(parent, "loco_split_at_collisions", (int*)&loco_split_at_collisions, 1);
	json_fetch_bool_with_default(parent, "loco_resample_trajectory", (int*)&loco_resample_trajectory, 1);
	json_fetch_bool_with_default(parent, "loco_resample_visibility", (int*)&loco_resample_visibility, 0);
	json_fetch_bool_with_default(parent, "loco_verbose", (int*)&loco_verbose, 0);
	json_fetch_bool_with_default(parent, "loco_optimize_time", (int*)&loco_optimize_time, 1);
	json_fetch_double_with_default(parent, "loco_v_max", &loco_v_max, 1.0);
	json_fetch_double_with_default(parent, "loco_a_max", &loco_a_max, 2.0);
	json_fetch_double_with_default(parent, "loco_yaw_rate_max", &loco_yaw_rate_max, M_PI / 4.0);
	json_fetch_double_with_default(parent, "loco_sampling_dt", &loco_sampling_dt, 0.1);

	if(json_get_parse_error_flag()){
		fprintf(stderr, "failed to parse config file %s\n", CONF_FILE);
		cJSON_Delete(parent);
		return -1;
	}

	// write modified data to disk if neccessary
	if(json_get_modified_flag()){
		printf("The config file was modified during parsing, saving the changes to disk\n");
		json_write_to_file_with_header(CONF_FILE, parent, CONFIG_FILE_HEADER);
	}
	cJSON_Delete(parent);
	return 0;
}