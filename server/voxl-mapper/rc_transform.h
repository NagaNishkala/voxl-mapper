/**
 * rc_transform.h
 *
 *
 * @author     james@modalai.com
 * @date       2021
 */

#ifndef VOXL_MAPPER_RC_TRANSFORM_H_
#define VOXL_MAPPER_RC_TRANSFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdint.h>


/**
 * Transform with optional timestamp in nanoseconds. Set timestamp to -1
 * indicating it is static.
 *
 * The data is stored as a rotation matrix and translation in one 3x4 matrix in
 * row-major format for ease of multiplication with SIMD.
 *
 * In keeping consistent with the rest of the library, Translations are
 * considered the position of the child frame with respect to the parent frame,
 * e.g. T_child_wrt_parent. Rotation matrices rotate a vector from the child
 * frame to the parent frame, e.g. R_child_to_parent.
 */
typedef struct rc_tf_t {
	int64_t ts;		// timestamp in nanoseconds
	double d[3][4];	// Rotation and translation [R|T] in row-major format.
} rc_tf_t;



/**
 * initializer for trc_tf_t to make sure it starts zero'd out
 * timestamp = -1 indicates a static transform
 */
#define RC_TF_INITIALIZER {\
	.ts = -1,\
	.d  = {{1,0,0,0},{0,1,0,0},{0,0,1,0}}\
}


/**
 * timestamped transform with linear and angular velocity
 * linear velocity is in m/s
 * angular velocity is in rad/s
 */
typedef struct rc_tfv_t {
	rc_tf_t tf;
	double v[3];	// velocity m/s
	double w[3];	// XYZ angular rates ('gyro values') in rad/s
} rc_tfv_t;


/**
 * initializer for trc_tf_t to make sure it starts zero'd out
 * timestamp = -1 indicates a static transform
 */
#define RC_TFV_INITIALIZER {\
	.tf = RC_TF_INITIALIZER,\
	.v  = {NAN,NAN,NAN},\
	.w  = {NAN,NAN,NAN}\
}


/**
 * @brief      returns an empty rc_tf_t transform
 */
rc_tf_t rc_tf_empty(void);


/**
 * @brief      returns an empty rc_tfv_t transform with velocity
 */
rc_tfv_t rc_tfv_empty(void);


rc_tf_t rc_tf_from_tait_bryan_xyz_intrinsic_degrees(int64_t ts, double roll, double pitch, double yaw, double xyz[3]);
rc_tf_t rc_tf_from_tait_bryan_zyx_intrinsic_degrees(int64_t ts, double yaw, double pitch, double roll, double xyz[3]);

int rc_tf_print_matrix(rc_tf_t* tf);
int rc_tf_print_tait_bryan_xyz_intrinsic_degrees(rc_tf_t* tf);
int rc_tf_print_tait_bryan_zyx_intrinsic_degrees(rc_tf_t* tf);

int rc_tfv_print_matrix(rc_tfv_t* tfv);
int rc_tfv_print_tait_bryan_xyz_intrinsic_degrees(rc_tfv_t* tfv);
int rc_tfv_print_tait_bryan_zyx_intrinsic_degrees(rc_tfv_t* tfv);

int _rotation_matrix_to_tait_bryan_xyz_intrinsic_degrees(double R[3][4], double* roll, double* pitch, double* yaw);
int rc_tf_transform_vector(rc_tf_t* tf, double in[3], double out[3]);
int rc_tf_transform_vector_inplace(rc_tf_t* tf, double v[3]);
int rc_tf_invert(rc_tf_t* in, rc_tf_t* out);

int rc_tf_linear_interpolate(rc_tf_t* A, rc_tf_t* B, double h, rc_tf_t* out);
int rc_tf_cubic_interpolate(rc_tfv_t* A, rc_tfv_t* B, double h, rc_tf_t* out);

int rc_tf_combine_two(rc_tf_t C_wrt_B, rc_tf_t B_wrt_A, rc_tf_t* C_wrt_A);

int _rotation_to_quaternion(double R[3][4], double* q);



#ifdef __cplusplus
}
#endif

#endif