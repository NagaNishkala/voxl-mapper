/**
 * rc_transform.c
 *
 *
 * @author     james@modalai.com
 * @date       2021
 */

#include <stdio.h>
#include <string.h>
#include "rc_transform.h"


#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI/180.0)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0/M_PI)
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI/2.0)
#endif




rc_tf_t rc_tf_empty(void)
{
	rc_tf_t new = RC_TF_INITIALIZER;
	return new;
}

rc_tfv_t rc_tfv_empty(void)
{
	rc_tfv_t new = RC_TFV_INITIALIZER;
	return new;
}



int rc_tf_print_matrix(rc_tf_t* tf)
{
	int i;
	for(i=0;i<3;i++){
		printf("%6.3f %6.3f %6.3f | %6.3f\n", tf->d[i][0],tf->d[i][1],tf->d[i][2],tf->d[i][3]);
	}
	return 0;
}


int _rotation_matrix_to_tait_bryan_xyz_intrinsic_degrees(double R[3][4], double* roll, double* pitch, double* yaw)
{
	*pitch = asin(R[0][2])*RAD_TO_DEG;
	if(fabs(R[0][2])<0.9999999){
		*roll = atan2(-R[1][2], R[2][2])*RAD_TO_DEG;
		*yaw  = atan2(-R[0][1], R[0][0])*RAD_TO_DEG;
	}
	else{
		*roll = atan2(-R[2][1], R[1][1])*RAD_TO_DEG;
		*yaw  = 0.0;
	}
	return 0;
}



static int _rotation_matrix_to_tait_bryan_zyx_intrinsic_degrees(double R[3][4], double* yaw, double* pitch, double* roll)
{
	*pitch = asin(-R[2][0])*RAD_TO_DEG;
	if(fabs(R[2][0])<0.9999999){
		*roll = atan2(R[2][1], R[2][2])*RAD_TO_DEG;
		*yaw  = atan2(R[1][0], R[0][0])*RAD_TO_DEG;
	}
	else{
		*roll = 0.0;
		*yaw  = atan2(-R[0][1], R[1][1])*RAD_TO_DEG;
	}
	return 0;
}



static int _tait_bryan_xyz_intrinsic_degrees_to_rotation_matrix(double roll, double pitch, double yaw, double R[3][4])
{
	const double r_rad = roll*DEG_TO_RAD;
	const double cx = cos(r_rad);
	const double sx = sin(r_rad);
	const double p_rad = pitch*DEG_TO_RAD;
	const double cy = cos(p_rad);
	const double sy = sin(p_rad);
	const double y_rad = yaw*DEG_TO_RAD;
	const double cz = cos(y_rad);
	const double sz = sin(y_rad);

	const double cxcz = cx * cz;
	const double cxsz = cx * sz;
	const double sxcz = sx * cz;
	const double sxsz = sx * sz;

	R[0][0] =  cy * cz;
	R[0][1] = -cy * sz;
	R[0][2] =  sy;
	R[1][0] =  cxsz + (sxcz * sy);
	R[1][1] =  cxcz - (sxsz * sy);
	R[1][2] = -sx * cy;
	R[2][0] =  sxsz - (cxcz * sy);
	R[2][1] =  sxcz + (cxsz * sy);
	R[2][2] =  cx * cy;

	return 0;
}



static int _tait_bryan_zyx_intrinsic_degrees_to_rotation_matrix(double yaw, double pitch, double roll, double R[3][4])
{
	const double y_rad = yaw*DEG_TO_RAD;
	const double c1 = cos(y_rad);
	const double s1 = sin(y_rad);
	const double p_rad = pitch*DEG_TO_RAD;
	const double c2 = cos(p_rad);
	const double s2 = sin(p_rad);
	const double r_rad = roll*DEG_TO_RAD;
	const double c3 = cos(r_rad);
	const double s3 = sin(r_rad);

	const double c1c3 = c1 * c3;
	const double c1s3 = c1 * s3;
	const double s1c3 = s1 * c3;
	const double s1s3 = s1 * s3;

	R[0][0] = c1*c2;
	R[0][1] = (c1s3*s2) - s1c3;
	R[0][2] = s1s3 + (c1c3*s2);
	R[1][0] = c2*s1;
	R[1][1] = c1c3 + (s1s3*s2);
	R[1][2] = (s1c3*s2) - c1s3;
	R[2][0] = -s2;
	R[2][1] = c2*s3;
	R[2][2] = c2*c3;

	return 0;
}




rc_tf_t rc_tf_from_tait_bryan_xyz_intrinsic_degrees(int64_t ts, double roll, double pitch, double yaw, double xyz[3])
{
	rc_tf_t ret;
	_tait_bryan_xyz_intrinsic_degrees_to_rotation_matrix(roll, pitch, yaw, ret.d);
	ret.d[0][3] = xyz[0];
	ret.d[1][3] = xyz[1];
	ret.d[2][3] = xyz[2];
	ret.ts = ts;
	return ret;
}



rc_tf_t rc_tf_from_tait_bryan_zyx_intrinsic_degrees(int64_t ts, double yaw, double pitch, double roll, double xyz[3])
{
	rc_tf_t ret;
	_tait_bryan_zyx_intrinsic_degrees_to_rotation_matrix(yaw, pitch, roll, ret.d);
	ret.d[0][3] = xyz[0];
	ret.d[1][3] = xyz[1];
	ret.d[2][3] = xyz[2];
	ret.ts = ts;
	return ret;
}



int rc_tf_print_tait_bryan_xyz_intrinsic_degrees(rc_tf_t* tf)
{
	double roll, pitch, yaw;
	_rotation_matrix_to_tait_bryan_xyz_intrinsic_degrees(tf->d, &roll, &pitch, &yaw);
	double x = tf->d[0][3];
	double y = tf->d[1][3];
	double z = tf->d[2][3];
	printf("XYZ: %6.3f %6.3f %6.3f | RPY: %6.1f %6.1f %6.1f\n", x, y, z, roll, pitch, yaw);
	return 0;
}


int rc_tf_print_tait_bryan_zyx_intrinsic_degrees(rc_tf_t* tf)
{
	double roll, pitch, yaw;
	_rotation_matrix_to_tait_bryan_zyx_intrinsic_degrees(tf->d, &yaw, &pitch, &roll);
	double x = tf->d[0][3];
	double y = tf->d[1][3];
	double z = tf->d[2][3];
	printf("XYZ: %6.3f %6.3f %6.3f | YPR: %6.1f %6.1f %6.1f\n", x, y, z, yaw, pitch, roll);
	return 0;
}




int rc_tfv_print_matrix(rc_tfv_t* tfv)
{
	rc_tf_print_matrix(&tfv->tf);
	printf("v: %6.3f %6.3f %6.3f\n", tfv->v[0],tfv->v[1],tfv->v[2]);
	printf("w: %6.3f %6.3f %6.3f\n", tfv->w[0],tfv->w[1],tfv->w[2]);
	return 0;
}


int rc_tfv_print_tait_bryan_xyz_intrinsic_degrees(rc_tfv_t* tfv)
{
	rc_tf_print_tait_bryan_xyz_intrinsic_degrees(&tfv->tf);
	printf("vel: %6.3f %6.3f %6.3f |   w: %6.1f %6.1f %6.1f\n",\
				tfv->v[0], tfv->v[1], tfv->v[2], \
				tfv->w[0]*RAD_TO_DEG, tfv->w[1]*RAD_TO_DEG, tfv->w[2]*RAD_TO_DEG);
	return 0;
}


int rc_tfv_print_tait_bryan_zyx_intrinsic_degrees(rc_tfv_t* tfv)
{
	rc_tf_print_tait_bryan_zyx_intrinsic_degrees(&tfv->tf);
	printf("vel: %6.3f %6.3f %6.3f |   w: %6.1f %6.1f %6.1f\n",\
				tfv->v[0], tfv->v[1], tfv->v[2], \
				tfv->w[0]*RAD_TO_DEG, tfv->w[1]*RAD_TO_DEG, tfv->w[2]*RAD_TO_DEG);
	return 0;
}





// TODO make sure this uses NEON
int rc_tf_transform_vector(rc_tf_t* tf, double in[3], double out[3])
{
	int i,j;
	for(i=0;i<3;i++){
		out[i] = tf->d[i][3];
		for(j=0;j<3;j++){
			out[i] += tf->d[i][j] * in[j];
		}
	}
	return 0;
}


// TODO make sure this uses NEON
int rc_tf_transform_vector_inplace(rc_tf_t* tf, double v[3])
{
	int i,j;
	double x[4];
	x[0] = v[0];
	x[1] = v[1];
	x[2] = v[2];
	x[3] = 1.0;
	for(i=0;i<3;i++){
		v[i] = 0.0;
		for(j=0;j<4;j++){
			v[i] += tf->d[i][j] * x[j];
		}
	}
	return 0;
}

// TODO make sure this uses NEON
int rc_tf_invert(rc_tf_t* in, rc_tf_t* out)
{
	int i,j;
	// transpose R
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			out->d[i][j] = in->d[j][i];
		}
	}
	// Translation: T_B_wrt_A = -1 * R_B_to_A * T_A_wrt_B
	for(i=0;i<3;i++){
		out->d[i][3] = 0.0;
		for(j=0;j<3;j++){
			out->d[i][3] -= out->d[i][j] * in->d[j][3];
		}
	}
	return 0;
}



// this takes the whole 3x4 transform matrix but only reads the 3x3 rotation
int _rotation_to_quaternion(double R[3][4], double* q)
{
	double t,s;
	double trace = R[0][0] + R[1][1] + R[2][2];

	// NEEDS CHECKING
	if(trace > 0.0){
		s = sqrt(trace + 1.0);
		q[0] = 0.5 * s;
		s = 0.5 / s;
		q[1] = (R[1][2] - R[2][1]) * s;
		q[2] = (R[2][0] - R[0][2]) * s;
		q[3] = (R[0][1] - R[1][0]) * s;
	}

	// algorithm courtesy of Mike Day
	// https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
	if (R[2][2] < 0){
		if(R[0][0] >R[1][1]){
			t = 1 + R[0][0] - R[1][1] - R[2][2];
			s = (0.5 / sqrt(t));
			q[0] = (R[1][2] - R[2][1]) * s;
			q[1] = t*s;
			q[2] = (R[0][1] + R[1][0]) * s;
			q[3] = (R[2][0] + R[0][2]) * s;
		}else{
			t = 1 - R[0][0] + R[1][1] - R[2][2];
			s = (0.5 / sqrt(t));
			q[0] = (R[2][0] - R[0][2]) * s;
			q[1] = (R[0][1] + R[1][0]) * s;
			q[2] = t*s;
			q[3] = (R[1][2] + R[2][1]) * s;
		}
	}else{
		if(R[0][0] < -R[1][1]){
			t = 1 - R[0][0] - R[1][1] + R[2][2];
			s = (0.5 / sqrt(t));
			q[0] = (R[0][1] - R[1][0]) * s;
			q[1] = (R[2][0] + R[0][2]) * s;
			q[2] = (R[1][2] + R[2][1]) * s;
			q[3] = t*s;
		}else{
			t = 1 + R[0][0] + R[1][1] + R[2][2];
			s = (0.5 / sqrt(t));
			q[0] = t*s;
			q[1] = (R[1][2] - R[2][1]) * s;
			q[2] = (R[2][0] - R[0][2]) * s;
			q[3] = (R[0][1] - R[1][0]) * s;
		}
	}
	return 0;
}


// this takes the whole 3x4 translation matrix but only writes the 3x3 rotation
static int _quaternion_to_rotation_matrix(double q[4], double R[3][4])
{
	double s,xs,ys,zs,wx,wy,wz,xx,xy,xz,yy,yz,zz;

	// algorithm courtesy of "Advanced Animation and Rendering Techniques, theory
	// and practice" by Alan and Mark Watt.
	s = 2.0/(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

	// compute intermediate variables which will be used multiple times
	xs=q[1]*s; ys=q[2]*s; zs=q[3]*s;
	wx=q[0]*xs; wy=q[0]*ys; wz=q[0]*zs;
	xx=q[1]*xs; xy=q[1]*ys; xz=q[1]*zs;
	yy=q[2]*ys; yz=q[2]*zs; zz=q[3]*zs;

	R[0][0] = 1.0 - (yy + zz);
	R[0][1] = xy + wz;;
	R[0][2] = xz - wy;;

	R[1][0] = xy - wz;
	R[1][1] = 1.0 - (xx + zz);
	R[1][2] = yz + wx;

	R[2][0] = xz + wy;
	R[2][1] = yz - wx;
	R[2][2] = 1.0 - (xx + yy);

	return 0;
}


static int _slerp(double q1[4], double q2[4], double h, double out[4])
{
	// algorithm courtesy of "Advanced Animation and Rendering Techniques, theory
	// and practice" by Alan and Mark Watt.
	int i;
	double omega,cosom,sinom,sclp,sclq;
	cosom = (q1[0]*q2[0])+(q1[1]*q2[1])+(q1[2]*q2[2])+(q1[3]*q2[3]);

	if((1.0+cosom)>0.00001){
		if((1.0-cosom)>0.00001){
			omega = acos(cosom);
			sinom = sin(omega);
			sclp = sin((1.0-h)*omega) / sinom;
			sclq = sin(h*omega) / sinom;
		}
		else{
			sclp = 1.0 - h;
			sclq = h;
		}
		for(i=0;i<4;i++) out[i] = (sclp*q1[i]) + (sclq*q2[i]);
	}
	else{
		out[0] =  q1[3];
		out[1] = -q1[2];
		out[2] =  q1[1];
		out[3] = -q1[0];
		sclp = sin((1.0-h)*M_PI_2);
		sclq = sin(h*M_PI_2);
		for(i=1;i<4;i++) out[i] = (sclp*q1[i]) + (sclq*out[i]);
	}
	return 0;
}


int rc_tf_linear_interpolate(rc_tf_t* A, rc_tf_t* B, double h, rc_tf_t* out)
{
	// translation
	int i;
	for(i=0;i<3;i++){
		out->d[i][3] = A->d[i][3] + (h*(B->d[i][3] - A->d[i][3]));
	}

	// linearly interpolate Rotation with SLERP
	double Aq[4], Bq[4], Cq[4];
	_rotation_to_quaternion(A->d, Aq);
	_rotation_to_quaternion(B->d, Bq);
	_slerp(Aq, Bq, h, Cq);
	_quaternion_to_rotation_matrix(Cq, out->d);

	// also populate output timestamp if input timestamps are valid
	if(A->ts>0 && B->ts>0){
		int64_t diff = B->ts - A->ts;
		int64_t new = A->ts + (int64_t)((double)diff*h);
		out->ts = new;
	}
	else{
		out->ts = -1;
	}

	return 0;
}


int rc_tf_cubic_interpolate(rc_tfv_t* A, rc_tfv_t* B, double h, rc_tf_t* out)
{
	// translation
	int i;
	for(i=0;i<3;i++){
		double v1  = A->v[i];
		double v2  = B->v[i];
		double dt  = (double)(B->tf.ts - A->tf.ts) / 1000000000.0;
		double y1  = A->tf.d[i][3];
		double y2  = B->tf.d[i][3];
		double a   = (v1 + v2 - 2.0*((y2-y1)/dt)) / (dt*dt);
		double b   = ((v2 - v1)/(2.0*dt)) - (1.5*dt*a);
		double t   = dt*h;
		double tt  = t*t;
		double ttt = tt*t;
		out->d[i][3] = a*ttt + b*tt + v1*t + y1;
	}

	// linearly interpolate Rotation with SLERP
	// TODO try cubic here too
	double Aq[4], Bq[4], Cq[4];
	rc_tf_t AA = A->tf;
	rc_tf_t BB = B->tf;
	_rotation_to_quaternion(AA.d, Aq);
	_rotation_to_quaternion(BB.d, Bq);
	_slerp(Aq, Bq, h, Cq);
	_quaternion_to_rotation_matrix(Cq, out->d);

	// also populate output timestamp if input timestamps are valid
	if(A->tf.ts>0 && B->tf.ts>0){
		int64_t diff = B->tf.ts - A->tf.ts;
		int64_t new = A->tf.ts + (int64_t)((double)diff*h);
		out->ts = new;
	}
	else{
		out->ts = -1;
	}

	return 0;
}

int rc_tf_combine_two(rc_tf_t B_to_A, rc_tf_t C_to_B, rc_tf_t* C_to_A)
{
	int i,j,k;
	memset(C_to_A, 0, sizeof(rc_tf_t));
	for(i=0;i<3;i++){ // output row
		// rotation
		for(j=0;j<3;j++){ // output col
			for(k=0;k<3;k++){
				C_to_A->d[i][j] += B_to_A.d[i][k] * C_to_B.d[k][j];
			}
		}
		// translation
		C_to_A->d[i][3] = B_to_A.d[i][3];
		for(k=0;k<3;k++){
			C_to_A->d[i][3] += B_to_A.d[i][k] * C_to_B.d[k][3];
		}
	}
	return 0;
}
