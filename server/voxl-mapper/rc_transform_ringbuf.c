/**
 * rc_transform_ringbuf.c
 *
 *
 * @author     james@modalai.com
 * @date       2021
 */



#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include "rc_transform_ringbuf.h"


#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif



int rc_tf_ringbuf_alloc(rc_tfv_ringbuf_t* buf, int size)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(size<2)){
		fprintf(stderr,"ERROR in %s, size must be >=2\n", __FUNCTION__);
		return -1;
	}

	// if it's already allocated, nothing to do
	if(buf->initialized && buf->size==size && buf->d!=NULL) return 0;

	// make sure it's zero'd out
	buf->size = 0;
	buf->index = 0;
	buf->items_in_buf=0;
	buf->initialized = 0;

	// allocate mem for array
	buf->d = (rc_tfv_t*)calloc(size,sizeof(rc_tfv_t));
	if(buf->d==NULL){
		fprintf(stderr,"ERROR in %s, failed to allocate memory\n", __FUNCTION__);
		return -1;
	}

	// write out other details
	buf->size = size;
	buf->initialized = 1;
	return 0;
}


int rc_tf_ringbuf_free(rc_tfv_ringbuf_t* buf)
{
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(buf->initialized){
		free(buf->d);
		// put buffer back to default
		*buf = rc_tf_ringbuf_empty();
	}
	return 0;
}


rc_tfv_ringbuf_t rc_tf_ringbuf_empty(void)
{
	rc_tfv_ringbuf_t new = RC_TF_RINGBUF_INITIALIZER;
	return new;
}


int rc_tf_ringbuf_insert_tf(rc_tfv_ringbuf_t* buf, rc_tf_t* new_tf)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
		return -1;
	}

	// construct a new tfv to put in the ringbuffer
	rc_tfv_t tfv;
	tfv.tf = *new_tf;

	// flag velocities as invalid
	int i;
	for(i=0;i<3;i++){
		tfv.v[i] = NAN;
		tfv.w[i] = NAN;
	}

	return rc_tf_ringbuf_insert_tfv(buf, &tfv);
}


int rc_tf_ringbuf_insert_tfv(rc_tfv_ringbuf_t* buf, rc_tfv_t* new_tf)
{
	int new_index;

	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}

	// we are about to interact with the ringbuf, lock the mutex
	pthread_mutex_lock(&buf->mutex);

	// more sanity checks
	if(unlikely(!buf->initialized)){
		pthread_mutex_unlock(&buf->mutex);
		fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(new_tf->tf.ts <= buf->latest_ts)){
		pthread_mutex_unlock(&buf->mutex);
		fprintf(stderr,"ERROR in %s, detected timestamp out of order\n", __FUNCTION__);
		return -1;
	}

	// if this is the first thing to be entered make sure to start at zero
	if(buf->items_in_buf==0){
		new_index = 0;
	}
	else{
		// increment index and check for loop-around
		new_index = buf->index+1;
		if(new_index >= buf->size) new_index = 0;
	}

	// copy the data into our buffer
	memcpy(&buf->d[new_index], new_tf, sizeof(rc_tfv_t));

	// bump index and increment number of items if necessary
	buf->index = new_index;
	if(buf->items_in_buf < buf->size){
		buf->items_in_buf++;
	}

	// all done, save the timestamp and unlock mutex
	buf->latest_ts = new_tf->tf.ts;
	pthread_mutex_unlock(&buf->mutex);
	return 0;
}


// TODO remove this, it's only temporary for voxl-mapper, 
// we don't want dependencies on libmodal_pipe in librc_math
int rc_tf_ringbuf_insert_pose(rc_tfv_ringbuf_t* buf, pose_vel_6dof_t pose)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
		return -1;
	}

	// construct a new tfv to put in the ringbuffer
	rc_tfv_t tfv;
	tfv.tf.ts = pose.timestamp_ns;
	int i,j;
	for(i=0;i<3;i++){
		tfv.tf.d[i][3] = pose.T_child_wrt_parent[i];
		tfv.v[i] = pose.v_child_wrt_parent[i];
		tfv.w[i] = pose.w_child_wrt_child[i];
		for(j=0;j<3;j++){
			tfv.tf.d[i][j] = pose.R_child_to_parent[i][j];
		}
	}

	return rc_tf_ringbuf_insert_tfv(buf, &tfv);
}


int rc_tf_ringbuf_get_tfv_at_pos(rc_tfv_ringbuf_t* buf, int position, rc_tfv_t* result)
{
	// sanity checks
	if(unlikely(buf==NULL || result==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(position<0)){
		fprintf(stderr,"ERROR in %s, position must be >= 0\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
		return -1;
	}

	// about to start reading the buffer, lock the mutex
	pthread_mutex_lock(&buf->mutex);

	// silently return if user requested a position beyond buffer size
	if(position >= buf->size){
		pthread_mutex_unlock(&buf->mutex);
		return -3;
	}
	// silently return if user requested an item that hasn't been added yet
	if(position >= buf->items_in_buf){
		pthread_mutex_unlock(&buf->mutex);
		return -2;
	}

	// return index is just latest index minus position due to the order we keep
	// data (populated from left to right)
	int return_index = buf->index - position;

	// check for looparound
	if(return_index < 0){
		return_index += buf->size;
	}

	// write out data
	*result = buf->d[return_index];

	// all done, unlock mutex
	pthread_mutex_unlock(&buf->mutex);

	return 0;
}


// Fetches the timestamp which is 'position' steps behind the latest
// This is unprotected, for internal use only! Be careful!
static int64_t _get_ts_at_pos(rc_tfv_ringbuf_t* buf, int position)
{
	// silently return if user requested an item that hasn't been added yet
	if(position >= buf->items_in_buf){
		return -2;
	}

	// return index is just latest index minus position due to the order we keep
	// data (populated from left to right)
	int return_index = buf->index - position;

	// check for looparound
	if(return_index < 0){
		return_index += buf->size;
	}

	// return the requested timestamp
	return buf->d[return_index].tf.ts;
}


// Fetches a pointer to the tfv which is 'position' steps behind the latest
// This is unprotected, for internal use only! Be careful!
static rc_tfv_t* _get_tfv_ptr_at_pos(rc_tfv_ringbuf_t* buf, int position)
{
	// return index is just latest index minus position due to the order we keep
	// data (populated from left to right)
	int return_index = buf->index - position;

	// check for looparound
	if(return_index < 0){
		return_index += buf->size;
	}

	// write out data
	return &buf->d[return_index];
}



int rc_tf_ringbuf_get_tf_at_time(rc_tfv_ringbuf_t* buf, int64_t ts, rc_tf_t* result)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(ts<=0)){
		fprintf(stderr,"ERROR in %s, requested timestamp must be >0\n", __FUNCTION__);
		return -1;
	}
	if(buf->items_in_buf < 2){
		return -2;
	}

	// about to start messing with the buffer, lock the mutex
	pthread_mutex_lock(&buf->mutex);

	// allow timestamps up to 0.2s newer than our last position record
	if(ts > (buf->latest_ts+200000000)){
		fprintf(stderr,"ERROR in %s, timestamp too new\n", __FUNCTION__);
		pthread_mutex_unlock(&buf->mutex);
		return -3;
	}
	// don't deal with timestamps older than our buffer has data for
	if(ts < _get_ts_at_pos(buf,buf->items_in_buf-1)){
		fprintf(stderr, "ERROR in %s, requested timestamp older than oldest member in buffer\n", __FUNCTION__);
		pthread_mutex_unlock(&buf->mutex);
		return -4;
	}

	// next logic is going to be to find the two transforms to interpolate over
	rc_tfv_t *tfv_ptr_before, *tfv_ptr_after;
	
	// check for timestamp newer than we have record of, if so, extrapolate given
	// the two most recent records
	if(ts > buf->latest_ts){
		tfv_ptr_before = _get_tfv_ptr_at_pos(buf,1);
		tfv_ptr_after  = _get_tfv_ptr_at_pos(buf,0);
	}

	// now go searching through the buffer to find which two entries to 
	// interpolate between, starting from newest. TODO: binary search
	else{
		for(int i=0;i<buf->items_in_buf;i++){
			// timestamp to check at this point
			int64_t ts_at_i = _get_ts_at_pos(buf,i);

			// found the right value! no interpolation needed
			if(ts_at_i == ts){
				rc_tfv_t* tfv_ptr = _get_tfv_ptr_at_pos(buf,i);
				memcpy(result, &tfv_ptr->tf, sizeof(rc_tf_t));
				pthread_mutex_unlock(&buf->mutex);
				return 0;
			}

			// once we get a timestamp older than requested ts, we have found the
			// right interval, grab the appropriate transforms
			if(ts_at_i < ts){
				tfv_ptr_before = _get_tfv_ptr_at_pos(buf,i);
				tfv_ptr_after  = _get_tfv_ptr_at_pos(buf,i-1);
				break;
			}
		}
	}

	// calculate interpolation constant h which is between 0 and 1.
	// 0 would be right at tfv_ptr_before, 1 would be right at tfv_ptr_after.
	double h = (double)(ts-tfv_ptr_before->tf.ts) / (double)(tfv_ptr_after->tf.ts-tfv_ptr_before->tf.ts);

	// if no velocity, do linear interpolation
	// TODO check angular velocity too
	int ret;
	if( isnan(tfv_ptr_before->v[0]) || isnan(tfv_ptr_before->v[1]) || isnan(tfv_ptr_before->v[2]) || \
		isnan(tfv_ptr_after->v[0])  || isnan(tfv_ptr_after->v[1])  || isnan(tfv_ptr_after->v[2]))
	{
		ret = rc_tf_linear_interpolate(&tfv_ptr_before->tf, &tfv_ptr_after->tf, h, result);
	}
	else{
		ret = rc_tf_cubic_interpolate(tfv_ptr_before, tfv_ptr_after, h, result);
	}
	pthread_mutex_unlock(&buf->mutex);
	return ret;
}

