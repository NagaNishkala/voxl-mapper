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

#include <stdio.h>
#include <unistd.h>	// for usleep()
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "obs_pc_filter.h"

#define MIN_CONFIDENCE 45

int obs_pc_downsample(int n, const float in[][3], const uint8_t* conf, float max_depth, float cell_size, int threshold, voxblox::Pointcloud* out)
{
	// sanity checks
	if(unlikely(in==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(n<=0)){
		fprintf(stderr,"ERROR in %s, n points must be >0\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(out==NULL)){
		fprintf(stderr,"ERROR in %s, voxblox::Pointcloud output must be non-null\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(threshold<1 || threshold>27)){
		fprintf(stderr,"ERROR in %s, threshold must be in 1 to 27\n", __FUNCTION__);
		return -1;
	}


	int i,j,k;

	// bounding box to be calculated in first pass through points
	float x_min = FLT_MAX;
	float y_min = FLT_MAX;
	float z_min = FLT_MAX;
	float x_max = FLT_MIN;
	float y_max = FLT_MIN;
	float z_max = FLT_MIN;

	// allocate space for a new list of 'valid' points. Point clouds may contain
	// a majority of "zero" points were no tof/dfs data exists
	int n_valid_points = 0;
	float* valid_points = (float*)malloc(n*3*sizeof(float));
	if(valid_points==NULL){
		fprintf(stderr, "ERROR in %s, failed to malloc buffer\n", __FUNCTION__);
		return -1;
	}

	////////////////////////////////////////////////////////////////////////////
	// Find valid point cloud values and bounding box around them
	////////////////////////////////////////////////////////////////////////////

	// check every point in the input
	for(i=0; i<n; i++){

		float x = in[i][0];
		float z = in[i][2];
		float y = in[i][1];
        uint8_t c = conf[i];

		// throw out the "zero" and "inf" points from DFS/TOF
		if(z<=0.0f || z>max_depth) continue;

        // throw out the poor confidence values from TOF
        if (c <= MIN_CONFIDENCE) continue;

		if(z<z_min) z_min=z;
		if(z>z_max) z_max=z;

		// copy data over into valid points array
		int p = (n_valid_points*3);
		valid_points[p]   = x;
		valid_points[p+1] = y;
		valid_points[p+2] = z;
		n_valid_points++;
	}

////////////////////////////////////////////////////////////////////////////
// validate the bounding box
////////////////////////////////////////////////////////////////////////////

	// assume a fixed FOV and set x and y min/max based on Z
	// this saves significant checking in the last step
	x_min = -z_max*1.6f;
	x_max =  z_max*1.6f;
	y_min = -z_max*1.6f;
	y_max =  z_max*1.6f;

	// check that the bounding box worked. if min==max that's okay, that would
	// indicate all points are on a plane which is normal.
	if(x_min>x_max || y_min>y_max || z_min>z_max){
		fprintf(stderr, "ERROR in %s, invalid bounding box\n", __FUNCTION__);
		free(valid_points); // free old memory, it's still allocated!
		return -1;
	}

	// set up map1 to break up input points into grid. Need an extra empty box
	// around the edges for the box filter step, so add 2, 1 for each side
	int nx = (((x_max-x_min)/cell_size)+2);
	int ny = (((y_max-y_min)/cell_size)+2);
	int nz = (((z_max-z_min)/cell_size)+2);

////////////////////////////////////////////////////////////////////////////////
// insert valid points into map1 which is an occupancy map and map2 which
// contains the index from the "valid points" array for each occupied cell.
////////////////////////////////////////////////////////////////////////////////

	int map1_pts = 0;
	uint8_t map1[nx][ny][nz];
	uint16_t map1_populated_indices[n_valid_points][3];
	memset(map1, 0, nx*ny*nz);
	int map2[nx][ny][nz];

	int nx2 = (nx/2)+1;
	int ny2 = (ny/2)+1;

	// populate map1 with 1's and record the index in map2
	// idea: could populate with higher numbers indicating higher confidence
	// points or multiple points in that area
	for(i=0; i<n_valid_points; i++){

		float x = valid_points[(i*3)];
		float y = valid_points[(i*3)+1];
		float z = valid_points[(i*3)+2];

		// find index, add 1 due to the buffer box around the edge
		int xidx = (int)(x/cell_size)+nx2;
		int yidx = (int)(y/cell_size)+ny2;
		int zidx = (((z-z_min)/cell_size)+1);

		// This check isn't really necessary and hasn't tripped yet, but it's
		// cheap assurance against a segfault.
		if(unlikely(xidx<1 || yidx<1 ||  xidx>=nx || yidx>=ny)){
			fprintf(stderr, "WARNING in %s, index OOB: %d %d %d for point: %6.2f %6.2f %6.2f\n",\
						__FUNCTION__, xidx, yidx, zidx, (double)x, (double)y, (double)z);
			continue;
		}

		// record a new block if this one is empty
		if(map1[xidx][yidx][zidx]==0){
			map1[xidx][yidx][zidx]=1;
			map2[xidx][yidx][zidx]=i;
			map1_populated_indices[map1_pts][0] = xidx;
			map1_populated_indices[map1_pts][1] = yidx;
			map1_populated_indices[map1_pts][2] = zidx;

			map1_pts++;
		}
	}

////////////////////////////////////////////////////////////////////////////////
// Box filter around occupied cells in map1 using record
////////////////////////////////////////////////////////////////////////////////

	int n_out = 0; // output data points
	float output[map1_pts*3];

	for(int p=0;p<map1_pts;p++){

		// grab the indices in map1 from record
		i = map1_populated_indices[p][0];
		j = map1_populated_indices[p][1];
		k = map1_populated_indices[p][2];

		// sum neighboring cells if required
		int ctr = 0;
		if(threshold==1){
			ctr=1;
		}
		else{
			for(int ii=(i-1);ii<=(i+1);ii++){
				for(int jj=(j-1);jj<=(j+1);jj++){
					for(int kk=(k-1);kk<=(k+1);kk++){
						ctr+=map1[ii][jj][kk];
					}
				}
			}
		}

		// same counter as we did for z&y, but no need to save this time
		// just check if we need to output the point.
		if(ctr>=threshold){
			int idx = map2[i][j][k]*3;
			output[(n_out*3)]   = valid_points[idx];
			output[(n_out*3)+1] = valid_points[idx+1];
			output[(n_out*3)+2] = valid_points[idx+2];
			n_out++;
		}
	}

	// done with the dynmaically allocated data now
	free(valid_points);

////////////////////////////////////////////////////////////////////////////////
// copy out data now we know how many points we have
////////////////////////////////////////////////////////////////////////////////

	if(n_out>0){
        out->resize(n_out);
		memcpy(out->data(), output, n_out*3*sizeof(float));
	}

	return 0;
}
