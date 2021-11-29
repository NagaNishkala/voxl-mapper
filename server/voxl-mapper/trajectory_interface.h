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


#ifndef TRAJECTORY_INTERFACE_H
#define TRAJECTORY_INTERFACE_H

#define TRAJECTORY_PIPE_TOPIC	"/run/mpa/plan_msgs/"

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 * Also it spells "VOXL" in ASCII
 */
#define TRAJECTORY_MAGIC_NUMBER (0x564F584C)


// trajectory is made of up to 20 segements
// each segment is defined by up to an 11th order polynomial
#define TRAJ_MAX_COEFFICIENTS		12
#define TRAJ_MAX_SEGMENTS			20


// coefficients are INCREASING, e.g. x(t) = a + bt + ct^2 ...
// time begins at 0 for each coefficient, and is valid up to duration_s
// order of the polynomial is n_coef-1
typedef struct poly_segment_t{
	int n_coef;			//< number of segments, must be 0<n<=MAX_SEGMENTS
	double duration_s;
	double cx[TRAJ_MAX_COEFFICIENTS];
	double cy[TRAJ_MAX_COEFFICIENTS];
	double cz[TRAJ_MAX_COEFFICIENTS];
	double cyaw[TRAJ_MAX_COEFFICIENTS];
} __attribute__((packed)) poly_segment_t;


// TODO make this dynamic
typedef struct trajectory_t{
	uint32_t magic_number;		///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
	int64_t creation_time_ns;	///< timestamp in monotonic time when the trajectory was planned, 0 if unknown
	int n_segments;				//< number of segments, must be 0<n<=MAX_SEGMENTS
	poly_segment_t segments[TRAJ_MAX_SEGMENTS];
} __attribute__((packed)) trajectory_t;




#define TRAJECTORY_RECOMMENDED_READ_BUF_SIZE	(sizeof(trajectory_t)*4)
#define TRAJECTORY_RECOMMENDED_PIPE_SIZE		(64*1024)



static inline trajectory_t* modal_trajectory_validate_pipe_data(char* data, int bytes, int* n_packets)
{
	// cast raw data from buffer to an vio_data_t array so we can read data
	// without memcpy. Also write out packets read as 0 until we validate data.
	trajectory_t* new_ptr = (trajectory_t*) data;
	*n_packets = 0;

	// basic sanity checks
	if(bytes<0){
		fprintf(stderr, "ERROR validating trajectory data received through pipe: number of bytes = %d\n", bytes);
		return NULL;
	}
	if(data==NULL){
		fprintf(stderr, "ERROR validating trajectory data received through pipe: got NULL data pointer\n");
		return NULL;
	}
	if(bytes%sizeof(trajectory_t)){
		fprintf(stderr, "ERROR validating trajectory data received through pipe: read partial packet\n");
		fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(trajectory_t));
		return NULL;
	}

	// calculate number of packets locally until we validate each packet
	int n_packets_tmp = bytes/sizeof(trajectory_t);

	// check if any packets failed the magic number check
	int i, n_failed = 0;
	for(i=0;i<n_packets_tmp;i++){
		if(new_ptr[i].magic_number != TRAJECTORY_MAGIC_NUMBER) n_failed++;
	}
	if(n_failed>0){
		fprintf(stderr, "ERROR validating trajectory data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
		return NULL;
	}

	// if we get here, all good. Write out the number of packets read and return
	// the new cast pointer. It's the same pointer the user provided but cast to
	// the right type for simplicity and easy of use.
	*n_packets = n_packets_tmp;
	return new_ptr;
}



#endif // TRAJECTORY_INTERFACE_H
