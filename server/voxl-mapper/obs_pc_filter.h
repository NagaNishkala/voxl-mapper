#ifndef OBS_PC_FILTER_H
#define OBS_PC_FILTER_H

#include <stdint.h>
#include <float.h>
#include <pthread.h>
#include <voxblox/core/common.h>


#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

/**
 * @brief      downsample a high-density point cloud
 *
 *             creates and allocates a new obstable point cloud from a list of
 *             raw points. It downsamples by binning into a 3D grid and filters
 *             out stray floaters tunable by the threshold argument.
 *
 *             This is intended for point clouds from DFS or TOF sensors where Z
 *             points out the sensor. As such, it throws out any points with a Z
 *             value <=0 or >max_depth which discards the "zero" and "inf"
 *             points that DFS and TOF generate.
 *
 *             Cell size is the resolution of the 3D grid, a larger number will
 *             result in a more sparse output point cloud. Recommend something
 *             around 0.15
 *
 *             cells in the grid with fewer than "threshold" occupied cells in
 *             the surrounding 3x3 block will be discarded. This includes the
 *             cell itself, so threshold cannot be less than 1 or greater than
 *             27. Setting threshold to 1 disables this check, since the
 *             occupied cell itself counts in the check. The intention of this
 *             filter is to remove stray floating points which are likely noise.
 *
 * @param[in]  n          number of input points
 * @param      in         array of XYZ points
 * @param[in]  max_depth  The maximum depth in +Z direction allowed
 * @param[in]  cell_size  The cell size
 * @param[in]  threshold  The threshold
 * @param      out        The out
 *
 * @return     0 on success, -1 on failure
 */
int tof_pc_downsample(int n, const float in[][3], const uint8_t* conf, uint8_t conf_cutoff, float max_depth, float cell_size, int threshold, voxblox::Pointcloud* out, std::vector<uint8_t>* out_conf);

int dfs_pc_downsample(int n, const float in[][3], float max_depth, float cell_size, int threshold, voxblox::Pointcloud* out);


#endif // OBS_PC_FILTER_H
