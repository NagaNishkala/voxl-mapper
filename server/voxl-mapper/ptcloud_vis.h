#ifndef VOXL_MAPPER_PTCLOUD_VIS_H_
#define VOXL_MAPPER_PTCLOUD_VIS_H_

typedef struct point_xyz {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
} __attribute__((packed)) point_xyz;

typedef struct point_xyz_rgb {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
} __attribute__((packed)) point_xyz_rgb;

typedef struct point_xyz_i {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float intensity = 0.0;
} __attribute__((packed)) point_xyz_i;

/**
 * This file contains a set of functions to visualize layers as pointclouds
 * (or marker arrays) based on a passed-in function. It also offers some
 * specializations of functions as samples.
 */

namespace voxblox {

const float kFloatEpsilon = 1e-6;

/** Shortcut the placeholders namespace, since otherwise it chooses the boost
 * placeholders which are in the global namespace (thanks boost!).
 */
namespace ph = std::placeholders;

/// This is a fancy alias to be able to template functions.
template <typename VoxelType>
using ShouldVisualizeVoxelColorFunctionType = std::function<bool(
    const VoxelType& voxel, const Point& coord, Color* color)>;

/**
 * For intensities values, such as distances, which are mapped to a color only
 * by the subscriber.
 */
template <typename VoxelType>
using ShouldVisualizeVoxelIntensityFunctionType = std::function<bool(
    const VoxelType& voxel, const Point& coord, double* intensity)>;

/**
 * For boolean checks -- either a voxel is visualized or it is not.
 * This is used for occupancy bricks, for instance.
 */
template <typename VoxelType>
using ShouldVisualizeVoxelFunctionType =
    std::function<bool(const VoxelType& voxel, const Point& coord)>;  // NOLINT;

/// Template function to visualize a colored pointcloud.
template <typename VoxelType>
void createColorPointcloudFromLayer(const Layer<VoxelType>& layer,
                                    const ShouldVisualizeVoxelColorFunctionType<VoxelType>& vis_function,
                                    std::vector<point_xyz_rgb>& ptcloud) {

    ptcloud.clear();
    BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);

    // Cache layer settings.
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    // Temp variables.
    Color color;
    // Iterate over all blocks.
    for (const BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<VoxelType>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block;
             ++linear_index) {
            Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            if (vis_function(block.getVoxelByLinearIndex(linear_index), coord,
                             &color)) {
                point_xyz_rgb pt;
                pt.x = coord.x();
                pt.y = coord.y();
                pt.z = coord.z();
                pt.r = color.r;
                pt.g = color.g;
                pt.b = color.b;
                ptcloud.push_back(pt);
            }
        }
    }
}

/// Template function to visualize an intensity pointcloud.
template <typename VoxelType>
void createColorPointcloudFromLayer(const Layer<VoxelType>& layer,
                                    const ShouldVisualizeVoxelIntensityFunctionType<VoxelType>& vis_function,
                                    std::vector<point_xyz_i>& ptcloud) {
    ptcloud.clear();
    BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);

    // Cache layer settings.
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    // Temp variables.
    double intensity = 0.0;
    // Iterate over all blocks.
    for (const BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<VoxelType>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block;
             ++linear_index) {
            Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            if (vis_function(block.getVoxelByLinearIndex(linear_index), coord,
                             &intensity)) {
                point_xyz_i pt;
                pt.x = coord.x();
                pt.y = coord.y();
                pt.z = coord.z();
                pt.intensity = intensity;
                ptcloud.push_back(pt);
            }
        }
    }
}

// /Short-hand functions for visualizing different types of voxels.
inline bool visualizeNearSurfaceTsdfVoxels(const TsdfVoxel& voxel, const Point& /*coord*/,
                                           double surface_distance,
                                           Color* color) {
    if(color == nullptr){
        fprintf(stderr, "colors is null: %s\n", __FUNCTION__);
        return false;
    }
    constexpr float kMinWeight = 0;
    if (voxel.weight > kMinWeight &&
        std::abs(voxel.distance) < (float)surface_distance) {
        *color = voxel.color;
        return true;
    }
    return false;
}

inline bool visualizeTsdfVoxels(const TsdfVoxel& voxel, const Point& /*coord*/,
                                Color* color) {
    if(color == nullptr){
        fprintf(stderr, "colors is null: %s\n", __FUNCTION__);
        return false;
    }
    constexpr float kMinWeight = 0;
    if (voxel.weight > kMinWeight) {
        *color = voxel.color;
        return true;
    }
    return false;
}

inline bool visualizeDistanceIntensityTsdfVoxels(const TsdfVoxel& voxel,
                                                 const Point& /*coord*/,
                                                 double* intensity) {
    if(intensity == nullptr){
        fprintf(stderr, "intensity is null: %s\n", __FUNCTION__);
        return false;
    }
    constexpr float kMinWeight = 1e-3;
    if (voxel.weight > kMinWeight) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

inline bool visualizeDistanceIntensityTsdfVoxelsNearSurface(const TsdfVoxel& voxel, const Point& /*coord*/, double surface_distance,
                                                            double* intensity) {
    if(intensity == nullptr){
        fprintf(stderr, "intensity is null: %s\n", __FUNCTION__);
        return false;
    }
    constexpr float kMinWeight = 1e-3;
    if (voxel.weight > kMinWeight &&
        std::abs(voxel.distance) < (float)surface_distance) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

inline bool visualizeDistanceIntensityTsdfVoxelsSlice(const TsdfVoxel& voxel, const Point& coord,
                                                      unsigned int free_plane_index, float free_plane_val,
                                                      float voxel_size, double* intensity) {
    if(intensity == nullptr){
        fprintf(stderr, "intensity is null: %s\n", __FUNCTION__);
        return false;
    }
    constexpr float kMinWeight = 1e-3;

    if (std::abs(coord(free_plane_index) - free_plane_val) <=
        (voxel_size / 2.0f + kFloatEpsilon)) {
        if (voxel.weight > kMinWeight) {
            *intensity = voxel.distance;
            return true;
        }
    }
    return false;
}

inline bool visualizeDistanceIntensityEsdfVoxels(const EsdfVoxel& voxel, const Point& /*coord*/, double* intensity) {
    if(intensity == nullptr){
        fprintf(stderr, "intensity is null: %s\n", __FUNCTION__);
        return false;
    }
    if (voxel.observed) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

inline bool visualizeIntensityVoxels(const IntensityVoxel& voxel, const Point& /*coord*/, double* intensity) {
    constexpr float kMinWeight = 1e-3;

    if(intensity == nullptr){
        fprintf(stderr, "intensity is null: %s\n", __FUNCTION__);
        return false;
    }
    if (voxel.weight > kMinWeight) {
        *intensity = voxel.intensity;
        return true;
    }
    return false;
}

inline bool visualizeDistanceIntensityEsdfVoxelsSlice(const EsdfVoxel& voxel, const Point& coord, unsigned int free_plane_index,
                                                      float free_plane_val, float voxel_size, double* intensity) {
    if(intensity == nullptr){
        fprintf(stderr, "intensity is null: %s\n", __FUNCTION__);
        return false;
    }
    if (std::abs(coord(free_plane_index) - free_plane_val) <=
        (voxel_size / 2.0f + kFloatEpsilon)) {
        if (voxel.observed) {
            *intensity = voxel.distance;
            return true;
        }
    }
    return false;
}

inline bool visualizeOccupiedTsdfVoxels(const TsdfVoxel& voxel, const Point& /*coord*/, const float min_distance = 0.0) {
    constexpr float kMinWeight = 1e-3;
    if (voxel.weight > kMinWeight && voxel.distance <= min_distance) {
        return true;
    }
    return false;
}

inline bool visualizeFreeEsdfVoxels(const EsdfVoxel& voxel, const Point& /*coord*/, float min_distance, double* intensity) {
    if (voxel.observed && voxel.distance >= min_distance) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

inline bool visualizeOccupiedOccupancyVoxels(const OccupancyVoxel& voxel, const Point& /*coord*/) {
    const float kThresholdLogOccupancy = logOddsFromProbability(0.7);
    if (voxel.probability_log > kThresholdLogOccupancy) {
        return true;
    }
    return false;
}

// All functions that can be used directly for TSDF voxels.

/**
 * Create a pointcloud based on the TSDF voxels near the surface.
 * The RGB color is determined by the color of the TSDF voxel.
 */
inline void createSurfacePointcloudFromTsdfLayer(const Layer<TsdfVoxel>& layer, double surface_distance,
    std::vector<point_xyz_rgb> &ptcloud) {
    createColorPointcloudFromLayer<TsdfVoxel>(
        layer,
        std::bind(&visualizeNearSurfaceTsdfVoxels, ph::_1, ph::_2,
                  surface_distance, ph::_3),
                  ptcloud);
}

/**
 * Create a pointcloud based on all the TSDF voxels.
 * The RGB color is determined by the color of the TSDF voxel.
 */
inline void createPointcloudFromTsdfLayer(const Layer<TsdfVoxel>& layer, std::vector<point_xyz_rgb> &ptcloud) {
    createColorPointcloudFromLayer<TsdfVoxel>(layer, &visualizeTsdfVoxels,
                                              ptcloud);
}

/**
 * Create a pointcloud based on all the TSDF voxels.
 * The intensity is determined based on the distance to the surface.
 */
inline void createDistancePointcloudFromTsdfLayer(
    const Layer<TsdfVoxel>& layer,
    std::vector<point_xyz_i> &ptcloud) {
    createColorPointcloudFromLayer<TsdfVoxel>(
        layer, &visualizeDistanceIntensityTsdfVoxels, ptcloud);
}

/**
 * Create a pointcloud based on the TSDF voxels near the surface.
 * The intensity is determined based on the distance to the surface.
 */
inline void createSurfaceDistancePointcloudFromTsdfLayer(
    const Layer<TsdfVoxel>& layer, double surface_distance,
    std::vector<point_xyz_i> &ptcloud) {
    createColorPointcloudFromLayer<TsdfVoxel>(
        layer,
        std::bind(&visualizeDistanceIntensityTsdfVoxelsNearSurface, ph::_1,
                  ph::_2, surface_distance, ph::_3),
                  ptcloud);
}

inline void createDistancePointcloudFromEsdfLayer(
    const Layer<EsdfVoxel>& layer,
    std::vector<point_xyz_i> &ptcloud) {
    createColorPointcloudFromLayer<EsdfVoxel>(
        layer, &visualizeDistanceIntensityEsdfVoxels, ptcloud);
}

inline void createFreePointcloudFromEsdfLayer(
    const Layer<EsdfVoxel>& layer, float min_distance,
    std::vector<point_xyz_i> &ptcloud) {
    createColorPointcloudFromLayer<EsdfVoxel>(
        layer,
        std::bind(&visualizeFreeEsdfVoxels, ph::_1, ph::_2, min_distance, ph::_3),
        ptcloud);
}

inline void createIntensityPointcloudFromIntensityLayer(
    const Layer<IntensityVoxel>& layer,
    std::vector<point_xyz_i> &ptcloud) {
    createColorPointcloudFromLayer<IntensityVoxel>(
        layer, &visualizeIntensityVoxels, ptcloud);
}

inline void createDistancePointcloudFromTsdfLayerSlice(
    const Layer<TsdfVoxel>& layer, unsigned int free_plane_index,
    float free_plane_val, std::vector<point_xyz_i> &ptcloud) {
    // Make sure that the free_plane_val doesn't fall evenly between 2 slices.
    // Prefer to push it up.
    // Using std::remainder rather than std::fmod as the latter has huge crippling
    // issues with floating-point division.
    if (std::remainder(free_plane_val, layer.voxel_size()) < kFloatEpsilon) {
        free_plane_val += layer.voxel_size() / 2.0f;
    }

    createColorPointcloudFromLayer<TsdfVoxel>(
        layer,
        std::bind(&visualizeDistanceIntensityTsdfVoxelsSlice, ph::_1, ph::_2,
                  free_plane_index, free_plane_val, layer.voxel_size(), ph::_3),
        ptcloud);
}

inline void createDistancePointcloudFromEsdfLayerSlice(
    const Layer<EsdfVoxel>& layer, unsigned int free_plane_index,
    float free_plane_val, std::vector<point_xyz_i> &ptcloud) {
    // Make sure that the free_plane_val doesn't fall evenly between 2 slices.
    // Prefer to push it up.
    // Using std::remainder rather than std::fmod as the latter has huge crippling
    // issues with floating-point division.
    if (std::remainder(free_plane_val, layer.voxel_size()) < kFloatEpsilon) {
        free_plane_val += layer.voxel_size() / 2.0f;
    }

    createColorPointcloudFromLayer<EsdfVoxel>(
        layer,
        std::bind(&visualizeDistanceIntensityEsdfVoxelsSlice, ph::_1, ph::_2,
                  free_plane_index, free_plane_val, layer.voxel_size(), ph::_3),
        ptcloud);
}

}  // namespace voxblox

#endif  // VOXL_MAPPER_PTCLOUD_VIS_H_
