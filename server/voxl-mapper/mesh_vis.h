// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// Mesh output taken from open_chisel: github.com/personalrobotics/OpenChisel

#ifndef VOXBLOX_MESH_VIS_H_
#define VOXBLOX_MESH_VIS_H_

#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <modal_pipe.h>
#include "conversions.h"

#define MESH_MAGIC_NUMBER (0x4d455348)
typedef struct mesh_metadata_t {
    uint32_t magic_number;                  ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
    int64_t  timestamp_ns;                  ///< timestamp at the middle of the frame exposure in monotonic time
    uint64_t size_bytes;                    ///< Total size of following data (num_blocks number of blocks, each containing metadata and array of vertices
    uint32_t num_vertices;
    uint64_t reserved[5];                   ///< Reserved fields for later use, total size of struct 64 bytes
} __attribute__((packed)) mesh_metadata_t;

typedef struct mesh_vertex_t {
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    float normal_x;
    float normal_y;
    float normal_z;
} __attribute__((packed)) mesh_vertex_t;

namespace voxblox {

enum ColorMode {
    kColor = 0,
    kHeight,
    kNormals
};

constexpr float heat[] =
    {
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 1.0, 1.0,
        0.0, 0.0, 1.0
        };

#define COLOR_MODE "height"

//TODO: Eventually make this not all just gray
inline void getVertexColor(mesh_vertex_t *vertex, const voxblox::Mesh &mesh, const int index, const ColorMode color_mode) {

    switch (color_mode)
    {
    case ColorMode::kColor: {

        vertex->r = mesh.colors[index].r;
        vertex->g = mesh.colors[index].g;
        vertex->b = mesh.colors[index].b;
        break;
    }

    case ColorMode::kHeight: {

        // Interpolate the colors based on the height
        float height = -mesh.vertices[index].z() / 3.0f;
        float a = height * 5.0f;
        int color0 = std::floor(a);
        int color1 = std::ceil(a);
        float t = a - color0;

        vertex->r = 255 * ((1 - t) * heat[color0 * 3] + t * heat[color1 * 3]);
        vertex->g = 255 * ((1 - t) * heat[color0 * 3 + 1] + t * heat[color1 * 3 + 1]);
        vertex->b = 255 * ((1 - t) * heat[color0 * 3 + 2] + t * heat[color1 * 3 + 2]);
        break;
    }

    case ColorMode::kNormals: {
        
        // Normals should be in scale -1 to 1, so shift to 0 - 255
        vertex->r = 255 * (1 - mesh.normals[index].x() * 0.5f + 0.5f);
        vertex->g = 255 * (1 - mesh.normals[index].y() * 0.5f + 0.5f);
        vertex->b = 255 * (1 - mesh.normals[index].z() * 0.5f + 0.5f);
        break;
    }
    }

}

inline ColorMode getColorModeFromString(const std::string &color_mode_string) {
    if (color_mode_string.empty()) {
        return ColorMode::kColor;
    } else {
        if (color_mode_string == "color" || color_mode_string == "colors") {
            return ColorMode::kColor;
        } else if (color_mode_string == "height") {
            return ColorMode::kHeight;
        } else if (color_mode_string == "normals") {
            return ColorMode::kNormals;
        } else {  // Default case is height.
            return ColorMode::kHeight;
        }
    }
}

inline void generateVoxbloxMeshMsg(int ch, MeshLayer* mesh_layer, voxblox_msgs::Mesh* mesh_msg) {
    if (mesh_layer == NULL) {
        fprintf(stderr, "mesh_layer is null\n");
        return;
    }
    if (mesh_msg == NULL) {
        fprintf(stderr, "mesh_msg is null\n");
        return;
    }

    voxblox::Mesh connected_mesh;
    mesh_layer->getMesh(&connected_mesh);

    static mesh_metadata_t meta;

    //Setup the metadata
    meta.magic_number = MESH_MAGIC_NUMBER;
    meta.timestamp_ns = monotonic_time();
    meta.size_bytes   = 0;
    meta.num_vertices = connected_mesh.vertices.size();
    meta.size_bytes += connected_mesh.vertices.size() * sizeof(mesh_vertex_t);

    //Allocate the memory
    void *data = malloc(meta.size_bytes + sizeof(mesh_metadata_t));
    memcpy (data, &meta, sizeof(mesh_metadata_t));
    char *current = (char *)data + sizeof(mesh_metadata_t);

    ColorMode color_mode = getColorModeFromString(COLOR_MODE);

    for (size_t j = 0; j < connected_mesh.vertices.size(); j++) {

        mesh_vertex_t *vertex = (mesh_vertex_t *)current;
        current += sizeof(mesh_vertex_t);

        // push up our vertices
        vertex->x = connected_mesh.vertices[j].x();
        vertex->y = connected_mesh.vertices[j].y();
        vertex->z = connected_mesh.vertices[j].z();

        // push up our colors
        getVertexColor(vertex, connected_mesh, j, color_mode);

        // push up our normals
        vertex->normal_x = connected_mesh.normals[j].x();
        vertex->normal_y = connected_mesh.normals[j].y();
        vertex->normal_z = connected_mesh.normals[j].z();
    }
    pipe_server_write(ch, data, meta.size_bytes + sizeof(mesh_metadata_t));

    free(data);
}

// A hash function used to hash a pair of any kind
struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2>& p) const
    {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};

inline void createCostmapFromLayer(const Layer<EsdfVoxel>& layer, const unsigned int &free_plane_index, float &free_plane_val, std::unordered_map<std::pair<float, float>, float, hash_pair>& cost_map, bool &only_updates)
{
    BlockIndexList blocks;
    if (only_updates) layer.getAllUpdatedBlocks(Update::kMap, &blocks);
    else {
        cost_map.clear();
        layer.getAllAllocatedBlocks(&blocks);
    }
    
    if (blocks.size() == 0) return;

    // Cache layer settings.
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    // Iterate over all blocks.
    for (const BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<EsdfVoxel>& block = layer.getBlockByIndex(index);

        // Only keep blocks that are at this height
        Point origin = block.origin();
        if (std::abs(origin(free_plane_index) - free_plane_val) > block.block_size()) {
            continue;
        }

        for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            // not if the original gets modified by the cast
            float x = coord.x();
            float y = coord.y();
            std::pair<float, float> curr_coords = std::make_pair(x, y);
            float distance = 10.0;

            const EsdfVoxel voxel = block.getVoxelByLinearIndex(linear_index);
            if (std::abs(coord(free_plane_index) - free_plane_val) <= block.voxel_size()) {
                if (voxel.observed || voxel.hallucinated) {
                    distance = (float)std::abs(voxel.distance);
                } 
                
                cost_map[curr_coords] = distance;
            }
        }
    }
    return;
}

inline void create2DCostmap(const Layer<EsdfVoxel>& layer, float &start_height, std::unordered_map<std::pair<float, float>, float, hash_pair>& cost_map, bool &only_updates)
{
    static const unsigned int free_plane_index = 2;

    if (std::remainder(start_height, layer.voxel_size()) < 1e-6f) {
        start_height += layer.voxel_size() / 2.0f;
    }
    createCostmapFromLayer(layer, free_plane_index, start_height, cost_map, only_updates);

    return;
}

}  // namespace voxblox

#endif  // VOXBLOX_MESH_VIS_H_
