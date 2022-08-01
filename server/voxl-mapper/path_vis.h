#ifndef PATH_VIS_H_
#define PATH_VIS_H_

#include <stdint.h>
#include <vector>
#include <stdlib.h>
#include <cstring>
#include <modal_pipe.h>

#define PATH_VIS_MAGIC_NUMBER (0x564F584C)

#define PATH_VIS_LINE 0       // Waypoints joined by a line
#define PATH_VIS_TRAJECTORY 1 // Polynomial described by a set of samples along path
#define PATH_VIS_TREE 2       // RRT tree
#define PATH_VIS_POINTS 3     // Points

// struct containing all relevant metadata for path visualization
typedef struct path_vis_meta_t
{
    uint32_t magic_number;
    uint32_t n_points;
    uint32_t format;
    char name[32];
} __attribute__((packed)) path_vis_meta_t;

typedef struct path_vis_t
{
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} __attribute__((packed)) path_vis_t;

inline void generateAndSendPath(int ch, const std::vector<path_vis_t> &points, uint32_t format, std::string name)
{
    static path_vis_meta_t meta;

    // Setup the metadata
    meta.magic_number = PATH_VIS_MAGIC_NUMBER;
    meta.format = format;
    meta.n_points = points.size();

    if (meta.n_points == 0)
        return;

    int bytes_to_copy = std::min(sizeof(meta.name), name.size());
    memset(meta.name, '\0', sizeof(meta.name));
    memcpy(meta.name, name.c_str(), bytes_to_copy);

    int header_size = sizeof(path_vis_meta_t);
    int data_size = meta.n_points * sizeof(path_vis_t);
    int total_size = header_size + data_size;

    // Allocate the memory
    void *data = malloc(total_size);
    memcpy(data, &meta, header_size);
    memcpy((char *)data + header_size, points.data(), data_size);

    pipe_server_write(ch, data, total_size);
    free(data);
}

#endif