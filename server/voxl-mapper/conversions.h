#ifndef VOXBLOX_ROS_CONVERSIONS_H_
#define VOXBLOX_ROS_CONVERSIONS_H_

#include <voxblox/msgs/voxblox_msgs.h>

namespace voxblox
{

enum class MapDerializationAction : uint8_t{
    kUpdate = 0u,
    kMerge = 1u,
    kReset = 2u
};

inline void colorVoxbloxToMsg(const Color& color,
                              float color_msg[][4]) {
  if(color_msg == nullptr){
      fprintf(stderr, "color_msg is null: %s\n", __FUNCTION__);
  }
  color_msg[0][0] = color.r / 255.0;
  color_msg[0][1] = color.g / 255.0;
  color_msg[0][2] = color.b / 255.0;
  color_msg[0][3] = color.a / 255.0;
}

inline uint64_t monotonic_time(){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000) + ts.tv_nsec;
}

// Declarations
template <typename VoxelType>
void serializeLayerAsMsg(const Layer<VoxelType> &layer, const bool only_updated, voxblox_msgs::Layer *msg,
                         const MapDerializationAction &action = MapDerializationAction::kUpdate);
/**
 * Returns true if could parse the data into the existing layer (all parameters
 * are compatible), false otherwise.
 * This function will use the deserialization action suggested by the layer
 * message.
 */
template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer &msg, Layer<VoxelType> *layer);

template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer &msg, const MapDerializationAction &action,
                           Layer<VoxelType> *layer);

} // namespace voxblox

#endif // VOXBLOX_ROS_CONVERSIONS_H_

#include "conversions_inl.h"
