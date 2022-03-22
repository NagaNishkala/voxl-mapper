#ifndef VOXL_MAPPER_CONVERSIONS_INL_H_
#define VOXL_MAPPER_CONVERSIONS_INL_H_

namespace voxblox
{

template <typename VoxelType>
void serializeLayerAsMsg(const Layer<VoxelType> &layer, const bool only_updated,
                            voxblox_msgs::Layer *msg,
                            const MapDerializationAction &action){
    // sanity check
    if (msg == nullptr){
        fprintf(stderr, "msg is null: %s\n", __FUNCTION__);
        return;
    }

    msg->voxels_per_side = layer.voxels_per_side();
    msg->voxel_size = layer.voxel_size();

    msg->layer_type = getVoxelType<VoxelType>();
    BlockIndexList block_list;
    if (only_updated){
        layer.getAllUpdatedBlocks(Update::kMap, &block_list);
    }
    else{
        layer.getAllAllocatedBlocks(&block_list);
    }

    msg->action = static_cast<uint8_t>(action);

    voxblox_msgs::Block block_msg;
    msg->blocks.reserve(block_list.size());
    for (const BlockIndex &index : block_list){
        block_msg.x_index = index.x();
        block_msg.y_index = index.y();
        block_msg.z_index = index.z();

        std::vector<uint32_t> data;
        layer.getBlockByIndex(index).serializeToIntegers(&data);

        block_msg.data = data;
        msg->blocks.push_back(block_msg);
    }
}

template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer &msg, Layer<VoxelType> *layer){
    // sanity check
    if (layer == nullptr){
        fprintf(stderr, "layer is null: %s\n", __FUNCTION__);
        return false;
    }
    return deserializeMsgToLayer<VoxelType>(msg, static_cast<MapDerializationAction>(msg.action), layer);
}

template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer& msg, const MapDerializationAction& action,
                           Layer<VoxelType>* layer) {
    // sanity checks
    if (layer == nullptr){
        fprintf(stderr, "layer is null: %s\n", __FUNCTION__);
        return false;
    }
    if (getVoxelType<VoxelType>().compare(msg.layer_type) != 0) {
        return false;
    }

    // So we also need to check if the sizes match. If they don't, we can't
    // parse this at all.
    constexpr double kVoxelSizeEpsilon = 1e-5;
    if (msg.voxels_per_side != layer->voxels_per_side() ||
        std::abs(msg.voxel_size - layer->voxel_size()) > kVoxelSizeEpsilon) {
        fprintf(stderr, "Sizes dont match: %s\n", __FUNCTION__);
        return false;
    }

    if (action == MapDerializationAction::kReset) {
        printf("Resetting current layer\n");
        layer->removeAllBlocks();
    }

    for (const voxblox_msgs::Block& block_msg : msg.blocks) {
        BlockIndex index(block_msg.x_index, block_msg.y_index, block_msg.z_index);

        // Either we want to update an existing block or there was no block there
        // before.
        if (action == MapDerializationAction::kUpdate || !layer->hasBlock(index)) {
            // Create a new block if it doesn't exist yet, or get the existing one
            // at the correct block index.
            typename Block<VoxelType>::Ptr block_ptr =
                layer->allocateBlockPtrByIndex(index);

            std::vector<uint32_t> data = block_msg.data;
            block_ptr->deserializeFromIntegers(data);
        }
        else if (action == MapDerializationAction::kMerge) {
            typename Block<VoxelType>::Ptr old_block_ptr =
                layer->getBlockPtrByIndex(index);

            typename Block<VoxelType>::Ptr new_block_ptr(new Block<VoxelType>(
                old_block_ptr->voxels_per_side(), old_block_ptr->voxel_size(),
                old_block_ptr->origin()));

            std::vector<uint32_t> data = block_msg.data;
            new_block_ptr->deserializeFromIntegers(data);

            old_block_ptr->mergeBlock(*new_block_ptr);
        }
    }

    switch (action) {
        case MapDerializationAction::kReset:
            if (layer->getNumberOfAllocatedBlocks() != msg.blocks.size()){
                fprintf(stderr, "allocated blocks is not equal to the block msg size: %s\n", __FUNCTION__);
                return false;
            }
            break;
        case MapDerializationAction::kUpdate:
        // Fall through intended.
        case MapDerializationAction::kMerge:
            if (layer->getNumberOfAllocatedBlocks() < msg.blocks.size()){
                fprintf(stderr, "allocated blocks is not less than the block msg size: %s\n", __FUNCTION__);
                return false;
            }
            break;
    }
    return true;
}

} // namespace voxblox

#endif // VOXL_MAPPER_CONVERSIONS_INL_H_
