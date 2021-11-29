#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/merge_integration.h>
#include "voxblox_skeleton/skeleton_generator.h"

namespace voxblox {

class SkeletonizerNode {
 public:
  SkeletonizerNode(std::shared_ptr<voxblox::EsdfMap> esdf_map){
    esdf_map_ = esdf_map;
  }

  // Initialize the node.
  void init();

  // Make a skeletor!!!
  void skeletonize(Layer<EsdfVoxel>* esdf_layer, voxblox::Pointcloud* skeleton,
                   std::vector<float>* distances);

 private:
  std::shared_ptr<voxblox::EsdfMap> esdf_map_;

  SkeletonGenerator skeleton_generator_;
};

}  // namespace voxblox
