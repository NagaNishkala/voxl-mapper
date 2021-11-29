#include "voxblox_skeleton/skeletonizer.h"


namespace voxblox {

void SkeletonizerNode::init()
{
  // Skeletonize????
  voxblox::Pointcloud pointcloud;
  std::vector<float> distances;
  skeletonize(esdf_map_->getEsdfLayerPtr(), &pointcloud, &distances);
}

void SkeletonizerNode::skeletonize(Layer<EsdfVoxel>* esdf_layer, voxblox::Pointcloud* pointcloud, std::vector<float>* distances)
{
  skeleton_generator_.setEsdfLayer(esdf_layer);

  skeleton_generator_.setMinSeparationAngle(0.78);

  skeleton_generator_.setGenerateByLayerNeighbors(false);

//   skeleton_generator_.setNumNeighborsForEdge(num_neighbors_for_edge);

  skeleton_generator_.setMinGvdDistance(0);

  skeleton_generator_.generateSkeleton();

  skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(pointcloud,
                                                                   distances);

  printf("Edge Pointcloud With Distances size: %lu", pointcloud->size());


  printf("Finished generating skeleton.");

  skeleton_generator_.generateSparseGraph();
  printf("Finished generating sparse graph.");

  std::cout << "Total Timings: " << std::endl << timing::Timing::Print();

  // Now visualize the graph.
  //const SparseSkeletonGraph& graph = skeleton_generator_.getSparseGraph();
}

}  // namespace voxblox
