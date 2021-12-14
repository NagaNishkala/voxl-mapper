#ifndef VOXL_MAPPER_H_
#define VOXL_MAPPER_H_

#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <mav_local_planner/mav_local_planner.h>

#include "voxl_cutils.h"
#include "mesh_vis.h"
#include "rc_transform_ringbuf.h"

namespace voxblox
{

class TsdfServer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TsdfServer(const TsdfMap::Config &config, const TsdfIntegratorBase::Config &integrator_config,
                const MeshIntegratorConfig &mesh_config, bool debug, bool timing);
    virtual ~TsdfServer() {}

    // rrt planner
    bool maiRRT(Eigen::Vector3d start_pose, Eigen::Vector3d goal_pose, std::shared_ptr<EsdfMap> esdf_map_ptr, mav_trajectory_generation::Trajectory* path_to_follow);
    bool followPath();

    /// mpa callbacks
    static void _pc_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
    static void _pc_helper_cb(__attribute__((unused)) int ch, char *data, int bytes, void *context);
    static void _pc_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
    static void _vio_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
    static void _vio_helper_cb(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context);
    static void _vio_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
    static void _control_pipe_cb(__attribute__((unused)) int ch, char* string, int bytes, __attribute__((unused)) void* context);

    /// general mpa
    int initMPA();
    void closeMPA();

    /// timing
    uint64_t rc_nanos_thread_time(void);
    uint64_t rc_nanos_monotonic_time(void);

    /// integration
    void integratePointcloud(const Transformation &T_G_C,
                             const Pointcloud &ptcloud_C, const Colors &colors,
                             const bool is_freespace_pointcloud = false);

    /// Incremental update
    virtual void updateEsdf(bool clear_updated_flag);
    virtual void updateMesh();

    /// Pubs
    void publish2DCostmap();

    /// IO
    virtual bool saveMesh();                                            // mesh to .ply file
    virtual bool saveMap(std::string tsdf_path, std::string esdf_path); // saves tsdf + esdf maps
    virtual bool loadMap(std::string tsdf_path, std::string esdf_path); // loads tsdf + esdf maps


    /// getters
    std::shared_ptr<EsdfMap> getEsdfMapPtr() { return esdf_map_; }
    std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }
    std::shared_ptr<const TsdfMap> getTsdfMapPtr() const { return tsdf_map_; }

    /// clears map
    virtual void clear();

    // general public params
    bool en_debug;
    bool en_timing;

protected:
    /// boolean showing if we are planning, used to stop other background processes
    bool planning = false;

    /// base costmap built, can do incremental updates from now on (when true)
    bool costmap_updates_only;

    // Maps and integrators.
    std::shared_ptr<TsdfMap> tsdf_map_;
    std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;

    std::shared_ptr<EsdfMap> esdf_map_;
    std::unique_ptr<EsdfIntegrator> esdf_integrator_;

    // Mesh accessories.
    std::shared_ptr<MeshLayer> mesh_layer_;
    std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;

    // save paths
    std::string str_esdf_save_path;
    std::string str_tsdf_save_path;
    std::string str_mesh_save_path;

    /// MaiRRT
    Eigen::Vector3d curr_pose;
    Eigen::Vector3d start_pose;
    Eigen::Vector3d goal_pose;

    // paths
    mav_trajectory_generation::Trajectory path_to_follow;

    // costmap
    std::unordered_map<std::pair<double, double>, double, hash_pair> cost_map;
};

} // namespace voxblox

#endif // VOXL_MAPPER_H_
