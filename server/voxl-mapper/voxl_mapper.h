#ifndef VOXL_MAPPER_H_
#define VOXL_MAPPER_H_

#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/planning_utils.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <voxl_planner.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/layer.h>

#include "voxl_cutils.h"
#include "mesh_vis.h"
#include "rc_transform_ringbuf.h"
#include "config_file.h"

namespace voxblox
{

    class TsdfServer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TsdfServer(const TsdfMap::Config &config, const TsdfIntegratorBase::Config &integrator_config,
                   const MeshIntegratorConfig &mesh_config, bool debug, bool timing);
        virtual ~TsdfServer() {}

        /// mpa callbacks
        static void _pc_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
        static void _tof_helper_cb(__attribute__((unused)) int ch, char *data, int bytes, void *context);
        static void _pc_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
        static void _vio_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
        static void _vio_helper_cb(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context);
        static void _vio_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
        static void _control_pipe_cb(__attribute__((unused)) int ch, char *msg, int bytes, __attribute__((unused)) void *context);
        static void depth_helper0(__attribute__((unused)) int ch, point_cloud_metadata_t meta, void *data, void *context);
        static void depth_helper1(__attribute__((unused)) int ch, point_cloud_metadata_t meta, void *data, void *context);
        static void depth_helper2(__attribute__((unused)) int ch, point_cloud_metadata_t meta, void *data, void *context);
        static void depth_helper3(__attribute__((unused)) int ch, point_cloud_metadata_t meta, void *data, void *context);
        static void _stereo_pc_helper_cb(int ch, point_cloud_metadata_t &meta, rc_tf_t &tf_cam_wrt_body, int64_t &fixed_ts_dif, int &aligned_index, void *data, void *context);
        /// general mpa
        int initMPA();
        void closeMPA();

        /// integration
        void integratePointcloud(const Transformation &T_G_C,
                                 const Pointcloud &ptcloud_C, const Colors &colors,
                                 const bool is_freespace_pointcloud = false);

        void addNewRobotPositionToEsdf(float x, float y, float z);
        
        bool getRobotPose(rc_tf_t &tf_body_wrt_fixed, int64_t ts);

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

        /// visual update func
        void visual_updates_thread_worker();

        // general public params
        bool en_debug;
        bool en_timing;

        // helpers
        rc_tf_t get_rc_tf_t(int ch);
        int64_t get_dif_per_frame(int ch);
        int get_index_by_ch(int ch);

        static pthread_mutex_t pose_mutex;
        static pthread_mutex_t esdf_mutex;
        static pthread_mutex_t tsdf_mutex;
        static rc_tfv_ringbuf_t buf;

    protected:
        VoxlPlanner planner_;

        /// base costmap built, can do incremental updates from now on (when true)
        bool costmap_updates_only;
        float costmap_height = 50.0;

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

        std::thread visual_updates_thread;
        std::atomic<bool> keep_updating;

        // costmap
        std::unordered_map<std::pair<float, float>, float, hash_pair> cost_map;
    };

} // namespace voxblox

#endif // VOXL_MAPPER_H_
