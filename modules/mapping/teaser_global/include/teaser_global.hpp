#include <mutex>
#include <thread>
#include <memory>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <spdlog/logger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/mapping/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

class TEASERGlobal : public ExtensionModule {
public:
  explicit TEASERGlobal();
  ~TEASERGlobal() override;

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values);

  void global_localization_task();

  void downsample_convert_map_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

private:
  // ConcurrentVector<EstimationFrame::ConstPtr> odom_frames_queue_;
  ConcurrentVector<SubMap::ConstPtr> new_submaps_queue_;

  ConcurrentVector<gtsam::PriorFactor<gtsam::Pose3>::shared_ptr> factors_;

  Eigen::Matrix<double, 3, Eigen::Dynamic> map_points_;

  int frame_count_ = 0;
  std::string map_path_;

  std::atomic_bool kill_switch_;
  std::thread thread_;

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace glim