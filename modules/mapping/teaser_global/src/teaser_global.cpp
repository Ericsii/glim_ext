#include <teaser_global.hpp>

#include <pcl/io/pcd_io.h>

#include <glim/util/config.hpp>
#include <glim_ext/util/config_ext.hpp>

#include <teaser/registration.h>

namespace glim {

namespace fs = boost::filesystem;

TEASERGlobal::TEASERGlobal() : logger_(create_module_logger("teaser_global")) {
  logger_->info("initializing TEASER global");
  const auto config_path = GlobalConfigExt::get_config_path("config_teaser_global");
  logger_->info("teaser_global_config_path: {}", config_path);

  Config config(config_path);

  map_path_ = config.param<std::string>("teaser_global", "map_path", "");

  if (!fs::exists(map_path_)) {
    logger_->error("map_path does not exist!", map_path_);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (fs::extension(map_path_) == ".pcd") {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *map_cloud) == -1) {
      logger_->error("Failed to load PCD file: {}", map_path_);
    }
  } else {
    logger_->error("Unsupported map file extension: {}", fs::extension(map_path_));
  }

  map_points_.resize(3, map_cloud->size());
  for (auto i = 0; i < map_cloud->size(); ++i) {
    const auto& point = map_cloud->points[i];
    map_points_.col(i) << point.x, point.y, point.z;
  }

  GlobalMappingCallbacks::on_insert_submap.add([this](const SubMap::ConstPtr& submap) { new_submaps_queue_.push_back(submap); });
  GlobalMappingCallbacks::on_smoother_update.add(
    [this](gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) { on_smoother_update(isam2, new_factors, new_values); });

  kill_switch_ = false;
  thread_ = std::thread([this] { global_localization_task(); });
}

TEASERGlobal::~TEASERGlobal() {
  kill_switch_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void TEASERGlobal::on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
  auto factors = factors_.get_all_and_clear();
  if (!factors.empty()) {
    logger_->debug("insert {} TEASER global factors", factors.size());
    new_factors.add(factors);
  }
}

void TEASERGlobal::global_localization_task() {
  logger_->info("starting TEASER global localization thread");

  while (!kill_switch_) {
    const auto new_submaps = new_submaps_queue_.get_all_and_clear();
    if (new_submaps.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    for (const auto& submap : new_submaps) {
      const auto T_world_origin = submap->T_world_origin;

      Eigen::Matrix<double, 3, Eigen::Dynamic> submap_points{3, submap->frame->size()};
      for (size_t i = 0; i < submap->frame->size(); ++i) {
        const auto& point = submap->frame->points[i];
        submap_points.col(i) << point.x(), point.y(), point.z();
      }
    }
  }
}

}  // namespace glim