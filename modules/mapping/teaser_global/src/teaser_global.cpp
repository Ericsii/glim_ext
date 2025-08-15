#include <teaser_global.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

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
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *map_cloud) != 0) {
      logger_->error("Failed to load PCD file: {}", map_path_);
    }
  } else {
    logger_->error("Unsupported map file extension: {}", fs::extension(map_path_));
  }

  downsample_convert_map_points(map_cloud);

  // map_points_.resize(3, map_cloud->size());
  // for (auto i = 0; i < map_cloud->size(); ++i) {
  //   const auto& point = map_cloud->points[i];
  //   map_points_.col(i) << point.x, point.y, point.z;
  // }

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

  if (map_points_.cols() == 0) {
    logger_->error("Map points are empty, cannot perform global localization.");
    return;
  }

  while (!kill_switch_) {
    const auto new_submaps = new_submaps_queue_.get_all_and_clear();
    if (new_submaps.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    const auto& submap = new_submaps.back();

    Eigen::Matrix<double, 3, Eigen::Dynamic> submap_points{3, submap->frame->size()};
    for (size_t i = 0; i < submap->frame->size(); ++i) {
      const auto& point = submap->frame->points[i];
      submap_points.col(i) << point.x(), point.y(), point.z();
    }

    // Run TEASER++ registration
    teaser::RobustRegistrationSolver::Params params;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;

    teaser::RobustRegistrationSolver solver(params);

    logger_->info("submap id: {}. Running TEASER++ registration with {} points.", submap->id, submap_points.cols());
    solver.solve(submap_points, map_points_);  // map_points_ = T_origin_map * submap_points
    logger_->info("submap id: {}. TEASER++ registration finished.", submap->id);

    const auto solution = solver.getSolution();
    if (!solution.valid) {
      logger_->info("submap id: {}. Registration failed.", submap->id);
      continue;
    }

    const auto rotation = solution.rotation;
    const auto translation = solution.translation;

    using gtsam::symbol_shorthand::X;
    gtsam::Pose3 T_origin_map(gtsam::Rot3(rotation), gtsam::Point3(translation(0), translation(1), translation(2)));
    gtsam::Vector6 sigmas;
    sigmas << 0.05, 0.05, 0.05,  // Example: 0.05 radians (approx. 2.8 degrees) uncertainty for Roll, Pitch, Yaw
      0.2, 0.2, 0.2;             // Example: 0.2 meters (20 cm) uncertainty for X, Y, Z translation
    auto base = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    auto noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), base);

    auto factor = std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(submap->id), T_origin_map, noise_model);

    factors_.push_back(std::move(factor));
  }
}

void TEASERGlobal::downsample_convert_map_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
  const int target_points = 10000;
  if (!point_cloud || point_cloud->empty()) {
    logger_->error("Input map point cloud is empty.");
    return;
  }
  logger_->info("Loaded map pointcloud with {} points.", point_cloud->size());

  // Downsample if necessary
  if (point_cloud->size() > target_points) {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    const double leaf_size = 0.1;  // Set voxel grid to 0.1m
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.setInputCloud(point_cloud);
    sor.filter(*point_cloud);
  }
  if (point_cloud->size() > target_points) {
    pcl::RandomSample<pcl::PointXYZ> random_sample;
    random_sample.setInputCloud(point_cloud);
    random_sample.setSample(target_points);
    random_sample.filter(*point_cloud);
  }

  logger_->info("Downsampled map pointcloud to {} points.", point_cloud->size());

  // Convert to Eigen::Matrix<double, 3, N>
  map_points_.resize(3, point_cloud->size());
  for (size_t i = 0; i < point_cloud->size(); ++i) {
    const auto& pt = point_cloud->points[i];
    map_points_.col(i) << static_cast<double>(pt.x), static_cast<double>(pt.y), static_cast<double>(pt.z);
  }
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::TEASERGlobal();
}