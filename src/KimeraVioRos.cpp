/* @file   KimeraVioRos.cpp
 * @brief  ROS Wrapper for Kimera-VIO
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/KimeraVioRos.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <future>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <ctime>

// Still need gflags for parameters in VIO
#include <gflags/gflags.h>
#include <glog/logging.h>

// Dependencies from ROS
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

// Dependencies from VIO
#include <kimera-vio/pipeline/MonoImuPipeline.h>
#include <kimera-vio/pipeline/RgbdImuPipeline.h>
#include <kimera-vio/pipeline/StereoImuPipeline.h>
#include <kimera-vio/utils/Timer.h>

// Dependencies from this repository
#include "kimera_vio_ros/RosBagDataProvider.h"
#include "kimera_vio_ros/RosDataProviderInterface.h"
#include "kimera_vio_ros/RosOnlineDataProvider.h"
#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

namespace {

gtsam::Matrix6 poseCovarianceFromMatrix(const gtsam::Matrix& state_covariance) {
  gtsam::Matrix6 pose_cov = gtsam::Matrix6::Zero();
  if (state_covariance.rows() >= 6 && state_covariance.cols() >= 6) {
    pose_cov = gtsam::sub(state_covariance, 0, 6, 0, 6);
  }
  return pose_cov;
}

template <typename VisualizerT>
void drawRawPoseCovariance6x6(VisualizerT* visualizer,
                              const std::string& base_path,
                              const gtsam::Matrix& source_covariance) {
  if (!visualizer) {
    return;
  }

  const bool has_6x6 =
      source_covariance.rows() >= 6 && source_covariance.cols() >= 6;
  visualizer->drawScalar(base_path + "/valid/has_6x6", has_6x6 ? 1.0 : 0.0);
  if (!has_6x6) {
    return;
  }

  const gtsam::Matrix6 pose_covariance =
      gtsam::sub(source_covariance, 0, 6, 0, 6);
  const char* labels[6] = {"rot_x",   "rot_y",   "rot_z",
                           "trans_x", "trans_y", "trans_z"};

  bool all_finite = true;
  for (size_t row = 0u; row < 6u; ++row) {
    for (size_t col = 0u; col < 6u; ++col) {
      const double value = pose_covariance(row, col);
      all_finite = all_finite && std::isfinite(value);
      visualizer->drawScalar(base_path + "/matrix/" + labels[row] + "__" +
                                 labels[col],
                             value);
    }
    visualizer->drawScalar(base_path + "/diag/" + labels[row],
                           pose_covariance(row, row));
  }

  const gtsam::Matrix6 symmetric_covariance =
      0.5 * (pose_covariance + pose_covariance.transpose());
  visualizer->drawScalar(base_path + "/valid/all_finite",
                         all_finite ? 1.0 : 0.0);
  visualizer->drawScalar(base_path + "/summary/trace",
                         pose_covariance.trace());
  visualizer->drawScalar(base_path + "/summary/frobenius_norm",
                         pose_covariance.norm());
  visualizer->drawScalar(base_path + "/summary/asymmetry_frobenius_norm",
                         (pose_covariance - pose_covariance.transpose()).norm());
  visualizer->drawScalar(base_path + "/block_norm/rotation_3x3",
                         pose_covariance.block<3, 3>(0, 0).norm());
  visualizer->drawScalar(base_path + "/block_norm/translation_3x3",
                         pose_covariance.block<3, 3>(3, 3).norm());
  visualizer->drawScalar(base_path + "/block_norm/rotation_translation_3x3",
                         pose_covariance.block<3, 3>(0, 3).norm());

  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eig(symmetric_covariance);
  if (eig.info() != Eigen::Success) {
    visualizer->drawScalar(base_path + "/valid/eigen_success", 0.0);
    return;
  }
  visualizer->drawScalar(base_path + "/valid/eigen_success", 1.0);
  const gtsam::Vector6 eigenvalues = eig.eigenvalues();
  for (size_t i = 0u; i < 6u; ++i) {
    visualizer->drawScalar(base_path + "/eigenvalues/lambda_" +
                               std::to_string(i),
                           eigenvalues(i));
  }
  visualizer->drawScalar(base_path + "/eigenvalues/min", eigenvalues.minCoeff());
  visualizer->drawScalar(base_path + "/eigenvalues/max", eigenvalues.maxCoeff());
}

Eigen::Matrix3d translationCovarianceFromPoseCovariance(
    const gtsam::Matrix& pose_covariance) {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  if (pose_covariance.rows() >= 6 && pose_covariance.cols() >= 6) {
    covariance = pose_covariance.block<3, 3>(3, 3);
  }
  return covariance;
}

std::string sanitizeCsvToken(std::string token) {
  std::replace(token.begin(), token.end(), ',', '_');
  std::replace(token.begin(), token.end(), ' ', '_');
  return token.empty() ? "na" : token;
}

std::string vector6ToToken(const gtsam::Vector6& vector) {
  std::ostringstream oss;
  oss << "v[";
  for (size_t i = 0u; i < 6u; ++i) {
    if (i > 0u) {
      oss << ";";
    }
    oss << vector(i);
  }
  oss << "]";
  return oss.str();
}

std::string matrix6ToToken(const gtsam::Matrix6& matrix) {
  std::ostringstream oss;
  oss << "m[";
  for (size_t r = 0u; r < 6u; ++r) {
    if (r > 0u) {
      oss << "|";
    }
    for (size_t c = 0u; c < 6u; ++c) {
      if (c > 0u) {
        oss << ";";
      }
      oss << matrix(r, c);
    }
  }
  oss << "]";
  return oss.str();
}

double minEigenvalueSymmetric(const gtsam::Matrix6& matrix) {
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eig(
      0.5 * (matrix + matrix.transpose()));
  if (eig.info() != Eigen::Success) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return eig.eigenvalues().minCoeff();
}

double poseErrorNorm(const gtsam::Pose3& lhs, const gtsam::Pose3& rhs) {
  try {
    return gtsam::Pose3::Logmap(lhs.inverse() * rhs).norm();
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

std::string keyTokenForAgent(uint8_t source_agent, uint32_t pose_index) {
  std::ostringstream oss;
  oss << "p:";
  if (std::isprint(static_cast<unsigned char>(source_agent))) {
    oss << static_cast<char>(source_agent);
  } else {
    oss << static_cast<int>(source_agent);
  }
  oss << ":" << pose_index;
  return oss.str();
}

uint8_t resolveAgentId(const std::string& agent_id) {
  return agent_id.empty() ? static_cast<uint8_t>('k')
                          : static_cast<uint8_t>(agent_id.front());
}

std::string makeRerunRecordingId(const std::string& prefix) {
  std::time_t now = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&now, &local_time);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
  return prefix + "_" + buffer;
}

std::string getDockerGatewayIp() {
  std::ifstream route_file("/proc/net/route");
  std::string line;
  std::getline(route_file, line);
  while (std::getline(route_file, line)) {
    std::istringstream iss(line);
    std::string iface;
    std::string destination;
    std::string gateway;
    unsigned int flags = 0u;
    if (!(iss >> iface >> destination >> gateway >> std::hex >> flags)) {
      continue;
    }
    if (destination != "00000000" || gateway.size() != 8u) {
      continue;
    }

    const unsigned long raw_gateway = std::stoul(gateway, nullptr, 16);
    std::ostringstream ip;
    ip << (raw_gateway & 0xfful) << "." << ((raw_gateway >> 8u) & 0xfful)
       << "." << ((raw_gateway >> 16u) & 0xfful) << "."
       << ((raw_gateway >> 24u) & 0xfful);
    return ip.str();
  }
  return "";
}

std::string defaultRerunHost() {
  const char* env_host = std::getenv("CBSMS_RERUN_HOST");
  if (env_host != nullptr && std::string(env_host).empty() == false) {
    return env_host;
  }

  const std::string gateway_ip = getDockerGatewayIp();
  if (!gateway_ip.empty()) {
    return "rerun+http://" + gateway_ip + ":9876/proxy";
  }

  return "rerun+http://host.docker.internal:9876/proxy";
}

template <typename Array6>
gtsam::Vector6 toVector6(const Array6& values) {
  gtsam::Vector6 vector = gtsam::Vector6::Zero();
  for (size_t i = 0u; i < 6u; ++i) {
    vector(i) = values[i];
  }
  return vector;
}

template <typename Array36>
gtsam::Matrix6 toMatrix6(const Array36& values) {
  gtsam::Matrix6 matrix = gtsam::Matrix6::Zero();
  for (size_t r = 0u; r < 6u; ++r) {
    for (size_t c = 0u; c < 6u; ++c) {
      matrix(r, c) = values[r * 6u + c];
    }
  }
  return matrix;
}

template <typename Array6>
void fromVector6(const gtsam::Vector6& vector, Array6* values) {
  CHECK_NOTNULL(values);
  for (size_t i = 0u; i < 6u; ++i) {
    (*values)[i] = vector(i);
  }
}

template <typename Array36>
void fromMatrix6(const gtsam::Matrix6& matrix, Array36* values) {
  CHECK_NOTNULL(values);
  for (size_t r = 0u; r < 6u; ++r) {
    for (size_t c = 0u; c < 6u; ++c) {
      (*values)[r * 6u + c] = matrix(r, c);
    }
  }
}

}  // namespace

#define MAKE_CONFIG_FILEPATH(dir_to_use, config_name) \
  dir_to_use + '/' + VioParams::k##config_name

KimeraVioRos::KimeraVioRos()
    : nh_private_("~"),
      vio_params_(nullptr),
      vio_pipeline_(nullptr),
      use_lcd_registration_server_(false),
      ros_display_(nullptr),
      ros_visualizer_(nullptr),
      data_provider_(nullptr),
      restart_vio_pipeline_srv_(),
      restart_vio_pipeline_(false) {
  // Add rosservice to restart VIO pipeline if requested.
  restart_vio_pipeline_srv_ = nh_private_.advertiseService(
      "restart_kimera_vio", &KimeraVioRos::restartKimeraVio, this);

  CHECK(nh_private_.getParam("use_rviz", use_rviz_));

  nh_private_.getParam("use_lcd_registration_server",
                       use_lcd_registration_server_);

  // Parse VIO parameters
  std::string params_path;
  CHECK(nh_private_.getParam("params_folder_path", params_path));
  CHECK(!params_path.empty());

  std::string sensor_params_path;
  nh_private_.getParam("sensor_params_folder_path", sensor_params_path);
  if (sensor_params_path.empty()) {
    VLOG(1) << "Using provided parameter path for every configuration file";
    vio_params_ = std::make_shared<VioParams>(params_path);
  } else {
    VLOG(1) << "Using split parameter paths for general and sensor parameters";
    vio_params_ = std::make_shared<VioParams>(params_path, sensor_params_path);
  }

  int cbs_forward_queue_limit = 800;
  nh_private_.param<int>(
      "cbs_belief_forward_queue_limit", cbs_forward_queue_limit, 800);
  external_beliefs_queue_limit_ =
      static_cast<size_t>(std::max(1, cbs_forward_queue_limit));

  nh_private_.param(
      "cbs_belief_bridge_enable", headless_cbs_belief_bridge_enable_, true);
  nh_private_.param(
      "headless_odometry_publish_enable", headless_odometry_publish_enable_, true);
  nh_private_.param(
      "rerun_visualizer_enable", headless_rerun_visualizer_enable_, false);
  nh_private_.param("rerun_factor_graph_enable",
                    headless_rerun_factor_graph_enable_,
                    true);
  nh_private_.param<std::string>(
      "rerun_recording_id", headless_rerun_recording_id_, "");
  nh_private_.param<std::string>("rerun_host", headless_rerun_host_, "auto");
  if (headless_rerun_host_.empty() || headless_rerun_host_ == "auto") {
    headless_rerun_host_ = defaultRerunHost();
  }
  if (headless_rerun_recording_id_.empty()) {
    ros::param::param<std::string>(
        "/cbsms/rerun_recording_id", headless_rerun_recording_id_, "");
  }
  if (headless_rerun_recording_id_.empty()) {
    headless_rerun_recording_id_ = makeRerunRecordingId("kimera_vio_ros");
  }
  nh_private_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  nh_private_.param<std::string>(
      "base_link_frame_id", base_link_frame_id_, "base_link");
  nh_private_.param<std::string>("cbs_odom_belief_in_topic",
                                 cbs_odom_belief_in_topic_,
                                 "kimera/cbs/odom_belief_in");
  nh_private_.param<std::string>("cbs_odom_belief_out_topic",
                                 cbs_odom_belief_out_topic_,
                                 "kimera/cbs/odom_belief_out");
  nh_private_.param<double>("cbs_belief_receive_start_delay_sec",
                            cbs_belief_receive_start_delay_sec_,
                            0.0);
  if (!std::isfinite(cbs_belief_receive_start_delay_sec_) ||
      cbs_belief_receive_start_delay_sec_ < 0.0) {
    LOG(WARNING) << "Invalid cbs_belief_receive_start_delay_sec="
                 << cbs_belief_receive_start_delay_sec_
                 << "; falling back to 0.";
    cbs_belief_receive_start_delay_sec_ = 0.0;
  }
  nh_private_.param<std::string>("cbs_external_pose_frame_id",
                                 cbs_external_pose_frame_id_,
                                 base_link_frame_id_);
  if (cbs_external_pose_frame_id_.empty()) {
    cbs_external_pose_frame_id_ = base_link_frame_id_;
  }
  std::string cbs_agent_id = "k";
  nh_private_.param<std::string>("cbs_agent_id", cbs_agent_id, "k");
  cbs_agent_id_ = resolveAgentId(cbs_agent_id);

  initializeHeadlessCbsBeliefBridge();
  if (!use_rviz_) {
    initializeHeadlessOdometryPublisher();
    initializeHeadlessRerunVisualizer();
  }
}

#undef MAKE_CONFIG_FILEPATH

KimeraVioRos::~KimeraVioRos() {
  // necessary to clean this before the pipeline disappears (contains a bare
  // pointer to memory that the pipline owns)
  if (lcd_registration_server_) {
    lcd_registration_server_->stop();
    lcd_registration_server_.reset();
  }
}

bool KimeraVioRos::runKimeraVio() {
  {
    std::lock_guard<std::mutex> lock(external_beliefs_mutex_);
    pending_external_odom_beliefs_.clear();
  }

  // First, destroy VIO pipeline, this will in turn call the shutdown of
  // the data provider.
  // NOTE: had the data provider been destroyed before, the vio would be calling
  // the shutdown function of a deleted object, aka segfault.
  if (use_rviz_) {
    VLOG(1) << "Destroy Ros Display.";
    ros_display_.reset();
    ros_visualizer_.reset();

    VLOG(1) << "Creating Ros Display.";
    CHECK(vio_params_);
    ros_display_ = std::make_unique<RosDisplay>();
    ros_visualizer_ = std::make_unique<RosVisualizer>(*vio_params_);
  } else {
    ros_display_ = nullptr;
    ros_visualizer_ = nullptr;
  }

  ros_lcd_visualizer_.reset(new RosLoopClosureVisualizer());

  VLOG(1) << "Destroy Vio Pipeline.";
  vio_pipeline_.reset();

  // Second, destroy dataset parser.
  VLOG(1) << "Destroy Data Provider.";
  data_provider_.reset();

  std::unique_ptr<PreloadedVocab> preloaded_vocab;
  if (FLAGS_use_lcd) {
    preloaded_vocab.reset(new PreloadedVocab());
  }

  // Then, create dataset parser. This must be before vio pipeline bcs
  // the data provider may modify the init gt pose.
  VLOG(1) << "Creating Data Provider.";
  data_provider_ = createDataProvider(*vio_params_);
  CHECK(data_provider_) << "Data provider construction failed.";

  // Then, create Kimera-VIO from scratch.
  VLOG(1) << "Creating Kimera-VIO.";
  if (use_rviz_) {
    CHECK(ros_display_);
    CHECK(ros_visualizer_);
  }

  vio_pipeline_ = nullptr;
  switch (vio_params_->frontend_type_) {
    case VIO::FrontendType::kMonoImu: {
      vio_pipeline_ =
          std::make_unique<MonoImuPipeline>(*vio_params_,
                                            std::move(ros_visualizer_),
                                            std::move(ros_display_),
                                            std::move(preloaded_vocab));
    } break;
    case VIO::FrontendType::kStereoImu: {
      vio_pipeline_ =
          std::make_unique<StereoImuPipeline>(*vio_params_,
                                              std::move(ros_visualizer_),
                                              std::move(ros_display_),
                                              std::move(preloaded_vocab));
    } break;
    case VIO::FrontendType::kRgbdImu: {
      vio_pipeline_ =
          std::make_unique<RgbdImuPipeline>(*vio_params_,
                                            std::move(ros_visualizer_),
                                            std::move(ros_display_),
                                            std::move(preloaded_vocab));
    } break;
    default: {
      LOG(FATAL) << "Unrecognized frontend type: "
                 << VIO::to_underlying(vio_params_->frontend_type_)
                 << ". 0: Mono, 1: Stereo.";
    } break;
  }

  CHECK(vio_pipeline_) << "Vio pipeline construction failed.";
  if (headless_cbs_belief_bridge_enable_ ||
      (!use_rviz_ &&
       (headless_odometry_publish_enable_ || headless_rerun_visualizer_))) {
    vio_pipeline_->registerExternalBackendOutputCallback(
        [this](const BackendOutput::Ptr& output) {
          publishHeadlessBackendOutput(output);
        });
  }
  if (use_lcd_registration_server_) {
    LcdModule* lcd = vio_pipeline_->getLcdModule();
    if (!lcd) {
      LOG(ERROR)
          << "LCD module isn't valid: will not start registration server.";
    } else {
      lcd_registration_server_.reset(new LcdRegistrationServer(lcd));
    }
  }

  // Finally, connect data_provider and vio_pipeline
  VLOG(1) << "Connecting Vio Pipeline and Data Provider.";
  connectVIO();

  // Run
  return spin();
}

bool KimeraVioRos::spin() {
  CHECK(vio_params_);
  CHECK(vio_pipeline_);
  CHECK(data_provider_);

  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (vio_params_->parallel_run_) {
    // TODO(Toni): Technically, we can spare a thread with online dataprovider
    // since we can simply call .start() on the async spinners at the ctor level
    std::future<bool> data_provider_handle =
        std::async(std::launch::async,
                   &VIO::RosDataProviderInterface::spin,
                   CHECK_NOTNULL(data_provider_.get()));
    std::future<bool> vio_viz_handle =
        std::async(std::launch::async,
                   &VIO::Pipeline::spinViz,
                   CHECK_NOTNULL(vio_pipeline_.get()));
    std::future<bool> vio_pipeline_handle =
        std::async(std::launch::async,
                   &VIO::Pipeline::spin,
                   CHECK_NOTNULL(vio_pipeline_.get()));
    // Run while ROS is ok and vio pipeline is not shutdown.
    ros::WallRate rate(20);  // 20 Hz
    while (ros::ok() && !restart_vio_pipeline_) {
      flushExternalOdometryBeliefsToPipeline();

      const auto stats = vio_pipeline_->printStatistics();
      if (!stats.empty()) {
        LOG_EVERY_N(INFO, 20) << stats;
      }

      rate.sleep();

      if (vio_pipeline_->hasFinished() && data_provider_->isShutdown()) {
        break;
      }
    }

    if (!restart_vio_pipeline_) {
      LOG(INFO) << "Shutting down ROS and Kimera-VIO.";
      ros::shutdown();
    } else {
      LOG(INFO) << "Restarting Kimera-VIO.";
    }
    // TODO(Toni): right now vio shutsdown data provider, maybe we should
    // explicitly shutdown data provider: data_provider_->shutdown();
    if (!vio_pipeline_->isShutdown()) vio_pipeline_->shutdown();
    LOG(INFO) << "Joining Kimera-VIO thread.";
    vio_pipeline_handle.get();
    LOG(INFO) << "Kimera-VIO thread joined successfully.";
    LOG(INFO) << "Joining Ros Data Provider thread.";
    data_provider_handle.get();
    LOG(INFO) << "Ros Data Provider thread joined successfully.";
    LOG(INFO) << "Joining RosDisplay thread.";
    is_pipeline_successful = !vio_viz_handle.get();
    LOG(INFO) << "RosDisplay thread joined successfully.";
    if (restart_vio_pipeline_) {
      // Mind that this is a recursive call! As we call this function
      // inside runKimeraVio. Sorry, couldn't find a better way.
      restart_vio_pipeline_ = false;
      LOG(INFO) << "Restarting...";
      return runKimeraVio();
    }
  } else {
    ros::start();
    while (ros::ok() && data_provider_->spin() && vio_pipeline_->spin()) {
      flushExternalOdometryBeliefsToPipeline();

      // TODO(Toni): right now this will loop forwever unless ROS dies or Ctrl+C
      LOG(INFO) << vio_pipeline_->printStatistics();
      vio_pipeline_->spinViz();
    }
    LOG(INFO) << "Shutting down ROS and VIO pipeline.";
    ros::shutdown();
    vio_pipeline_->shutdown();
    is_pipeline_successful = true;
  }
  auto spin_duration = VIO::utils::Timer::toc(tic);
  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";
  LOG(INFO) << "Pipeline successful? "
            << (is_pipeline_successful ? "Yes!" : "No!");
  return is_pipeline_successful;
}

RosDataProviderInterface::UniquePtr KimeraVioRos::createDataProvider(
    const VioParams& vio_params) {
  bool online_run = false;
  CHECK(nh_private_.getParam("online_run", online_run));
  if (online_run) {
    // Running ros online.
    return std::make_unique<RosOnlineDataProvider>(vio_params);
  } else {
    // Parse rosbag.
    auto rosbag_data_provider =
        std::make_unique<RosbagDataProvider>(vio_params);
    rosbag_data_provider->initialize();
    return rosbag_data_provider;
  }
}

void KimeraVioRos::connectVIO() {
  // Register VIO pipeline callbacks
  // Register callback to shutdown data provider in case VIO pipeline
  // shutsdown.
  CHECK(data_provider_);
  CHECK(vio_pipeline_);
  vio_pipeline_->registerShutdownCallback(
      std::bind(&VIO::DataProviderInterface::shutdown,
                std::ref(*CHECK_NOTNULL(data_provider_.get()))));

  // Register Data Provider callbacks
  data_provider_->registerImuSingleCallback(
      std::bind(&VIO::Pipeline::fillSingleImuQueue,
                std::ref(*CHECK_NOTNULL(vio_pipeline_.get())),
                std::placeholders::_1));

  data_provider_->registerImuMultiCallback(
      std::bind(&VIO::Pipeline::fillMultiImuQueue,
                std::ref(*CHECK_NOTNULL(vio_pipeline_.get())),
                std::placeholders::_1));

  data_provider_->registerLeftFrameCallback(
      std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                std::ref(*CHECK_NOTNULL(vio_pipeline_.get())),
                std::placeholders::_1));

  data_provider_->registerExternalOdomCallback(
      std::bind(&VIO::Pipeline::fillExternalOdomQueue,
                std::ref(*CHECK_NOTNULL(vio_pipeline_.get())),
                std::placeholders::_1));

  if (vio_params_->frontend_type_ == VIO::FrontendType::kStereoImu) {
    auto stereo_pipeline = dynamic_cast<StereoImuPipeline*>(vio_pipeline_.get());
    CHECK(stereo_pipeline);

    data_provider_->registerRightFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue,
                  std::ref(*stereo_pipeline),
                  std::placeholders::_1));
  }

  if (vio_params_->frontend_type_ == VIO::FrontendType::kRgbdImu) {
    data_provider_->registerDepthFrameCallback(std::bind(
        &VIO::RgbdImuPipeline::fillDepthFrameQueue,
        CHECK_NOTNULL(dynamic_cast<RgbdImuPipeline*>(vio_pipeline_.get())),
        std::placeholders::_1));
  }

  if (ros_lcd_visualizer_) {
    vio_pipeline_->registerLcdOutputCallback([&](const auto& msg) {
      if (msg) {
        ros_lcd_visualizer_->publishLcdOutput(msg);
      }
    });
  }
}

void KimeraVioRos::bufferExternalOdometryBeliefs(
    const std::vector<ExternalOdometryBelief>& beliefs) {
  if (beliefs.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(external_beliefs_mutex_);
  for (const auto& belief : beliefs) {
    pending_external_odom_beliefs_.push_back(belief);
  }
  while (pending_external_odom_beliefs_.size() >
         external_beliefs_queue_limit_) {
    pending_external_odom_beliefs_.pop_front();
  }
}

void KimeraVioRos::flushExternalOdometryBeliefsToPipeline() {
  if (!vio_pipeline_) {
    return;
  }

  std::vector<ExternalOdometryBelief> beliefs_to_forward;
  {
    std::lock_guard<std::mutex> lock(external_beliefs_mutex_);
    beliefs_to_forward.reserve(pending_external_odom_beliefs_.size());
    while (!pending_external_odom_beliefs_.empty()) {
      beliefs_to_forward.push_back(pending_external_odom_beliefs_.front());
      pending_external_odom_beliefs_.pop_front();
    }
  }

  if (!beliefs_to_forward.empty()) {
    vio_pipeline_->enqueueExternalOdometryBeliefs(beliefs_to_forward);
  }
}

void KimeraVioRos::initializeHeadlessCbsBeliefBridge() {
  if (!headless_cbs_belief_bridge_enable_) {
    LOG(INFO) << "Kimera headless belief bridge disabled.";
    return;
  }

  ros::NodeHandle nh;
  pose_odom_belief_out_pub_ =
      nh.advertise<liorf::pose_odom_belief_array>(
          cbs_odom_belief_out_topic_, 50);
  pose_odom_belief_in_sub_ =
      nh.subscribe<liorf::pose_odom_belief_array>(
          cbs_odom_belief_in_topic_,
          50,
          &KimeraVioRos::poseOdomBeliefInCallback,
          this,
          ros::TransportHints().tcpNoDelay());
  if (cbs_belief_receive_start_delay_sec_ > 0.0) {
    cbs_belief_receive_clock_sub_ =
        nh.subscribe<rosgraph_msgs::Clock>(
            "/clock",
            10,
            &KimeraVioRos::cbsBeliefReceiveClockCallback,
            this,
            ros::TransportHints().tcpNoDelay());
  }

  LOG(INFO) << "Kimera headless belief bridge enabled. agent='"
            << static_cast<char>(cbs_agent_id_) << "', odom_in='"
            << cbs_odom_belief_in_topic_
            << "', odom_out='" << cbs_odom_belief_out_topic_
            << "', external_pose_frame='" << cbs_external_pose_frame_id_
            << "', receive_start_delay_sec="
            << cbs_belief_receive_start_delay_sec_
            << "'.";
}

void KimeraVioRos::initializeHeadlessOdometryPublisher() {
  if (!headless_odometry_publish_enable_) {
    LOG(INFO) << "Kimera headless odometry publisher disabled.";
    return;
  }

  ros::NodeHandle nh;
  headless_odometry_pub_ =
      nh.advertise<nav_msgs::Odometry>("odometry", 10, true);
  LOG(INFO) << "Kimera headless odometry publisher enabled on topic '"
            << headless_odometry_pub_.getTopic() << "'.";
}

void KimeraVioRos::initializeHeadlessRerunVisualizer() {
  if (!headless_rerun_visualizer_enable_) {
    LOG(INFO) << "Kimera headless Rerun visualizer disabled.";
    return;
  }

  headless_rerun_visualizer_ = std::make_unique<RosRerunVisualizer>(
      "cbsms", headless_rerun_recording_id_, headless_rerun_host_);
  LOG(INFO) << "Kimera headless Rerun visualizer enabled. recording_id='"
            << headless_rerun_recording_id_ << "', host='"
            << headless_rerun_host_ << "'.";
}

void KimeraVioRos::publishHeadlessBackendOutput(
    const BackendOutput::ConstPtr& output) {
  if (!use_rviz_) {
    publishHeadlessOdometry(output);
    publishHeadlessRerunBackendOutput(output);
  }
  publishHeadlessOdometryBelief(output);
}

void KimeraVioRos::publishHeadlessOdometry(
    const BackendOutput::ConstPtr& output) {
  try {
    CHECK(output);
    if (!headless_odometry_publish_enable_) {
      return;
    }

    const Timestamp& ts = output->timestamp_;
    const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
    const gtsam::Rot3& rotation = pose.rotation();
    const gtsam::Quaternion& quaternion = rotation.toQuaternion();
    const gtsam::Vector3& velocity = output->W_State_Blkf_.velocity_;
    const gtsam::Matrix6 pose_cov =
        poseCovarianceFromMatrix(output->state_covariance_lkf_);
    gtsam::Matrix3 vel_cov = gtsam::Matrix3::Identity() * 1e-3;
    if (output->state_covariance_lkf_.rows() >= 9 &&
        output->state_covariance_lkf_.cols() >= 9) {
      vel_cov = gtsam::sub(output->state_covariance_lkf_, 6, 9, 6, 9);
    }

    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp.fromNSec(ts);
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;

    odometry_msg.pose.pose.position.x = pose.x();
    odometry_msg.pose.pose.position.y = pose.y();
    odometry_msg.pose.pose.position.z = pose.z();
    odometry_msg.pose.pose.orientation.w = quaternion.w();
    odometry_msg.pose.pose.orientation.x = quaternion.x();
    odometry_msg.pose.pose.orientation.y = quaternion.y();
    odometry_msg.pose.pose.orientation.z = quaternion.z();

    static const std::vector<int> remapping{3, 4, 5, 0, 1, 2};
    for (int i = 0; i < pose_cov.rows(); ++i) {
      for (int j = 0; j < pose_cov.cols(); ++j) {
        odometry_msg.pose.covariance[remapping[i] * pose_cov.cols() +
                                     remapping[j]] = pose_cov(i, j);
      }
    }

    const gtsam::Matrix3 inversed_rotation = rotation.transpose();
    const gtsam::Vector3 velocity_body = inversed_rotation * velocity;
    odometry_msg.twist.twist.linear.x = velocity_body(0);
    odometry_msg.twist.twist.linear.y = velocity_body(1);
    odometry_msg.twist.twist.linear.z = velocity_body(2);

    const gtsam::Matrix3 vel_cov_body =
        inversed_rotation.matrix() * vel_cov * rotation.matrix();
    for (int i = 0; i < vel_cov_body.rows(); ++i) {
      for (int j = 0; j < vel_cov_body.cols(); ++j) {
        odometry_msg.twist.covariance[i * 6 + j] = vel_cov_body(i, j);
      }
    }

    headless_odometry_pub_.publish(odometry_msg);
  } catch (const std::exception& e) {
    LOG(WARNING) << "Kimera headless odometry publish skipped: " << e.what();
  } catch (...) {
    LOG(WARNING) << "Kimera headless odometry publish skipped.";
  }
}

void KimeraVioRos::publishHeadlessRerunBackendOutput(
    const BackendOutput::ConstPtr& output) {
  try {
    CHECK(output);
    if (!headless_rerun_visualizer_) {
      return;
    }

    const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
    headless_rerun_visualizer_->setTimeNSec(output->timestamp_);
    headless_rerun_visualizer_->drawTf("kimera/base_link", pose, 0.5f);

    const Eigen::Matrix3d current_pose_covariance =
        translationCovarianceFromPoseCovariance(output->state_covariance_lkf_);
    drawRawPoseCovariance6x6(
        headless_rerun_visualizer_.get(),
        "kimera/current_pose/raw_pose_covariance_6x6",
        poseCovarianceFromMatrix(output->state_covariance_lkf_));
    headless_rerun_visualizer_->drawUncertainty(
        "kimera/current_pose/uncertainty",
        pose,
        current_pose_covariance,
        Eigen::Vector4f(40.f, 220.f, 80.f, 160.f),
        1.25f);
    headless_rerun_visualizer_->drawScalar(
        "kimera/current_pose/kimera_uncertainty_frobenius_norm",
        current_pose_covariance.norm());
    headless_rerun_visualizer_->drawScalar("kimera/keyframe_id",
                                           output->cur_kf_id_);
    headless_rerun_visualizer_->drawScalar(
        "kimera/timing/optimization_ms",
        output->optimization_time_sec_ * 1000.0);

    if (headless_cbs_belief_bridge_enable_) {
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/published_per_update",
          output->cbs_outgoing_odom_beliefs_.size());
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/received_per_update",
          output->external_beliefs_received_per_update_);
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/dropped_by_receive_gate_per_update",
          cbs_beliefs_receive_gate_dropped_per_rerun_frame_.exchange(
              0u, std::memory_order_relaxed));
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/added_to_factor_graph_per_update",
          output->external_beliefs_added_per_update_);
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/rejected_first_message_per_update",
          output->external_beliefs_rejected_first_message_per_update_);
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/rejected_update_status_per_update",
          output->external_beliefs_rejected_update_status_per_update_);
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/rejected_inactive_window_per_update",
          output->external_beliefs_rejected_inactive_window_per_update_);
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/rejected_shape_per_update",
          output->external_beliefs_rejected_shape_per_update_);
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/beliefs/rejected_exception_per_update",
          output->external_beliefs_rejected_exception_per_update_);
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/timing/belief_generation_ms",
          output->cbs_belief_generation_time_sec_ * 1000.0);
      headless_rerun_visualizer_->drawScalar(
          "kimera/cbs/marginalization_graph/factor_count",
          output->cbs_marginalization_graph_factor_count_);
    }

    const int64_t current_kf_id = static_cast<int64_t>(output->cur_kf_id_);
    if (current_kf_id != headless_rerun_last_kf_id_) {
      headless_rerun_trajectory_.push_back(pose);
      headless_rerun_last_kf_id_ = current_kf_id;
    }
    if (headless_rerun_trajectory_.size() > 1u) {
      headless_rerun_visualizer_->drawTrajectory(
          "kimera/trajectory",
          headless_rerun_trajectory_,
          Eigen::Vector4f(40.f, 220.f, 80.f, 255.f),
          1.5f);
    }

    std::vector<gtsam::Point3> landmarks;
    landmarks.reserve(output->landmarks_with_id_map_.size());
    for (const auto& id_landmark : output->landmarks_with_id_map_) {
      landmarks.emplace_back(id_landmark.second);
    }
    if (!landmarks.empty()) {
      headless_rerun_visualizer_->drawPoints(
          "kimera/landmarks",
          landmarks,
          Eigen::Vector4f(40.f, 220.f, 80.f, 180.f),
          2.f);
    }

    if (headless_rerun_factor_graph_enable_ &&
        output->factor_graph_.size() > 0u && output->state_.size() > 0u) {
      headless_rerun_visualizer_->drawFactors(
          "kimera/factor_graph",
          output->factor_graph_,
          output->state_,
          Eigen::Vector4f(40.f, 220.f, 80.f, 180.f),
          0.75f);
      headless_rerun_visualizer_->drawScalar(
          "kimera/factor_graph/factors_total", output->factor_graph_.size());
    }
  } catch (const std::exception& e) {
    LOG(WARNING) << "Kimera headless Rerun publish skipped: " << e.what();
  } catch (...) {
    LOG(WARNING) << "Kimera headless Rerun publish skipped.";
  }
}

void KimeraVioRos::publishHeadlessOdometryBelief(
    const BackendOutput::ConstPtr& output) {
  try {
    CHECK(output);
    if (!headless_cbs_belief_bridge_enable_ ||
        output->cbs_outgoing_odom_beliefs_.empty()) {
      return;
    }

    liorf::pose_odom_belief_array msg;
    msg.header.stamp.fromNSec(output->timestamp_);
    msg.header.frame_id = odom_frame_id_;
    msg.beliefs.reserve(output->cbs_outgoing_odom_beliefs_.size());

    const bool publishes_external_pose_frame =
        (cbs_external_pose_frame_id_ != base_link_frame_id_);
    gtsam::Pose3 base_T_external;
    gtsam::Pose3 external_T_base;
    if (publishes_external_pose_frame &&
        !lookupExternalPoseFrameTransform(&base_T_external,
                                          &external_T_base)) {
      return;
    }

    for (const auto& cbs_belief : output->cbs_outgoing_odom_beliefs_) {
      liorf::pose_odom_belief belief;
      belief.header = msg.header;
      belief.source_agent = cbs_belief.source_agent;
      belief.from_pose_index = cbs_belief.from_pose_index;
      belief.to_pose_index = cbs_belief.to_pose_index;
      belief.from_stamp_sec = cbs_belief.from_stamp_sec;
      belief.to_stamp_sec =
          cbs_belief.to_stamp_sec > 0.0 ? cbs_belief.to_stamp_sec
                                        : msg.header.stamp.toSec();
      belief.relax_factor = cbs_belief.relax_factor;

      gtsam::Pose3 relative_to_publish =
          gtsam::Pose3::Expmap(toVector6(cbs_belief.relative_mu));
      gtsam::Matrix6 covariance_to_publish =
          poseCovarianceFromMatrix(toMatrix6(cbs_belief.covariance));

      if (publishes_external_pose_frame) {
        relative_to_publish =
            external_T_base * relative_to_publish * base_T_external;
        const gtsam::Matrix6 adjoint_external_base =
            external_T_base.AdjointMap();
        covariance_to_publish = poseCovarianceFromMatrix(
            adjoint_external_base * covariance_to_publish *
            adjoint_external_base.transpose());
      }

      fromVector6(gtsam::Pose3::Logmap(relative_to_publish),
                  &belief.relative_mu);
      fromMatrix6(covariance_to_publish, &belief.covariance);
      msg.beliefs.push_back(belief);
    }

    if (!msg.beliefs.empty()) {
      pose_odom_belief_out_pub_.publish(msg);
    }
  } catch (const std::exception& e) {
    LOG(WARNING) << "Kimera headless CBS odometry belief publish skipped: "
                 << e.what();
  } catch (...) {
    LOG(WARNING) << "Kimera headless CBS odometry belief publish skipped.";
  }
}

double KimeraVioRos::incomingOdomBeliefGateStampSec(
    const liorf::pose_odom_belief& belief) const {
  const double to_stamp_sec = belief.to_stamp_sec > 0.0
                                  ? belief.to_stamp_sec
                                  : belief.header.stamp.toSec();
  if (std::isfinite(to_stamp_sec) && to_stamp_sec > 0.0) {
    return to_stamp_sec;
  }
  const double from_stamp_sec = belief.from_stamp_sec;
  if (std::isfinite(from_stamp_sec) && from_stamp_sec > 0.0) {
    return from_stamp_sec;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void KimeraVioRos::initializeIncomingOdomBeliefReceiveGate(
    const liorf::pose_odom_belief_array& msg) {
  if (cbs_belief_receive_start_delay_sec_ <= 0.0 ||
      cbs_belief_receive_gate_reference_set_) {
    return;
  }

  double earliest_stamp_sec = std::numeric_limits<double>::infinity();
  for (const auto& belief : msg.beliefs) {
    if (belief.source_agent == cbs_agent_id_) {
      continue;
    }
    const double stamp_sec = incomingOdomBeliefGateStampSec(belief);
    if (std::isfinite(stamp_sec) && stamp_sec > 0.0) {
      earliest_stamp_sec = std::min(earliest_stamp_sec, stamp_sec);
    }
  }

  if (!std::isfinite(earliest_stamp_sec)) {
    return;
  }

  cbs_belief_receive_gate_reference_stamp_sec_ = earliest_stamp_sec;
  cbs_belief_receive_gate_reference_set_ = true;
  LOG(INFO) << "Kimera CBS incoming belief receive gate reference stamp="
            << std::fixed << std::setprecision(9)
            << cbs_belief_receive_gate_reference_stamp_sec_
            << " from first incoming belief, accepting belief odometry with "
            << "to_stamp >= "
            << (cbs_belief_receive_gate_reference_stamp_sec_ +
                cbs_belief_receive_start_delay_sec_)
            << " (delay=" << cbs_belief_receive_start_delay_sec_ << " s).";
}

bool KimeraVioRos::isIncomingOdomBeliefBeforeReceiveGate(
    const liorf::pose_odom_belief& belief) const {
  if (cbs_belief_receive_start_delay_sec_ <= 0.0 ||
      !cbs_belief_receive_gate_reference_set_) {
    return false;
  }

  const double stamp_sec = incomingOdomBeliefGateStampSec(belief);
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return true;
  }
  return stamp_sec <
         cbs_belief_receive_gate_reference_stamp_sec_ +
             cbs_belief_receive_start_delay_sec_;
}

void KimeraVioRos::cbsBeliefReceiveClockCallback(
    const rosgraph_msgs::ClockConstPtr& msg) {
  if (!msg || cbs_belief_receive_start_delay_sec_ <= 0.0 ||
      cbs_belief_receive_gate_reference_set_) {
    return;
  }

  const double stamp_sec = msg->clock.toSec();
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return;
  }

  cbs_belief_receive_gate_reference_stamp_sec_ = stamp_sec;
  cbs_belief_receive_gate_reference_set_ = true;
  cbs_belief_receive_clock_sub_.shutdown();
  LOG(INFO) << "Kimera CBS incoming belief receive gate reference stamp="
            << std::fixed << std::setprecision(9)
            << cbs_belief_receive_gate_reference_stamp_sec_
            << " from /clock, accepting belief odometry with to_stamp >= "
            << (cbs_belief_receive_gate_reference_stamp_sec_ +
                cbs_belief_receive_start_delay_sec_)
            << " (delay=" << cbs_belief_receive_start_delay_sec_ << " s).";
}

void KimeraVioRos::poseOdomBeliefInCallback(
    const liorf::pose_odom_belief_arrayConstPtr& msg) {
  try {
    if (!msg) {
      return;
    }

    std::vector<ExternalOdometryBelief> converted_beliefs;
    converted_beliefs.reserve(msg->beliefs.size());
    size_t dropped_by_receive_gate = 0u;

    initializeIncomingOdomBeliefReceiveGate(*msg);

    gtsam::Pose3 base_T_external;
    gtsam::Pose3 external_T_base;
    if (cbs_external_pose_frame_id_ != base_link_frame_id_ &&
        !lookupExternalPoseFrameTransform(&base_T_external, &external_T_base)) {
      return;
    }

    for (const auto& belief : msg->beliefs) {
      if (belief.source_agent == cbs_agent_id_) {
        continue;
      }
      if (isIncomingOdomBeliefBeforeReceiveGate(belief)) {
        ++dropped_by_receive_gate;
        continue;
      }

      ExternalOdometryBelief converted;
      converted.source_agent = belief.source_agent;
      converted.from_pose_index = belief.from_pose_index;
      converted.to_pose_index = belief.to_pose_index;
      converted.sender_from_pose_index = belief.from_pose_index;
      converted.sender_to_pose_index = belief.to_pose_index;
      converted.from_stamp_sec = belief.from_stamp_sec;
      converted.to_stamp_sec = belief.to_stamp_sec > 0.0
                                   ? belief.to_stamp_sec
                                   : belief.header.stamp.toSec();
      converted.sender_timestamp_ns = belief.header.stamp.toNSec();
      converted.sender_frame_id = belief.header.frame_id;
      converted.received_wall_time_sec = ros::WallTime::now().toSec();
      converted.relax_factor = belief.relax_factor;

      gtsam::Pose3 transformed_relative =
          gtsam::Pose3::Expmap(toVector6(belief.relative_mu));
      gtsam::Matrix6 transformed_covariance =
          poseCovarianceFromMatrix(toMatrix6(belief.covariance));

      if (cbs_external_pose_frame_id_ != base_link_frame_id_) {
        transformed_relative =
            base_T_external * transformed_relative * external_T_base;
        const gtsam::Matrix6 adjoint_base_external =
            base_T_external.AdjointMap();
        transformed_covariance = poseCovarianceFromMatrix(
            adjoint_base_external * transformed_covariance *
            adjoint_base_external.transpose());
      }

      fromVector6(gtsam::Pose3::Logmap(transformed_relative),
                  &converted.relative_mu);
      fromMatrix6(transformed_covariance, &converted.covariance);
      converted_beliefs.push_back(converted);
    }

    if (dropped_by_receive_gate > 0u) {
      cbs_beliefs_receive_gate_dropped_per_rerun_frame_.fetch_add(
          dropped_by_receive_gate, std::memory_order_relaxed);
      ROS_INFO_STREAM_THROTTLE(
          1.0,
          "Kimera CBS incoming belief receive gate dropped "
              << dropped_by_receive_gate
              << " belief(s) before delay="
              << cbs_belief_receive_start_delay_sec_ << " s");
    }

    bufferExternalOdometryBeliefs(converted_beliefs);
  } catch (const std::exception& e) {
    LOG(WARNING) << "Kimera headless CBS incoming odometry belief callback "
                    "skipped: "
                 << e.what();
  } catch (...) {
    LOG(WARNING) << "Kimera headless CBS incoming odometry belief callback "
                    "skipped.";
  }
}

bool KimeraVioRos::lookupExternalPoseFrameTransform(
    gtsam::Pose3* base_T_external,
    gtsam::Pose3* external_T_base) {
  CHECK_NOTNULL(base_T_external);
  CHECK_NOTNULL(external_T_base);
  if (cbs_external_pose_frame_id_ == base_link_frame_id_) {
    *base_T_external = gtsam::Pose3();
    *external_T_base = gtsam::Pose3();
    return true;
  }

  tf::StampedTransform base_to_external_tf;
  try {
    tf_listener_.lookupTransform(base_link_frame_id_,
                                 cbs_external_pose_frame_id_,
                                 ros::Time(0),
                                 base_to_external_tf);
  } catch (const tf::TransformException& ex) {
    ROS_WARN_STREAM_THROTTLE(2.0,
                             "CBS belief frame transform unavailable for "
                                 << base_link_frame_id_ << " -> "
                                 << cbs_external_pose_frame_id_ << ": "
                                 << ex.what());
    return false;
  }

  geometry_msgs::Transform tf_msg;
  tf_msg.translation.x = base_to_external_tf.getOrigin().x();
  tf_msg.translation.y = base_to_external_tf.getOrigin().y();
  tf_msg.translation.z = base_to_external_tf.getOrigin().z();
  tf_msg.rotation.x = base_to_external_tf.getRotation().x();
  tf_msg.rotation.y = base_to_external_tf.getRotation().y();
  tf_msg.rotation.z = base_to_external_tf.getRotation().z();
  tf_msg.rotation.w = base_to_external_tf.getRotation().w();

  utils::rosTfToGtsamPose(tf_msg, base_T_external);
  *external_T_base = base_T_external->inverse();
  return true;
}

bool KimeraVioRos::restartKimeraVio(std_srvs::Trigger::Request& request,
                                    std_srvs::Trigger::Response& response) {
  if (!restart_vio_pipeline_) {
    restart_vio_pipeline_ = true;
    response.message = "Kimera-VIO restart requested.";
    response.success = true;
  } else {
    response.message = "Kimera-VIO should already be restarting...";
    response.success = false;
  }
  LOG(WARNING) << response.message;
  return true;
}

}  // namespace VIO
