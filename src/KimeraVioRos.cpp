/* @file   KimeraVioRos.cpp
 * @brief  ROS Wrapper for Kimera-VIO
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/KimeraVioRos.h"

#include <future>

// zy Step 2_f
// Adds helpers for source normalization and robust ROS message parsing.
#include <algorithm>
#include <cctype>
#include <cmath>
#include "kimera_vio_ros/utils/UtilsRos.h"



// Still need gflags for parameters in VIO
#include <gflags/gflags.h>
#include <glog/logging.h>

// Dependencies from ROS
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
  
  // zy Step 1_c
  // Loads external-bridge ROS params early so later callback wiring uses one consistent config source.
  nh_private_.param<bool>("enable_external_pose_bridge",
                          enable_external_pose_bridge_,
                          false);
  // zy Step 4_c
  // Uses LIORF-compatible topic when no ROS param override is provided.
  nh_private_.param<std::string>("external_pose_belief_topic",
                                external_pose_belief_topic_,
                                "/liorf/cbs/external_pose_prior");

  // zy Step 5_b
  // Uses LIORF belief topic as default incoming prior channel when launch does not override.
  nh_private_.param<std::string>("external_pose_prior_topic",
                               external_pose_prior_topic_,
                               "/liorf/cbs/external_pose_belief");

  nh_private_.param<std::string>("external_pose_belief_source",
                                 external_pose_belief_source_,
                                 "kimera");
  nh_private_.param<std::string>("external_prior_default_source",
                                 external_prior_default_source_,
                                 "liorf");

  std::string odom_frame_id = "odom";
  nh_private_.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  nh_private_.param<std::string>("external_exchange_frame_id",
                                 external_exchange_frame_id_,
                                 odom_frame_id);

  LOG(INFO) << "External pose bridge config: enabled="
            << enable_external_pose_bridge_
            << ", belief_topic=" << external_pose_belief_topic_
            << ", prior_topic=" << external_pose_prior_topic_
            << ", source=" << external_pose_belief_source_
            << ", frame_id=" << external_exchange_frame_id_;


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

// zy Step 2_d
// Converts source labels to canonical tags so both estimators use the same sender identity.
std::string KimeraVioRos::normalizeExternalSourceTag(
    const std::string& source) const {
  std::string normalized = source;
  std::transform(normalized.begin(),
                 normalized.end(),
                 normalized.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  normalized.erase(
      std::remove_if(normalized.begin(),
                     normalized.end(),
                     [](unsigned char c) { return std::isspace(c) != 0; }),
      normalized.end());
  while (!normalized.empty() && normalized.front() == '/') {
    normalized.erase(normalized.begin());
  }
  return normalized.empty() ? std::string("unknown") : normalized;
}

// zy Step 2_e
// Publishes Kimera belief (W_Pose_B + covariance at keyframe timestamp) on ROS for external fusion.
void KimeraVioRos::publishExternalPoseBelief(
    const VioBackend::ExternalPoseBelief& belief) {
  if (!enable_external_pose_bridge_) return;
  if (belief.timestamp_kf_nsec_ <= 0) return;

  nav_msgs::Odometry msg;
  msg.header.seq = external_pose_belief_seq_counter_++;
  msg.header.stamp.fromNSec(static_cast<uint64_t>(belief.timestamp_kf_nsec_));
  msg.header.frame_id = external_exchange_frame_id_;
  msg.child_frame_id = normalizeExternalSourceTag(external_pose_belief_source_);

  const gtsam::Quaternion& q = belief.W_Pose_B_.rotation().toQuaternion();
  msg.pose.pose.position.x = belief.W_Pose_B_.x();
  msg.pose.pose.position.y = belief.W_Pose_B_.y();
  msg.pose.pose.position.z = belief.W_Pose_B_.z();
  msg.pose.pose.orientation.w = q.w();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();

  // GTSAM pose covariance order: [rx ry rz tx ty tz]
  // ROS odom covariance order:   [tx ty tz rx ry rz]
  static const int remap[6] = {3, 4, 5, 0, 1, 2};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      msg.pose.covariance[remap[i] * 6 + remap[j]] = belief.covariance_(i, j);
    }
  }

  pub_external_pose_belief_.publish(msg);
}

// zy Step 2_f
// Converts incoming ROS prior into Kimera's W_Pose_B + covariance and enqueues it for next backend optimize.
void KimeraVioRos::externalPosePriorCallback(
    const nav_msgs::Odometry::ConstPtr& msg) {
  if (!enable_external_pose_bridge_) return;
  if (!msg) return;
  if (!vio_pipeline_) return;

  const Timestamp ts_nsec = static_cast<Timestamp>(msg->header.stamp.toNSec());
  if (ts_nsec <= 0) {
    LOG(WARNING) << "External prior ignored: invalid timestamp.";
    return;
  }

  gtsam::Pose3 W_Pose_B;
  VIO::utils::rosOdometryToGtsamPose(*msg, &W_Pose_B);

  gtsam::Matrix6 covariance = gtsam::Matrix6::Zero();
  static const int remap[6] = {3, 4, 5, 0, 1, 2};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      covariance(i, j) = msg->pose.covariance[remap[i] * 6 + remap[j]];
    }
  }
  covariance = 0.5 * (covariance + covariance.transpose());

  std::string source = normalizeExternalSourceTag(msg->child_frame_id);
  if (source == "unknown") {
    source = normalizeExternalSourceTag(external_prior_default_source_);
  }

  const bool queued = vio_pipeline_->enqueueExternalPosePriorFromCovariance(
      ts_nsec, W_Pose_B, covariance, source, static_cast<uint64_t>(msg->header.seq));

  if (!queued) {
    LOG(WARNING) << "External prior rejected by Kimera queue. ts_nsec=" << ts_nsec
                 << ", source=" << source;
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

  // zy Step 2_g
  // Enables Kimera<->LIORF bridge only when requested, preserving default behavior when disabled.
  if (enable_external_pose_bridge_) {
    pub_external_pose_belief_ =
        nh_private_.advertise<nav_msgs::Odometry>(external_pose_belief_topic_, 20);
    sub_external_pose_prior_ = nh_private_.subscribe<nav_msgs::Odometry>(
        external_pose_prior_topic_,
        200,
        &KimeraVioRos::externalPosePriorCallback,
        this);

    vio_pipeline_->registerExternalPoseBeliefCallback(
        std::bind(&KimeraVioRos::publishExternalPoseBelief,
                  this,
                  std::placeholders::_1));

    LOG(INFO) << "External pose bridge connected. publish_topic="
              << external_pose_belief_topic_
              << ", subscribe_topic=" << external_pose_prior_topic_;
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
