/**
 * @file   RosDisplay.cpp
 * @brief  Publishes 2D data sent in the display_queue in Kimera. This publishes
 * images at any rate (frame rate, keyframe rate,...).
 * @author Antoni Rosinol
 */
#include "kimera_vio_ros/RosVisualizer.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <kimera-vio/backend/VioBackend-definitions.h>
#include <kimera-vio/frontend/StereoVisionImuFrontend-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/mesh/Mesh.h>
#include <kimera-vio/mesh/Mesher-definitions.h>
#include <kimera-vio/pipeline/QueueSynchronizer.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>

#include "kimera_vio_ros/utils/UtilsRos.h"

DECLARE_int32(viz_type);

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

uint64_t timestampAbsDiff(uint64_t lhs, uint64_t rhs) {
  return lhs > rhs ? lhs - rhs : rhs - lhs;
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

struct OutgoingCovProvenance {
  std::string source_path;
  std::string is_local_only;
  std::string is_anchor_regularized;
  std::string is_fused_fallback;
};

OutgoingCovProvenance resolveKimeraOutgoingCovProvenance(
    const std::string& raw_source_tag) {
  const std::string source_tag = sanitizeCsvToken(raw_source_tag);
  if (source_tag == "local_bpsam") {
    return {"Kimera::local_bpsam_pose_covariance", "true", "true", "false"};
  }
  if (source_tag == "local_smoother") {
    return {"Kimera::local_smoother_pose_covariance", "true", "true", "false"};
  }
  if (source_tag == "filtered_fallback") {
    return {"Kimera::filtered_local_only_main_graph_covariance_fallback",
            "true",
            "true",
            "true"};
  }
  if (source_tag == "unavailable") {
    return {"Kimera::unavailable_pose_covariance", "false", "false", "false"};
  }
  return {"Kimera::unknown_pose_covariance_source_" + source_tag,
          "false",
          "false",
          "false"};
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
    ip << (raw_gateway & 0xfful) << "." << ((raw_gateway >> 8u) & 0xfful) << "."
       << ((raw_gateway >> 16u) & 0xfful) << "."
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
}  // namespace

RosVisualizer::RosVisualizer(const VioParams& vio_params)
    // I'm not sure we use this flag in ROS?
    : Visualizer3D(vio_params.frontend_type_ == FrontendType::kMonoImu
                       ? VisualizationType::kNone
                       : static_cast<VisualizationType>(FLAGS_viz_type)),
      nh_(),
      nh_private_("~"),
      image_size_(vio_params.camera_params_.at(0).image_size_),
      image_publishers_(nullptr) {
  //! To publish 2d images
  image_publishers_ = std::make_unique<ImagePublishers>(nh_private_);

  // Get ROS params
  CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  CHECK(!base_link_frame_id_.empty());
  CHECK(nh_private_.getParam("odom_frame_id", odom_frame_id_));
  CHECK(!odom_frame_id_.empty());
  CHECK(nh_private_.getParam("map_frame_id", map_frame_id_));
  CHECK(!map_frame_id_.empty());

  nh_private_.param(
      "cbs_belief_bridge_enable", cbs_belief_bridge_enable_, true);
  nh_private_.param<std::string>("cbs_external_pose_frame_id",
                                 cbs_external_pose_frame_id_,
                                 base_link_frame_id_);
  if (cbs_external_pose_frame_id_.empty()) {
    cbs_external_pose_frame_id_ = base_link_frame_id_;
  }

  bool rerun_visualizer_enable = false;
  std::string rerun_recording_id;
  std::string rerun_host;
  nh_private_.param("rerun_visualizer_enable", rerun_visualizer_enable, false);
  nh_private_.param<std::string>("rerun_recording_id", rerun_recording_id, "");
  nh_private_.param<std::string>("rerun_host", rerun_host, "auto");
  nh_private_.param(
      "rerun_factor_graph_enable", rerun_factor_graph_enable_, true);
  nh_private_.param(
      "rerun_world_alignment_enable", rerun_world_alignment_enable_, true);
  if (rerun_host.empty() || rerun_host == "auto") {
    rerun_host = defaultRerunHost();
  }
  if (rerun_recording_id.empty()) {
    ros::param::param<std::string>(
        "/cbsms/rerun_recording_id", rerun_recording_id, "");
  }
  if (rerun_recording_id.empty()) {
    rerun_recording_id = makeRerunRecordingId("kimera_vio_ros");
  }
  if (rerun_visualizer_enable) {
    rerun_visualizer_ = std::make_unique<RosRerunVisualizer>(
        "cbsms", rerun_recording_id, rerun_host);
    ROS_INFO_STREAM("Kimera Rerun visualizer enabled. recording_id='"
                    << rerun_recording_id << "', host='" << rerun_host << "'.");
  }

  // Publishers
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1, true);
  frontend_stats_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("frontend_stats", 1);
  resiliency_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("resiliency", 1);
  imu_bias_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("imu_bias", 1);
  pointcloud_pub_ =
      nh_.advertise<PointCloudXYZRGB>("time_horizon_pointcloud", 1, true);
  mesh_3d_frame_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("mesh", 1, true);
  if (cbs_belief_bridge_enable_) {
    ROS_INFO_STREAM("Kimera visualizer CBS pose-belief bridge is disabled; "
                    << "headless odometry-belief bridge handles CBS exchange.");
  } else {
    ROS_INFO("Kimera belief bridge disabled.");
  }
}

void RosVisualizer::updateRerunPoseHistory(uint64_t timestamp_ns,
                                           const gtsam::Pose3& pose) {
  if (!rerun_world_alignment_enable_) {
    return;
  }

  std::lock_guard<std::mutex> lock(rerun_world_alignment_mutex_);
  rerun_kimera_pose_history_.push_back({timestamp_ns, pose});

  constexpr size_t kMaxPoseHistory = 2000u;
  if (rerun_kimera_pose_history_.size() > kMaxPoseHistory) {
    const size_t extra = rerun_kimera_pose_history_.size() - kMaxPoseHistory;
    rerun_kimera_pose_history_.erase(rerun_kimera_pose_history_.begin(),
                                     rerun_kimera_pose_history_.begin() +
                                         extra);
  }
}

bool RosVisualizer::maybeInitializeRerunWorldAlignment(
    uint64_t peer_timestamp_ns,
    const gtsam::Pose3& liorf_world_pose_body,
    const std::string& key_token) {
  if (!rerun_world_alignment_enable_ || !rerun_visualizer_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(rerun_world_alignment_mutex_);
  if (rerun_world_alignment_initialized_ ||
      rerun_kimera_pose_history_.empty()) {
    return false;
  }

  auto nearest_it = rerun_kimera_pose_history_.end() - 1;
  if (peer_timestamp_ns != 0u) {
    nearest_it = std::min_element(
        rerun_kimera_pose_history_.begin(),
        rerun_kimera_pose_history_.end(),
        [peer_timestamp_ns](const RerunTimedPose& lhs,
                            const RerunTimedPose& rhs) {
          return timestampAbsDiff(lhs.timestamp_ns, peer_timestamp_ns) <
                 timestampAbsDiff(rhs.timestamp_ns, peer_timestamp_ns);
        });
  }

  rerun_liorf_T_kimera_world_ =
      liorf_world_pose_body * nearest_it->pose.inverse();
  rerun_world_alignment_timestamp_delta_ms_ =
      peer_timestamp_ns == 0u
          ? 0.0
          : static_cast<double>(
                timestampAbsDiff(peer_timestamp_ns, nearest_it->timestamp_ns)) /
                1.0e6;
  rerun_world_alignment_initialized_ = true;

  ROS_INFO_STREAM("Kimera Rerun world alignment initialized from "
                  << key_token << " with nearest Kimera pose dt="
                  << rerun_world_alignment_timestamp_delta_ms_
                  << " ms. This only affects kimera_aligned/* visualization.");
  return true;
}

bool RosVisualizer::lookupExternalPoseFrameTransform(
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

VisualizerOutput::UniquePtr RosVisualizer::spinOnce(
    const VisualizerInput& viz_input) {
  if (viz_input.frontend_output_) {
    publishFrontendOutput(viz_input.frontend_output_);
  }

  if (viz_input.backend_output_) {
    publishBackendOutput(viz_input.backend_output_);
  }

  if (viz_input.mesher_output_) {
    publishMesherOutput(viz_input.mesher_output_);
  }

  // Return empty output, since in ROS, we only publish, not display...
  return std::make_unique<VisualizerOutput>();
}

void RosVisualizer::publishBackendOutput(
    const BackendOutput::ConstPtr& output) {
  CHECK(output);
  publishTf(output);
  if (odometry_pub_.getNumSubscribers() > 0) {
    publishState(output);
  }
  if (imu_bias_pub_.getNumSubscribers() > 0) {
    publishImuBias(output);
  }
  if (pointcloud_pub_.getNumSubscribers() > 0) {
    publishTimeHorizonPointCloud(output);
  }
  publishRerunBackendOutput(output);
}

void RosVisualizer::publishRerunBackendOutput(
    const BackendOutput::ConstPtr& output) {
  CHECK(output);
  if (!rerun_visualizer_) {
    return;
  }

  rerun_visualizer_->setTimeNSec(output->timestamp_);
  updateRerunPoseHistory(output->timestamp_, output->W_State_Blkf_.pose_);
  rerun_visualizer_->drawTf(
      "kimera/base_link", output->W_State_Blkf_.pose_, 0.5f);
  const Eigen::Matrix3d current_pose_covariance =
      translationCovarianceFromPoseCovariance(output->state_covariance_lkf_);
  drawRawPoseCovariance6x6(
      rerun_visualizer_.get(),
      "kimera/current_pose/raw_pose_covariance_6x6",
      poseCovarianceFromMatrix(output->state_covariance_lkf_));
  rerun_visualizer_->drawUncertainty("kimera/current_pose/uncertainty",
                                     output->W_State_Blkf_.pose_,
                                     current_pose_covariance,
                                     Eigen::Vector4f(40.f, 220.f, 80.f, 160.f),
                                     1.25f);
  rerun_visualizer_->drawScalar(
      "kimera/current_pose/kimera_uncertainty_frobenius_norm",
      current_pose_covariance.norm());
  rerun_visualizer_->drawScalar("kimera/keyframe_id", output->cur_kf_id_);
  rerun_visualizer_->drawScalar("kimera/timing/optimization_ms",
                                output->optimization_time_sec_ * 1000.0);
  rerun_visualizer_->drawScalar("kimera/timing/optimize_total_ms",
                                output->optimize_total_time_sec_ * 1000.0);
  rerun_visualizer_->drawScalar("kimera/timing/smoother_update_ms",
                                output->optimization_time_sec_ * 1000.0);
  rerun_visualizer_->drawScalar(
      "kimera/timing/compute_state_covariance_ms",
      output->compute_state_covariance_time_sec_ * 1000.0);
  if (cbs_belief_bridge_enable_) {
    rerun_visualizer_->drawScalar(
        "kimera/cbs/beliefs/published_per_update",
        output->cbs_outgoing_odom_beliefs_.size());
    rerun_visualizer_->drawScalar(
        "kimera/cbs/beliefs/received_per_update",
        output->external_beliefs_received_per_update_);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/beliefs/added_to_factor_graph_per_update",
        output->external_beliefs_added_per_update_);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/beliefs/rejected_first_message_per_update",
        output->external_beliefs_rejected_first_message_per_update_);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/beliefs/rejected_update_status_per_update",
        output->external_beliefs_rejected_update_status_per_update_);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/beliefs/rejected_inactive_window_per_update",
        output->external_beliefs_rejected_inactive_window_per_update_);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/beliefs/rejected_shape_per_update",
        output->external_beliefs_rejected_shape_per_update_);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/beliefs/rejected_exception_per_update",
        output->external_beliefs_rejected_exception_per_update_);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/timing/belief_generation_ms",
        output->cbs_belief_generation_time_sec_ * 1000.0);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/timing/collect_external_beliefs_ms",
        output->collect_external_beliefs_time_sec_ * 1000.0);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/timing/outgoing_total_ms",
        output->cbs_outgoing_total_time_sec_ * 1000.0);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/timing/set_marginalization_graph_ms",
        output->cbs_set_marginalization_graph_time_sec_ * 1000.0);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/timing/get_odometry_beliefs_ms",
        output->cbs_get_odometry_beliefs_time_sec_ * 1000.0);
    rerun_visualizer_->drawScalar(
        "kimera/cbs/marginalization_graph/factor_count",
        output->cbs_marginalization_graph_factor_count_);
  }

  const int64_t current_kf_id = static_cast<int64_t>(output->cur_kf_id_);
  if (current_kf_id != rerun_last_kf_id_) {
    rerun_trajectory_.push_back(output->W_State_Blkf_.pose_);
    rerun_last_kf_id_ = current_kf_id;
  }
  if (rerun_trajectory_.size() > 1u) {
    rerun_visualizer_->drawTrajectory("kimera/trajectory",
                                      rerun_trajectory_,
                                      Eigen::Vector4f(40.f, 220.f, 80.f, 255.f),
                                      1.5f);
  }

  std::vector<gtsam::Point3> landmarks;
  landmarks.reserve(output->landmarks_with_id_map_.size());
  for (const auto& id_landmark : output->landmarks_with_id_map_) {
    landmarks.emplace_back(id_landmark.second);
  }
  if (!landmarks.empty()) {
    rerun_visualizer_->drawPoints("kimera/landmarks",
                                  landmarks,
                                  Eigen::Vector4f(40.f, 220.f, 80.f, 180.f),
                                  2.f);
  }

  gtsam::Pose3 liorf_T_kimera_world;
  bool world_alignment_initialized = false;
  double world_alignment_timestamp_delta_ms = 0.0;
  {
    std::lock_guard<std::mutex> lock(rerun_world_alignment_mutex_);
    world_alignment_initialized =
        rerun_world_alignment_enable_ && rerun_world_alignment_initialized_;
    if (world_alignment_initialized) {
      liorf_T_kimera_world = rerun_liorf_T_kimera_world_;
      world_alignment_timestamp_delta_ms =
          rerun_world_alignment_timestamp_delta_ms_;
    }
  }

  if (world_alignment_initialized) {
    const gtsam::Pose3 aligned_pose =
        liorf_T_kimera_world * output->W_State_Blkf_.pose_;
    rerun_visualizer_->drawTf("kimera_aligned/base_link", aligned_pose, 0.6f);

    const Eigen::Matrix3d alignment_rotation =
        liorf_T_kimera_world.rotation().matrix();
    const Eigen::Matrix3d aligned_pose_covariance =
        alignment_rotation * current_pose_covariance *
        alignment_rotation.transpose();
    rerun_visualizer_->drawUncertainty(
        "kimera_aligned/current_pose/uncertainty",
        aligned_pose,
        aligned_pose_covariance,
        Eigen::Vector4f(64.f, 255.f, 255.f, 180.f),
        1.25f);

    if (rerun_trajectory_.size() > 1u) {
      std::vector<gtsam::Pose3> aligned_trajectory;
      aligned_trajectory.reserve(rerun_trajectory_.size());
      for (const auto& pose : rerun_trajectory_) {
        aligned_trajectory.push_back(liorf_T_kimera_world * pose);
      }
      rerun_visualizer_->drawTrajectory(
          "kimera_aligned/trajectory",
          aligned_trajectory,
          Eigen::Vector4f(64.f, 255.f, 255.f, 255.f),
          2.0f);
    }

    if (!landmarks.empty()) {
      std::vector<gtsam::Point3> aligned_landmarks;
      aligned_landmarks.reserve(landmarks.size());
      for (const auto& landmark : landmarks) {
        aligned_landmarks.push_back(liorf_T_kimera_world.transformFrom(
            landmark));
      }
      rerun_visualizer_->drawPoints(
          "kimera_aligned/landmarks",
          aligned_landmarks,
          Eigen::Vector4f(64.f, 255.f, 255.f, 160.f),
          2.f);
    }

    rerun_visualizer_->drawScalar("kimera_aligned/alignment/initialized", 1.0);
    rerun_visualizer_->drawScalar(
        "kimera_aligned/alignment/timestamp_delta_ms",
        world_alignment_timestamp_delta_ms);
  } else if (rerun_world_alignment_enable_) {
    rerun_visualizer_->drawScalar("kimera_aligned/alignment/initialized", 0.0);
  }

  if (rerun_factor_graph_enable_ && output->factor_graph_.size() > 0u &&
      output->state_.size() > 0u) {
    rerun_visualizer_->drawFactors("kimera/factor_graph",
                                   output->factor_graph_,
                                   output->state_,
                                   Eigen::Vector4f(40.f, 220.f, 80.f, 180.f),
                                   0.75f);
    rerun_visualizer_->drawScalar("kimera/factor_graph/factors_total",
                                  output->factor_graph_.size());
  }
}

void RosVisualizer::publishFrontendOutput(
    const FrontendOutputPacketBase::ConstPtr& output) const {
  CHECK(output);
  if (frontend_stats_pub_.getNumSubscribers() > 0) {
    publishFrontendStats(output);
  }
}

void RosVisualizer::publishMesherOutput(
    const MesherOutput::ConstPtr& output) const {
  CHECK(output);
  if (mesh_3d_frame_pub_.getNumSubscribers() > 0) {
    publishPerFrameMesh3D(output);
  }
}

void RosVisualizer::publishTimeHorizonPointCloud(
    const BackendOutput::ConstPtr& output) const {
  CHECK(output);
  const Timestamp& timestamp = output->timestamp_;
  const PointsWithIdMap& points_with_id = output->landmarks_with_id_map_;
  const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map =
      output->lmk_id_to_lmk_type_map_;

  PointCloudXYZRGB::Ptr msg(new PointCloudXYZRGB);
  msg->header.frame_id = odom_frame_id_;
  msg->is_dense = true;
  msg->height = 1;
  msg->width = points_with_id.size();
  msg->points.resize(points_with_id.size());

  bool color_the_cloud = false;
  if (lmk_id_to_lmk_type_map.size() != 0) {
    color_the_cloud = true;
    CHECK_EQ(points_with_id.size(), lmk_id_to_lmk_type_map.size());
  }

  if (points_with_id.size() == 0) {
    // No points to visualize.
    return;
  }

  // Populate cloud structure with 3D points.
  size_t i = 0;
  for (const std::pair<LandmarkId, gtsam::Point3>& id_point : points_with_id) {
    const gtsam::Point3 point_3d = id_point.second;
    msg->points[i].x = static_cast<float>(point_3d.x());
    msg->points[i].y = static_cast<float>(point_3d.y());
    msg->points[i].z = static_cast<float>(point_3d.z());
    if (color_the_cloud) {
      DCHECK(lmk_id_to_lmk_type_map.find(id_point.first) !=
             lmk_id_to_lmk_type_map.end());
      switch (lmk_id_to_lmk_type_map.at(id_point.first)) {
        case LandmarkType::SMART: {
          // point_cloud_color.col(i) = cv::viz::Color::white();
          msg->points[i].r = 0;
          msg->points[i].g = 255;
          msg->points[i].b = 0;
          break;
        }
        case LandmarkType::PROJECTION: {
          // point_cloud_color.col(i) = cv::viz::Color::green();
          msg->points[i].r = 0;
          msg->points[i].g = 0;
          msg->points[i].b = 255;
          break;
        }
        default: {
          // point_cloud_color.col(i) = cv::viz::Color::white();
          msg->points[i].r = 255;
          msg->points[i].g = 0;
          msg->points[i].b = 0;
          break;
        }
      }
    }
    i++;
  }

  ros::Time ros_timestamp;
  ros_timestamp.fromNSec(timestamp);
  pcl_conversions::toPCL(ros_timestamp, msg->header.stamp);
  pointcloud_pub_.publish(msg);
}

void RosVisualizer::publishDebugImage(const Timestamp& timestamp,
                                      const cv::Mat& debug_image) const {
  // CHECK(debug_image.type(), CV_8UC1);
  std_msgs::Header h;
  h.stamp.fromNSec(timestamp);
  h.frame_id = base_link_frame_id_;
  // Copies...
  image_publishers_->publish(
      "mesh_2d", cv_bridge::CvImage(h, "bgr8", debug_image).toImageMsg());
}

void RosVisualizer::publishPerFrameMesh3D(
    const MesherOutput::ConstPtr& output) const {
  CHECK(output);

  const Mesh2D& mesh_2d = output->mesh_2d_;
  const Mesh3D& mesh_3d = output->mesh_3d_;
  size_t number_mesh_2d_polygons = mesh_2d.getNumberOfPolygons();
  size_t mesh_2d_poly_dim = mesh_2d.getMeshPolygonDimension();

  static const size_t cam_width = image_size_.width;
  static const size_t cam_height = image_size_.height;
  DCHECK_GT(cam_width, 0);
  DCHECK_GT(cam_height, 0);

  pcl_msgs::PolygonMesh::Ptr msg(new pcl_msgs::PolygonMesh());
  msg->header.stamp.fromNSec(output->timestamp_);
  msg->header.frame_id = odom_frame_id_;

  // Create point cloud to hold vertices.
  pcl::PointCloud<PointNormalUV> cloud;
  cloud.points.reserve(number_mesh_2d_polygons * mesh_2d_poly_dim);
  msg->polygons.reserve(number_mesh_2d_polygons);

  Mesh2D::Polygon polygon;
  for (size_t i = 0; i < number_mesh_2d_polygons; i++) {
    CHECK(mesh_2d.getPolygon(i, &polygon)) << "Could not retrieve 2d polygon.";
    const LandmarkId& lmk0_id = polygon.at(0).getLmkId();
    const LandmarkId& lmk1_id = polygon.at(1).getLmkId();
    const LandmarkId& lmk2_id = polygon.at(2).getLmkId();

    // Returns indices of points in the 3D mesh corresponding to the
    // vertices
    // in the 2D mesh.
    Mesh3D::VertexId p0_id, p1_id, p2_id;
    Mesh3D::VertexType vtx0, vtx1, vtx2;
    if (mesh_3d.getVertex(lmk0_id, &vtx0, &p0_id) &&
        mesh_3d.getVertex(lmk1_id, &vtx1, &p1_id) &&
        mesh_3d.getVertex(lmk2_id, &vtx2, &p2_id)) {
      // Get pixel coordinates of the vertices of the 2D mesh.
      const Vertex2D& px0 = polygon.at(0).getVertexPosition();
      const Vertex2D& px1 = polygon.at(1).getVertexPosition();
      const Vertex2D& px2 = polygon.at(2).getVertexPosition();

      // Get 3D coordinates of the vertices of the 3D mesh.
      const Vertex3D& lmk0_pos = vtx0.getVertexPosition();
      const Vertex3D& lmk1_pos = vtx1.getVertexPosition();
      const Vertex3D& lmk2_pos = vtx2.getVertexPosition();

      // Get normals of the vertices of the 3D mesh.
      const Mesh3D::VertexNormal& normal0 = vtx0.getVertexNormal();
      const Mesh3D::VertexNormal& normal1 = vtx1.getVertexNormal();
      const Mesh3D::VertexNormal& normal2 = vtx2.getVertexNormal();

      // FILL POINTCLOUD
      // clang-format off
      PointNormalUV pn0, pn1, pn2;
      pn0.x = lmk0_pos.x; pn1.x = lmk1_pos.x; pn2.x = lmk2_pos.x;
      pn0.y = lmk0_pos.y; pn1.y = lmk1_pos.y; pn2.y = lmk2_pos.y;
      pn0.z = lmk0_pos.z; pn1.z = lmk1_pos.z; pn2.z = lmk2_pos.z;
      // OpenGL textures range from 0 to 1.
      pn0.u = px0.x / cam_width; pn1.u = px1.x / cam_width; pn2.u = px2.x / cam_width;
      pn0.v = px0.y / cam_height; pn1.v = px1.y / cam_height; pn2.v = px2.y / cam_height;
      pn0.normal_x = normal0.x; pn1.normal_x = normal1.x; pn2.normal_x = normal2.x;
      pn0.normal_y = normal0.y; pn1.normal_y = normal1.y; pn2.normal_y = normal2.y;
      pn0.normal_z = normal0.z; pn1.normal_z = normal1.z; pn2.normal_z = normal2.z;
      // clang-format on

      // TODO(Toni): we are adding repeated vertices!!
      cloud.points.push_back(pn0);
      cloud.points.push_back(pn1);
      cloud.points.push_back(pn2);

      // Store polygon connectivity
      pcl_msgs::Vertices vtx_ii;
      vtx_ii.vertices.resize(3);
      size_t idx = i * mesh_2d_poly_dim;
      // Store connectivity CCW bcs of RVIZ
      vtx_ii.vertices[0] = idx + 2;
      vtx_ii.vertices[1] = idx + 1;
      vtx_ii.vertices[2] = idx;
      msg->polygons.push_back(vtx_ii);
    } else {
      // LOG_EVERY_N(ERROR, 1000) << "Polygon in 2d mesh did not have a
      // corresponding polygon in"
      //                          " 3d mesh!";
    }
  }

  cloud.is_dense = false;
  cloud.width = cloud.points.size();
  cloud.height = 1;
  pcl::toROSMsg(cloud, msg->cloud);

  // NOTE: Header fields need to be filled in after pcl::toROSMsg() call.
  msg->cloud.header = std_msgs::Header();
  msg->cloud.header.stamp = msg->header.stamp;
  msg->cloud.header.frame_id = msg->header.frame_id;

  if (msg->polygons.size() > 0u) {
    mesh_3d_frame_pub_.publish(msg);
  }
}

void RosVisualizer::publishState(const BackendOutput::ConstPtr& output) const {
  CHECK(output);
  // Get latest estimates for odometry.
  const Timestamp& ts = output->timestamp_;
  const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
  const gtsam::Rot3& rotation = pose.rotation();
  const gtsam::Quaternion& quaternion = rotation.toQuaternion();
  const gtsam::Vector3& velocity = output->W_State_Blkf_.velocity_;
  const gtsam::Matrix6& pose_cov =
      gtsam::sub(output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3& vel_cov =
      gtsam::sub(output->state_covariance_lkf_, 6, 9, 6, 9);

  // First publish odometry estimate
  nav_msgs::Odometry odometry_msg;

  // Create header.
  odometry_msg.header.stamp.fromNSec(ts);
  odometry_msg.header.frame_id = odom_frame_id_;
  odometry_msg.child_frame_id = base_link_frame_id_;

  // Position
  odometry_msg.pose.pose.position.x = pose.x();
  odometry_msg.pose.pose.position.y = pose.y();
  odometry_msg.pose.pose.position.z = pose.z();

  // Orientation
  odometry_msg.pose.pose.orientation.w = quaternion.w();
  odometry_msg.pose.pose.orientation.x = quaternion.x();
  odometry_msg.pose.pose.orientation.y = quaternion.y();
  odometry_msg.pose.pose.orientation.z = quaternion.z();

  // Remap covariance from GTSAM convention
  // to odometry convention and fill in covariance
  static const std::vector<int> remapping{3, 4, 5, 0, 1, 2};

  // Position covariance first, angular covariance after
  DCHECK_EQ(pose_cov.rows(), remapping.size());
  DCHECK_EQ(pose_cov.rows() * pose_cov.cols(),
            odometry_msg.pose.covariance.size());
  for (int i = 0; i < pose_cov.rows(); i++) {
    for (int j = 0; j < pose_cov.cols(); j++) {
      odometry_msg.pose
          .covariance[remapping[i] * pose_cov.cols() + remapping[j]] =
          pose_cov(i, j);
    }
  }

  // Linear velocities, trivial values for angular
  const gtsam::Matrix3& inversed_rotation = rotation.transpose();
  const Vector3 velocity_body = inversed_rotation * velocity;
  odometry_msg.twist.twist.linear.x = velocity_body(0);
  odometry_msg.twist.twist.linear.y = velocity_body(1);
  odometry_msg.twist.twist.linear.z = velocity_body(2);

  // Velocity covariance: first linear
  // and then angular (trivial values for angular)
  const gtsam::Matrix3 vel_cov_body =
      inversed_rotation.matrix() * vel_cov * rotation.matrix();
  DCHECK_EQ(vel_cov_body.rows(), 3);
  DCHECK_EQ(vel_cov_body.cols(), 3);
  DCHECK_EQ(odometry_msg.twist.covariance.size(), 36);
  for (int i = 0; i < vel_cov_body.rows(); i++) {
    for (int j = 0; j < vel_cov_body.cols(); j++) {
      odometry_msg.twist
          .covariance[i * static_cast<int>(
                              sqrt(odometry_msg.twist.covariance.size())) +
                      j] = vel_cov_body(i, j);
    }
  }
  // Publish message
  odometry_pub_.publish(odometry_msg);
}

void RosVisualizer::publishFrontendStats(
    const FrontendOutputPacketBase::ConstPtr& output) const {
  CHECK(output);

  // Get frontend data for resiliency output
  const DebugTrackerInfo& debug_tracker_info = output->getTrackerInfo();

  // Create message type
  std_msgs::Float64MultiArray frontend_stats_msg;

  // Build Message Layout
  frontend_stats_msg.data.resize(13);
  frontend_stats_msg.data[0] = debug_tracker_info.nrDetectedFeatures_;
  frontend_stats_msg.data[1] = debug_tracker_info.nrTrackerFeatures_;
  frontend_stats_msg.data[2] = debug_tracker_info.nrMonoInliers_;
  frontend_stats_msg.data[3] = debug_tracker_info.nrMonoPutatives_;
  frontend_stats_msg.data[4] = debug_tracker_info.nrStereoInliers_;
  frontend_stats_msg.data[5] = debug_tracker_info.nrStereoPutatives_;
  frontend_stats_msg.data[6] = debug_tracker_info.monoRansacIters_;
  frontend_stats_msg.data[7] = debug_tracker_info.stereoRansacIters_;
  frontend_stats_msg.data[8] = debug_tracker_info.nrValidRKP_;
  frontend_stats_msg.data[9] = debug_tracker_info.nrNoLeftRectRKP_;
  frontend_stats_msg.data[10] = debug_tracker_info.nrNoRightRectRKP_;
  frontend_stats_msg.data[11] = debug_tracker_info.nrNoDepthRKP_;
  frontend_stats_msg.data[12] = debug_tracker_info.nrFailedArunRKP_;
  frontend_stats_msg.layout.dim.resize(1);
  frontend_stats_msg.layout.dim[0].size = frontend_stats_msg.data.size();
  frontend_stats_msg.layout.dim[0].stride = 1;
  frontend_stats_msg.layout.dim[0].label =
      "Frontend: nrDetFeat, nrTrackFeat, nrMoIn, nrMoPu, nrStIn, nrStPu, "
      "moRaIt, stRaIt, nrVaRKP, nrNoLRKP, nrNoRRKP, nrNoDRKP nrFaARKP";

  // Publish Message
  frontend_stats_pub_.publish(frontend_stats_msg);
}

void RosVisualizer::publishResiliency(
    const FrontendOutputPacketBase::ConstPtr& frontend_output,
    const BackendOutput::ConstPtr& backend_output) const {
  CHECK(frontend_output);
  CHECK(backend_output);

  // Get frontend and velocity covariance data for resiliency output
  const DebugTrackerInfo& debug_tracker_info =
      frontend_output->getTrackerInfo();
  const gtsam::Matrix6& pose_cov =
      gtsam::sub(backend_output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3& vel_cov =
      gtsam::sub(backend_output->state_covariance_lkf_, 6, 9, 6, 9);

  // Create message type for quality of KimeraVIO
  std_msgs::Float64MultiArray resiliency_msg;

  // Publishing extra information:
  // cov_v_det and nrStIn should be the most relevant!
  resiliency_msg.layout.dim[0].label =
      "Values: cbrtPDet, cbrtVDet, nrStIn, nrMoIn. "
      "Thresholds : cbrtPDet, cbrtVDet, nrStIn, nrMoIn.";

  CHECK_EQ(pose_cov.size(), 36);
  gtsam::Matrix3 position_cov = gtsam::sub(pose_cov, 3, 6, 3, 6);
  CHECK_EQ(position_cov.size(), 9);

  // Compute eigenvalues and determinant of velocity covariance
  gtsam::Matrix U;
  gtsam::Matrix V;
  gtsam::Vector cov_v_eigv;
  gtsam::svd(vel_cov, U, cov_v_eigv, V);
  CHECK_EQ(cov_v_eigv.size(), 3);

  // Compute eigenvalues and determinant of position covariance
  gtsam::Vector cov_p_eigv;
  gtsam::svd(position_cov, U, cov_p_eigv, V);
  CHECK_EQ(cov_p_eigv.size(), 3);

  // Quality statistics to publish
  resiliency_msg.data.resize(8);
  resiliency_msg.data[0] =
      std::cbrt(cov_p_eigv(0) * cov_p_eigv(1) * cov_p_eigv(2));
  resiliency_msg.data[1] =
      std::cbrt(cov_v_eigv(0) * cov_v_eigv(1) * cov_v_eigv(2));
  resiliency_msg.data[2] = debug_tracker_info.nrStereoInliers_;
  resiliency_msg.data[3] = debug_tracker_info.nrMonoInliers_;

  // Publish thresholds for statistics
  float pos_det_threshold, vel_det_threshold;
  int mono_ransac_theshold, stereo_ransac_threshold;
  CHECK(nh_private_.getParam("velocity_det_threshold", vel_det_threshold));
  CHECK(nh_private_.getParam("position_det_threshold", pos_det_threshold));
  CHECK(
      nh_private_.getParam("stereo_ransac_threshold", stereo_ransac_threshold));
  CHECK(nh_private_.getParam("mono_ransac_threshold", mono_ransac_theshold));
  resiliency_msg.data[4] = pos_det_threshold;
  resiliency_msg.data[5] = vel_det_threshold;
  resiliency_msg.data[6] = stereo_ransac_threshold;
  resiliency_msg.data[7] = mono_ransac_theshold;

  // Build Message Layout
  resiliency_msg.layout.dim.resize(1);
  resiliency_msg.layout.dim[0].size = resiliency_msg.data.size();
  resiliency_msg.layout.dim[0].stride = 1;

  // Publish Message
  resiliency_pub_.publish(resiliency_msg);
}

void RosVisualizer::publishImuBias(
    const BackendOutput::ConstPtr& output) const {
  CHECK(output);

  // Get imu bias to output
  const ImuBias& imu_bias = output->W_State_Blkf_.imu_bias_;
  const Vector3& accel_bias = imu_bias.accelerometer();
  const Vector3& gyro_bias = imu_bias.gyroscope();

  // Create message type
  std_msgs::Float64MultiArray imu_bias_msg;

  // Get Imu Bias to Publish
  imu_bias_msg.data.resize(6);
  imu_bias_msg.data.at(0) = gyro_bias[0];
  imu_bias_msg.data.at(1) = gyro_bias[1];
  imu_bias_msg.data.at(2) = gyro_bias[2];
  imu_bias_msg.data.at(3) = accel_bias[0];
  imu_bias_msg.data.at(4) = accel_bias[1];
  imu_bias_msg.data.at(5) = accel_bias[2];

  // Build Message Layout
  imu_bias_msg.layout.dim.resize(1);
  imu_bias_msg.layout.dim[0].size = imu_bias_msg.data.size();
  imu_bias_msg.layout.dim[0].stride = 1;
  imu_bias_msg.layout.dim[0].label = "Gyro Bias: x,y,z. Accel Bias: x,y,z";

  // Publish Message
  imu_bias_pub_.publish(imu_bias_msg);
}

void RosVisualizer::publishTf(const BackendOutput::ConstPtr& output) {
  CHECK(output);

  const Timestamp& timestamp = output->timestamp_;
  const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
  // const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();
  // Publish base_link TF.
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header.stamp.fromNSec(timestamp);
  odom_tf.header.frame_id = odom_frame_id_;
  odom_tf.child_frame_id = base_link_frame_id_;

  utils::gtsamPoseToRosTf(pose, &odom_tf.transform);
  tf_broadcaster_.sendTransform(odom_tf);
}

}  // namespace VIO
