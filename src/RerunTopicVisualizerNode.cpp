/* -----------------------------------------------------------------------------
 * Copyright 2026.
 * ----------------------------------------------------------------------------*/

#include "kimera_vio_ros/RosRerunVisualizer.h"

#include <glog/logging.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Eigenvalues>

namespace VIO {
namespace {

constexpr double kMinQuaternionNorm = 1e-9;

std::string makeRerunRecordingId(const std::string& prefix) {
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
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

uint64_t stampToNSec(const ros::Time& stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ull +
         static_cast<uint64_t>(stamp.nsec);
}

bool odometryToPose(const nav_msgs::Odometry& odom, gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);
  const auto& p = odom.pose.pose.position;
  const auto& q = odom.pose.pose.orientation;
  const double norm =
      std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(norm) || norm < kMinQuaternionNorm) {
    return false;
  }
  *pose = gtsam::Pose3(
      gtsam::Rot3::Quaternion(q.w / norm, q.x / norm, q.y / norm, q.z / norm),
      gtsam::Point3(p.x, p.y, p.z));
  return pose->matrix().allFinite();
}

gtsam::Matrix6 poseCovarianceFromOdometry(const nav_msgs::Odometry& odom) {
  gtsam::Matrix6 covariance = gtsam::Matrix6::Zero();
  static const int remapping[6] = {3, 4, 5, 0, 1, 2};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      covariance(row, col) =
          odom.pose.covariance[remapping[row] * 6 + remapping[col]];
    }
  }
  return covariance;
}

Eigen::Matrix3d translationCovarianceFromPoseCovariance(
    const gtsam::Matrix& pose_covariance) {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  if (pose_covariance.rows() >= 6 && pose_covariance.cols() >= 6) {
    covariance = pose_covariance.block<3, 3>(3, 3);
  }
  return covariance;
}

bool isUsableCovariance(const Eigen::Matrix3d& covariance) {
  return covariance.allFinite() &&
         covariance.norm() > std::numeric_limits<double>::epsilon();
}

void drawRawPoseCovariance6x6(RosRerunVisualizer* visualizer,
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
  visualizer->drawScalar(
      base_path + "/summary/asymmetry_frobenius_norm",
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
  visualizer->drawScalar(base_path + "/eigenvalues/min",
                         eigenvalues.minCoeff());
  visualizer->drawScalar(base_path + "/eigenvalues/max",
                         eigenvalues.maxCoeff());
}

std::vector<gtsam::Point3> pointCloud2ToPoints(
    const sensor_msgs::PointCloud2& cloud,
    const int max_points) {
  std::vector<gtsam::Point3> points;
  const size_t total_points =
      static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  if (total_points == 0u) {
    return points;
  }

  const size_t capped_points =
      max_points > 0 ? std::min(total_points, static_cast<size_t>(max_points))
                     : total_points;
  points.reserve(capped_points);
  const size_t stride =
      capped_points < total_points
          ? std::max<size_t>(1u, (total_points + capped_points - 1u) /
                                     capped_points)
          : 1u;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    size_t index = 0u;
    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++index) {
      if (index % stride != 0u) {
        continue;
      }
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        points.emplace_back(x, y, z);
      }
      if (points.size() >= capped_points) {
        break;
      }
    }
  } catch (const std::exception& e) {
    LOG(WARNING) << "Could not convert PointCloud2 to Rerun points: "
                 << e.what();
  }

  return points;
}

struct SourceState {
  explicit SourceState(std::string source_name) : name(std::move(source_name)) {}

  std::string name;
  nav_msgs::Odometry latest;
  bool has_latest = false;
  double last_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  double stamp_interval_sec = std::numeric_limits<double>::quiet_NaN();
  double last_published_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  std::vector<gtsam::Pose3> trajectory;
};

struct CloudState {
  explicit CloudState(std::string source_name) : name(std::move(source_name)) {}

  std::string name;
  sensor_msgs::PointCloud2 latest;
  bool has_latest = false;
  double last_published_stamp_sec = std::numeric_limits<double>::quiet_NaN();
};

class RerunTopicVisualizer {
 public:
  RerunTopicVisualizer()
      : private_nh_("~"),
        kimera_("kimera"),
        liorf_("liorf"),
        liorf_local_map_("liorf_local_map"),
        liorf_current_scan_("liorf_current_scan"),
        kimera_landmarks_("kimera_landmarks"),
        visualizer_(nullptr) {
    private_nh_.param<std::string>(
        "kimera_odom_topic", kimera_odom_topic_, "/kimera_vio_ros/odometry");
    private_nh_.param<std::string>(
        "liorf_odom_topic", liorf_odom_topic_, "/liorf/mapping/odometry");
    private_nh_.param<double>("publish_rate_hz", publish_rate_hz_, 5.0);
    private_nh_.param<bool>("uncertainty_enable", uncertainty_enable_, true);
    private_nh_.param<bool>("raw_covariance_enable",
                            raw_covariance_enable_,
                            true);
    private_nh_.param<bool>("lag_scalars_enable", lag_scalars_enable_, true);
    private_nh_.param<bool>(
        "world_alignment_enable", world_alignment_enable_, true);
    private_nh_.param<int>("max_trajectory_len", max_trajectory_len_, 2000);
    private_nh_.param<bool>(
        "liorf_point_clouds_enable", liorf_point_clouds_enable_, true);
    private_nh_.param<std::string>("liorf_local_map_topic",
                                   liorf_local_map_topic_,
                                   "/liorf/mapping/map_local");
    private_nh_.param<std::string>("liorf_current_scan_topic",
                                   liorf_current_scan_topic_,
                                   "/liorf/mapping/cloud_registered");
    private_nh_.param<int>(
        "liorf_local_map_max_points", liorf_local_map_max_points_, 10000);
    private_nh_.param<int>(
        "liorf_current_scan_max_points", liorf_current_scan_max_points_, 20000);
    private_nh_.param<bool>(
        "kimera_landmarks_enable", kimera_landmarks_enable_, true);
    private_nh_.param<std::string>("kimera_landmarks_topic",
                                   kimera_landmarks_topic_,
                                   "/kimera_vio_ros/landmarks");
    private_nh_.param<int>(
        "kimera_landmarks_max_points", kimera_landmarks_max_points_, 3000);
    private_nh_.param<std::string>("rerun_recording_id", recording_id_, "");
    private_nh_.param<std::string>("rerun_host", rerun_host_, "auto");

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      LOG(WARNING) << "Invalid publish_rate_hz=" << publish_rate_hz_
                   << "; falling back to 5 Hz.";
      publish_rate_hz_ = 5.0;
    }
    max_trajectory_len_ = std::max(2, max_trajectory_len_);

    if (recording_id_.empty()) {
      ros::param::param<std::string>("/cbsms/rerun_recording_id",
                                     recording_id_,
                                     "");
    }
    if (recording_id_.empty()) {
      recording_id_ = makeRerunRecordingId("rerun_topic_visualizer");
    }
    if (rerun_host_.empty() || rerun_host_ == "auto") {
      rerun_host_ = defaultRerunHost();
    }

    visualizer_ = std::make_unique<RosRerunVisualizer>(
        "cbsms", recording_id_, rerun_host_);

    kimera_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        kimera_odom_topic_,
        1,
        &RerunTopicVisualizer::kimeraCallback,
        this,
        ros::TransportHints().tcpNoDelay());
    liorf_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        liorf_odom_topic_,
        1,
        &RerunTopicVisualizer::liorfCallback,
        this,
        ros::TransportHints().tcpNoDelay());
    if (liorf_point_clouds_enable_) {
      liorf_local_map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          liorf_local_map_topic_,
          1,
          &RerunTopicVisualizer::liorfLocalMapCallback,
          this,
          ros::TransportHints().tcpNoDelay());
      liorf_current_scan_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          liorf_current_scan_topic_,
          1,
          &RerunTopicVisualizer::liorfCurrentScanCallback,
          this,
          ros::TransportHints().tcpNoDelay());
    }
    if (kimera_landmarks_enable_) {
      kimera_landmarks_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          kimera_landmarks_topic_,
          1,
          &RerunTopicVisualizer::kimeraLandmarksCallback,
          this,
          ros::TransportHints().tcpNoDelay());
    }
    timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / publish_rate_hz_),
        &RerunTopicVisualizer::publishLatest,
        this);

    LOG(INFO) << "Rerun topic visualizer enabled. recording_id='"
              << recording_id_ << "', host='" << rerun_host_
              << "', kimera_odom_topic='" << kimera_odom_topic_
              << "', liorf_odom_topic='" << liorf_odom_topic_
              << "', publish_rate_hz=" << publish_rate_hz_
              << ", liorf_point_clouds_enable="
              << (liorf_point_clouds_enable_ ? "true" : "false")
              << ", kimera_landmarks_enable="
              << (kimera_landmarks_enable_ ? "true" : "false") << ".";
  }

 private:
  void kimeraCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    updateSource(msg, &kimera_);
  }

  void liorfCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    updateSource(msg, &liorf_);
  }

  void liorfLocalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    updateCloud(msg, &liorf_local_map_);
  }

  void liorfCurrentScanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    updateCloud(msg, &liorf_current_scan_);
  }

  void kimeraLandmarksCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    updateCloud(msg, &kimera_landmarks_);
  }

  void updateSource(const nav_msgs::Odometry::ConstPtr& msg,
                    SourceState* source) {
    CHECK_NOTNULL(msg);
    CHECK_NOTNULL(source);
    std::lock_guard<std::mutex> lock(mutex_);
    const double stamp_sec = msg->header.stamp.toSec();
    if (std::isfinite(source->last_stamp_sec)) {
      source->stamp_interval_sec = stamp_sec - source->last_stamp_sec;
    }
    source->last_stamp_sec = stamp_sec;
    source->latest = *msg;
    source->has_latest = true;
  }

  void updateCloud(const sensor_msgs::PointCloud2::ConstPtr& msg,
                   CloudState* cloud) {
    CHECK_NOTNULL(msg);
    CHECK_NOTNULL(cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    cloud->latest = *msg;
    cloud->has_latest = true;
  }

  void publishLatest(const ros::WallTimerEvent&) {
    nav_msgs::Odometry kimera_msg;
    nav_msgs::Odometry liorf_msg;
    bool has_kimera = false;
    bool has_liorf = false;
    double kimera_interval_sec = std::numeric_limits<double>::quiet_NaN();
    double liorf_interval_sec = std::numeric_limits<double>::quiet_NaN();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_kimera = kimera_.has_latest;
      has_liorf = liorf_.has_latest;
      if (has_kimera) {
        kimera_msg = kimera_.latest;
        kimera_interval_sec = kimera_.stamp_interval_sec;
      }
      if (has_liorf) {
        liorf_msg = liorf_.latest;
        liorf_interval_sec = liorf_.stamp_interval_sec;
      }
    }

    gtsam::Pose3 kimera_pose;
    gtsam::Pose3 liorf_pose;
    const bool kimera_pose_valid =
        has_kimera && odometryToPose(kimera_msg, &kimera_pose);
    const bool liorf_pose_valid =
        has_liorf && odometryToPose(liorf_msg, &liorf_pose);

    if (kimera_pose_valid) {
      publishSource(kimera_msg,
                    kimera_pose,
                    "kimera",
                    "base_link",
                    Eigen::Vector4f(40.f, 220.f, 80.f, 220.f),
                    &kimera_);
    }
    if (liorf_pose_valid) {
      publishSource(liorf_msg,
                    liorf_pose,
                    "liorf",
                    "lidar_link",
                    Eigen::Vector4f(245.f, 180.f, 20.f, 220.f),
                    &liorf_);
    }

    if (world_alignment_enable_ && kimera_pose_valid && liorf_pose_valid) {
      publishAlignedKimera(kimera_msg, kimera_pose, liorf_msg, liorf_pose);
    }

    if (lag_scalars_enable_ && (has_kimera || has_liorf)) {
      const uint64_t latest_stamp_nsec =
          std::max(has_kimera ? stampToNSec(kimera_msg.header.stamp) : 0ull,
                   has_liorf ? stampToNSec(liorf_msg.header.stamp) : 0ull);
      visualizer_->setTimeNSec(latest_stamp_nsec);
      const ros::Time now = ros::Time::now();
      if (has_kimera) {
        visualizer_->drawScalar("visualization/lag/kimera_stamp_age_sec",
                                (now - kimera_msg.header.stamp).toSec());
        if (std::isfinite(kimera_interval_sec)) {
          visualizer_->drawScalar(
              "visualization/rate/kimera_stamp_interval_sec",
              kimera_interval_sec);
        }
      }
      if (has_liorf) {
        visualizer_->drawScalar("visualization/lag/liorf_stamp_age_sec",
                                (now - liorf_msg.header.stamp).toSec());
        if (std::isfinite(liorf_interval_sec)) {
          visualizer_->drawScalar(
              "visualization/rate/liorf_stamp_interval_sec",
              liorf_interval_sec);
        }
      }
      if (has_kimera && has_liorf) {
        visualizer_->drawScalar(
            "visualization/lag/kimera_minus_liorf_stamp_sec",
            kimera_msg.header.stamp.toSec() - liorf_msg.header.stamp.toSec());
      }
    }

    if (liorf_point_clouds_enable_) {
      publishCloudLatest(liorf_local_map_,
                         "liorf/local_map",
                         Eigen::Vector4f(80.f, 180.f, 255.f, 90.f),
                         1.0f,
                         liorf_local_map_max_points_);
      publishCloudLatest(liorf_current_scan_,
                         "liorf/current_scan",
                         Eigen::Vector4f(255.f, 255.f, 255.f, 180.f),
                         1.5f,
                         liorf_current_scan_max_points_);
    }
    if (kimera_landmarks_enable_) {
      publishKimeraLandmarksLatest();
    }
  }

  bool takeCloudForPublish(CloudState* cloud, sensor_msgs::PointCloud2* msg) {
    CHECK_NOTNULL(cloud);
    CHECK_NOTNULL(msg);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cloud->has_latest) {
      return false;
    }
    const double stamp_sec = cloud->latest.header.stamp.toSec();
    if (std::isfinite(cloud->last_published_stamp_sec) &&
        stamp_sec <= cloud->last_published_stamp_sec) {
      return false;
    }
    cloud->last_published_stamp_sec = stamp_sec;
    *msg = cloud->latest;
    return true;
  }

  void publishCloudLatest(CloudState& cloud,
                          const std::string& entity_path,
                          const Eigen::Vector4f& rgba,
                          const float radius,
                          const int max_points) {
    sensor_msgs::PointCloud2 msg;
    if (!takeCloudForPublish(&cloud, &msg)) {
      return;
    }

    const std::vector<gtsam::Point3> points =
        pointCloud2ToPoints(msg, max_points);
    if (points.empty()) {
      return;
    }
    visualizer_->setTimeNSec(stampToNSec(msg.header.stamp));
    visualizer_->drawPoints(entity_path, points, rgba, radius);
  }

  void publishKimeraLandmarksLatest() {
    sensor_msgs::PointCloud2 msg;
    if (!takeCloudForPublish(&kimera_landmarks_, &msg)) {
      return;
    }

    const std::vector<gtsam::Point3> landmarks =
        pointCloud2ToPoints(msg, kimera_landmarks_max_points_);
    if (landmarks.empty()) {
      return;
    }

    const Eigen::Vector4f rgba(40.f, 220.f, 80.f, 180.f);
    visualizer_->setTimeNSec(stampToNSec(msg.header.stamp));
    visualizer_->drawPoints("kimera/landmarks", landmarks, rgba, 2.f);

    if (!alignment_initialized_) {
      return;
    }
    std::vector<gtsam::Point3> aligned_landmarks;
    aligned_landmarks.reserve(landmarks.size());
    for (const auto& landmark : landmarks) {
      aligned_landmarks.push_back(
          liorf_T_kimera_world_.transformFrom(landmark));
    }
    visualizer_->drawPoints(
        "kimera_aligned/landmarks", aligned_landmarks, rgba, 2.f);
  }

  void publishSource(const nav_msgs::Odometry& msg,
                     const gtsam::Pose3& pose,
                     const std::string& entity_prefix,
                     const std::string& frame_name,
                     const Eigen::Vector4f& rgba,
                     SourceState* source) {
    CHECK_NOTNULL(source);
    const double stamp_sec = msg.header.stamp.toSec();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (std::isfinite(source->last_published_stamp_sec) &&
          stamp_sec <= source->last_published_stamp_sec) {
        return;
      }
      source->last_published_stamp_sec = stamp_sec;
      source->trajectory.push_back(pose);
      if (source->trajectory.size() >
          static_cast<size_t>(max_trajectory_len_)) {
        const size_t extra = source->trajectory.size() -
                             static_cast<size_t>(max_trajectory_len_);
        source->trajectory.erase(source->trajectory.begin(),
                                 source->trajectory.begin() + extra);
      }
    }

    std::vector<gtsam::Pose3> trajectory;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      trajectory = source->trajectory;
    }

    visualizer_->setTimeNSec(stampToNSec(msg.header.stamp));
    visualizer_->drawTf(entity_prefix + "/" + frame_name, pose, 0.5f);
    if (trajectory.size() > 1u) {
      visualizer_->drawTrajectory(
          entity_prefix + "/trajectory", trajectory, rgba, 1.5f);
    }

    if (uncertainty_enable_) {
      const gtsam::Matrix6 pose_covariance = poseCovarianceFromOdometry(msg);
      if (raw_covariance_enable_) {
        drawRawPoseCovariance6x6(
            visualizer_.get(),
            entity_prefix + "/current_pose/raw_pose_covariance_6x6",
            pose_covariance);
      }
      const Eigen::Matrix3d covariance =
          translationCovarianceFromPoseCovariance(pose_covariance);
      if (isUsableCovariance(covariance)) {
        visualizer_->drawUncertainty(
            entity_prefix + "/current_pose/uncertainty",
            pose,
            covariance,
            rgba,
            1.25f);
        visualizer_->drawScalar(
            entity_prefix + "/current_pose/uncertainty_frobenius_norm",
            covariance.norm());
      }
    }
  }

  void publishAlignedKimera(const nav_msgs::Odometry& kimera_msg,
                            const gtsam::Pose3& kimera_pose,
                            const nav_msgs::Odometry& liorf_msg,
                            const gtsam::Pose3& liorf_pose) {
    if (!alignment_initialized_) {
      liorf_T_kimera_world_ = liorf_pose * kimera_pose.inverse();
      alignment_initialized_ = true;
      alignment_timestamp_delta_sec_ =
          liorf_msg.header.stamp.toSec() - kimera_msg.header.stamp.toSec();
      LOG(INFO) << "Rerun topic visualizer initialized Kimera->LiORF "
                << "world alignment. timestamp_delta_sec="
                << alignment_timestamp_delta_sec_ << ".";
    }

    const double stamp_sec = kimera_msg.header.stamp.toSec();
    if (std::isfinite(last_aligned_kimera_stamp_sec_) &&
        stamp_sec <= last_aligned_kimera_stamp_sec_) {
      return;
    }
    last_aligned_kimera_stamp_sec_ = stamp_sec;

    const gtsam::Pose3 aligned_pose = liorf_T_kimera_world_ * kimera_pose;
    aligned_kimera_trajectory_.push_back(aligned_pose);
    if (aligned_kimera_trajectory_.size() >
        static_cast<size_t>(max_trajectory_len_)) {
      const size_t extra = aligned_kimera_trajectory_.size() -
                           static_cast<size_t>(max_trajectory_len_);
      aligned_kimera_trajectory_.erase(aligned_kimera_trajectory_.begin(),
                                       aligned_kimera_trajectory_.begin() +
                                           extra);
    }

    const Eigen::Vector4f rgba(40.f, 220.f, 80.f, 220.f);
    visualizer_->setTimeNSec(stampToNSec(kimera_msg.header.stamp));
    visualizer_->drawTf("kimera_aligned/base_link", aligned_pose, 0.6f);
    if (aligned_kimera_trajectory_.size() > 1u) {
      visualizer_->drawTrajectory("kimera_aligned/trajectory",
                                  aligned_kimera_trajectory_,
                                  rgba,
                                  1.75f);
    }
    visualizer_->drawScalar("kimera_aligned/alignment/initialized", 1.0);
    visualizer_->drawScalar("kimera_aligned/alignment/timestamp_delta_sec",
                            alignment_timestamp_delta_sec_);

    if (uncertainty_enable_) {
      const Eigen::Matrix3d covariance =
          translationCovarianceFromPoseCovariance(
              poseCovarianceFromOdometry(kimera_msg));
      if (isUsableCovariance(covariance)) {
        const Eigen::Matrix3d rotation =
            liorf_T_kimera_world_.rotation().matrix();
        visualizer_->drawUncertainty(
            "kimera_aligned/current_pose/uncertainty",
            aligned_pose,
            rotation * covariance * rotation.transpose(),
            rgba,
            1.25f);
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber kimera_sub_;
  ros::Subscriber liorf_sub_;
  ros::Subscriber liorf_local_map_sub_;
  ros::Subscriber liorf_current_scan_sub_;
  ros::Subscriber kimera_landmarks_sub_;
  ros::WallTimer timer_;
  std::mutex mutex_;
  SourceState kimera_;
  SourceState liorf_;
  CloudState liorf_local_map_;
  CloudState liorf_current_scan_;
  CloudState kimera_landmarks_;
  std::unique_ptr<RosRerunVisualizer> visualizer_;

  std::string kimera_odom_topic_;
  std::string liorf_odom_topic_;
  std::string liorf_local_map_topic_;
  std::string liorf_current_scan_topic_;
  std::string kimera_landmarks_topic_;
  std::string recording_id_;
  std::string rerun_host_;
  double publish_rate_hz_ = 5.0;
  bool uncertainty_enable_ = true;
  bool raw_covariance_enable_ = true;
  bool lag_scalars_enable_ = true;
  bool world_alignment_enable_ = true;
  bool liorf_point_clouds_enable_ = true;
  bool kimera_landmarks_enable_ = true;
  int max_trajectory_len_ = 2000;
  int liorf_local_map_max_points_ = 10000;
  int liorf_current_scan_max_points_ = 20000;
  int kimera_landmarks_max_points_ = 3000;

  bool alignment_initialized_ = false;
  double alignment_timestamp_delta_sec_ = 0.0;
  double last_aligned_kimera_stamp_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  gtsam::Pose3 liorf_T_kimera_world_;
  std::vector<gtsam::Pose3> aligned_kimera_trajectory_;
};

}  // namespace
}  // namespace VIO

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "rerun_topic_visualizer_node");
  VIO::RerunTopicVisualizer visualizer;
  ros::spin();
  return 0;
}
