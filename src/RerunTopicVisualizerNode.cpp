/* -----------------------------------------------------------------------------
 * Copyright 2026.
 * ----------------------------------------------------------------------------*/

#include "kimera_vio_ros/RosRerunVisualizer.h"

#include <glog/logging.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <geometry_msgs/TransformStamped.h>
#include <liorf/pose_odom_belief_array.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rerun.hpp>
#include <rerun/blueprint/archetypes/container_blueprint.hpp>
#include <rerun/blueprint/archetypes/time_panel_blueprint.hpp>
#include <rerun/blueprint/archetypes/view_blueprint.hpp>
#include <rerun/blueprint/archetypes/view_contents.hpp>
#include <rerun/blueprint/archetypes/viewport_blueprint.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <opencv2/imgcodecs.hpp>

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

std::string normalizeEntityPrefix(std::string prefix,
                                  const std::string& fallback) {
  while (!prefix.empty() && prefix.front() == '/') {
    prefix.erase(prefix.begin());
  }
  while (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }
  return prefix.empty() ? fallback : prefix;
}

uint64_t fnv1a64(const std::string& text, const uint64_t salt) {
  uint64_t hash = 1469598103934665603ull ^ salt;
  for (const unsigned char ch : text) {
    hash ^= static_cast<uint64_t>(ch);
    hash *= 1099511628211ull;
  }
  return hash;
}

rerun::datatypes::Uuid makeDeterministicUuid(const std::string& seed) {
  const uint64_t hi = fnv1a64("hi:" + seed, 0x9e3779b97f4a7c15ull);
  const uint64_t lo = fnv1a64("lo:" + seed, 0xbf58476d1ce4e5b9ull);

  std::array<uint8_t, 16> bytes{};
  for (int i = 0; i < 8; ++i) {
    bytes[static_cast<size_t>(i)] =
        static_cast<uint8_t>((hi >> ((7 - i) * 8)) & 0xffull);
    bytes[static_cast<size_t>(8 + i)] =
        static_cast<uint8_t>((lo >> ((7 - i) * 8)) & 0xffull);
  }

  bytes[6] = static_cast<uint8_t>((bytes[6] & 0x0f) | 0x40);
  bytes[8] = static_cast<uint8_t>((bytes[8] & 0x3f) | 0x80);
  return rerun::datatypes::Uuid(bytes);
}

std::string uuidToString(const rerun::datatypes::Uuid& uuid) {
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0u; i < uuid.bytes.size(); ++i) {
    oss << std::setw(2) << static_cast<int>(uuid.bytes[i]);
    if (i == 3u || i == 5u || i == 7u || i == 9u) {
      oss << '-';
    }
  }
  return oss.str();
}

std::string blueprintViewPath(const std::string& key) {
  return "view/" + uuidToString(makeDeterministicUuid("view:" + key));
}

std::string blueprintContainerPath(const std::string& key) {
  return "container/" + uuidToString(makeDeterministicUuid("container:" + key));
}

void logBlueprintView(rerun::RecordingStream* stream,
                      const std::string& key,
                      const std::string& class_identifier,
                      const std::string& display_name,
                      const std::string& space_origin,
                      const std::vector<std::string>& queries) {
  CHECK_NOTNULL(stream);

  const std::string path = blueprintViewPath(key);
  stream->log(
      path,
      rerun::blueprint::archetypes::ViewBlueprint(
          rerun::blueprint::components::ViewClass(class_identifier))
          .with_display_name(rerun::components::Name(display_name))
          .with_space_origin(
              rerun::blueprint::components::ViewOrigin(space_origin))
          .with_visible(rerun::components::Visible(true)));

  std::vector<rerun::blueprint::components::QueryExpression> filters;
  filters.reserve(queries.size());
  for (const std::string& query : queries) {
    filters.emplace_back(query);
  }
  stream->log(path + "/ViewContents",
              rerun::blueprint::archetypes::ViewContents(filters));
}

void logBlueprintContainer(
    rerun::RecordingStream* stream,
    const std::string& key,
    const rerun::blueprint::components::ContainerKind kind,
    const std::string& display_name,
    const std::vector<std::string>& children,
    const std::vector<float>& col_shares = {},
    const std::vector<float>& row_shares = {},
    const uint32_t grid_columns = 0u) {
  CHECK_NOTNULL(stream);

  std::vector<rerun::blueprint::components::IncludedContent> contents;
  contents.reserve(children.size());
  for (const std::string& child : children) {
    contents.emplace_back(child);
  }

  auto container = rerun::blueprint::archetypes::ContainerBlueprint(kind)
                       .with_display_name(rerun::components::Name(display_name))
                       .with_contents(contents)
                       .with_visible(rerun::components::Visible(true));

  if (!col_shares.empty()) {
    std::vector<rerun::blueprint::components::ColumnShare> shares;
    shares.reserve(col_shares.size());
    for (const float share : col_shares) {
      shares.emplace_back(share);
    }
    container = std::move(container).with_col_shares(shares);
  }

  if (!row_shares.empty()) {
    std::vector<rerun::blueprint::components::RowShare> shares;
    shares.reserve(row_shares.size());
    for (const float share : row_shares) {
      shares.emplace_back(share);
    }
    container = std::move(container).with_row_shares(shares);
  }

  if (grid_columns > 0u) {
    container = std::move(container).with_grid_columns(
        rerun::blueprint::components::GridColumns(grid_columns));
  }

  stream->log(blueprintContainerPath(key), std::move(container));
}

uint64_t stampToNSec(const ros::Time& stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ull +
         static_cast<uint64_t>(stamp.nsec);
}

uint64_t stampSecToNSec(const double stamp_sec) {
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return 0ull;
  }
  return static_cast<uint64_t>(std::llround(stamp_sec * 1.0e9));
}

double beliefMessageStampSec(const liorf::pose_odom_belief_array& msg);

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

gtsam::Matrix6 rotatePoseCovariance6x6(const gtsam::Matrix6& pose_covariance,
                                       const Eigen::Matrix3d& rotation) {
  gtsam::Matrix6 transform = gtsam::Matrix6::Zero();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 3>(3, 3) = rotation;
  return transform * pose_covariance * transform.transpose();
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

struct CloudState {
  explicit CloudState(std::string source_name) : name(std::move(source_name)) {}

  std::string name;
  sensor_msgs::PointCloud2 latest;
  bool has_latest = false;
  double last_published_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  bool raw_entity_cleared = false;
};

struct ImageState {
  explicit ImageState(std::string source_name) : name(std::move(source_name)) {}

  std::string name;
  sensor_msgs::CompressedImage latest;
  bool has_latest = false;
  double last_published_stamp_sec = std::numeric_limits<double>::quiet_NaN();
};

struct GroundTruthPose {
  double stamp_sec = std::numeric_limits<double>::quiet_NaN();
  gtsam::Pose3 pose;
};

struct TimedPose {
  double stamp_sec = std::numeric_limits<double>::quiet_NaN();
  gtsam::Pose3 pose;
};

struct SourceState {
  explicit SourceState(std::string source_name) : name(std::move(source_name)) {}

  std::string name;
  nav_msgs::Odometry latest;
  bool has_latest = false;
  double last_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  double stamp_interval_sec = std::numeric_limits<double>::quiet_NaN();
  double last_published_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  double last_timed_trajectory_stamp_sec =
      std::numeric_limits<double>::quiet_NaN();
  std::vector<gtsam::Pose3> trajectory;
  std::vector<TimedPose> timed_trajectory;
};

struct BeliefTrafficStats {
  size_t beliefs_per_message = 0u;
  size_t beliefs_total = 0u;
  size_t messages_total = 0u;
  double message_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  double message_interval_sec = std::numeric_limits<double>::quiet_NaN();
  double edge_duration_mean_sec = std::numeric_limits<double>::quiet_NaN();
  double edge_duration_min_sec = std::numeric_limits<double>::quiet_NaN();
  double edge_duration_max_sec = std::numeric_limits<double>::quiet_NaN();
  double covariance_trace_mean = std::numeric_limits<double>::quiet_NaN();
  double covariance_frobenius_mean = std::numeric_limits<double>::quiet_NaN();
  double latest_source_agent = std::numeric_limits<double>::quiet_NaN();
  double latest_from_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  double latest_to_stamp_sec = std::numeric_limits<double>::quiet_NaN();
};

struct AccumulatedBeliefCovarianceState {
  explicit AccumulatedBeliefCovarianceState(std::string source)
      : source_name(std::move(source)) {}

  std::string source_name;
  bool initialized = false;
  uint32_t chain_start_index = 0u;
  uint32_t last_to_index = 0u;
  double chain_start_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  double last_to_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  gtsam::Pose3 accumulated_pose;
  gtsam::Matrix6 accumulated_covariance = gtsam::Matrix6::Zero();
  size_t accepted_edges = 0u;
  size_t skipped_duplicate_total = 0u;
  size_t skipped_noncontiguous_total = 0u;
  size_t skipped_invalid_total = 0u;
};

struct AccumulatedBeliefCovarianceSnapshot {
  std::string source_name;
  double stamp_sec = std::numeric_limits<double>::quiet_NaN();
  bool initialized = false;
  bool has_relative_covariance = false;
  bool has_accumulated_covariance = false;
  gtsam::Matrix6 relative_pose_covariance = gtsam::Matrix6::Zero();
  Eigen::Matrix3d relative_translation_covariance = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d accumulated_translation_covariance = Eigen::Matrix3d::Zero();
  double edge_duration_sec = std::numeric_limits<double>::quiet_NaN();
  double chain_duration_sec = std::numeric_limits<double>::quiet_NaN();
  uint32_t latest_from_index = 0u;
  uint32_t latest_to_index = 0u;
  size_t edge_count = 0u;
  size_t skipped_duplicate_total = 0u;
  size_t skipped_noncontiguous_total = 0u;
  size_t skipped_invalid_total = 0u;
};

struct BeliefTrafficState {
  BeliefTrafficState(std::string source_name,
                     std::string entity_path,
                     std::string covariance_source_name)
      : name(std::move(source_name)),
        entity(std::move(entity_path)),
        accumulated_covariance(std::move(covariance_source_name)) {}

  std::string name;
  std::string entity;
  AccumulatedBeliefCovarianceState accumulated_covariance;
  double last_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  size_t messages_total = 0u;
  size_t beliefs_total = 0u;
};

gtsam::Pose3 relativePoseFromBelief(const liorf::pose_odom_belief& belief) {
  gtsam::Vector6 relative_mu;
  for (size_t i = 0u; i < 6u; ++i) {
    relative_mu(i) = belief.relative_mu[i];
  }
  return gtsam::Pose3::Expmap(relative_mu);
}

gtsam::Matrix6 covarianceFromBelief(const liorf::pose_odom_belief& belief) {
  gtsam::Matrix6 covariance = gtsam::Matrix6::Zero();
  for (size_t row = 0u; row < 6u; ++row) {
    for (size_t col = 0u; col < 6u; ++col) {
      covariance(row, col) = belief.covariance[row * 6u + col];
    }
  }
  return covariance;
}

bool isFinitePoseCovariance(const gtsam::Matrix6& covariance) {
  return covariance.allFinite();
}

gtsam::Matrix6 symmetrizePoseCovariance(const gtsam::Matrix6& covariance) {
  return 0.5 * (covariance + covariance.transpose());
}

bool isValidBeliefEdge(const liorf::pose_odom_belief& belief,
                       gtsam::Pose3* relative_pose,
                       gtsam::Matrix6* covariance) {
  CHECK_NOTNULL(relative_pose);
  CHECK_NOTNULL(covariance);
  if (belief.to_pose_index <= belief.from_pose_index ||
      !std::isfinite(belief.from_stamp_sec) ||
      !std::isfinite(belief.to_stamp_sec) ||
      belief.to_stamp_sec < belief.from_stamp_sec) {
    return false;
  }

  for (size_t i = 0u; i < 6u; ++i) {
    if (!std::isfinite(belief.relative_mu[i])) {
      return false;
    }
  }

  *relative_pose = relativePoseFromBelief(belief);
  if (!relative_pose->matrix().allFinite()) {
    return false;
  }

  *covariance = symmetrizePoseCovariance(covarianceFromBelief(belief));
  return isFinitePoseCovariance(*covariance);
}

AccumulatedBeliefCovarianceSnapshot makeAccumulatedCovarianceSnapshot(
    const AccumulatedBeliefCovarianceState& state,
    const double stamp_sec) {
  AccumulatedBeliefCovarianceSnapshot snapshot;
  snapshot.source_name = state.source_name;
  snapshot.stamp_sec = stamp_sec;
  snapshot.initialized = state.initialized;
  snapshot.edge_count = state.accepted_edges;
  snapshot.skipped_duplicate_total = state.skipped_duplicate_total;
  snapshot.skipped_noncontiguous_total = state.skipped_noncontiguous_total;
  snapshot.skipped_invalid_total = state.skipped_invalid_total;
  snapshot.latest_to_index = state.last_to_index;
  if (state.initialized && std::isfinite(state.chain_start_stamp_sec) &&
      std::isfinite(state.last_to_stamp_sec)) {
    snapshot.chain_duration_sec =
        state.last_to_stamp_sec - state.chain_start_stamp_sec;
  }
  return snapshot;
}

void appendAccumulatedBeliefCovarianceSnapshots(
    const liorf::pose_odom_belief_array& msg,
    AccumulatedBeliefCovarianceState* state,
    std::vector<AccumulatedBeliefCovarianceSnapshot>* snapshots) {
  CHECK_NOTNULL(state);
  CHECK_NOTNULL(snapshots);

  std::vector<const liorf::pose_odom_belief*> beliefs;
  beliefs.reserve(msg.beliefs.size());
  for (const auto& belief : msg.beliefs) {
    beliefs.push_back(&belief);
  }
  std::sort(beliefs.begin(),
            beliefs.end(),
            [](const liorf::pose_odom_belief* lhs,
               const liorf::pose_odom_belief* rhs) {
              if (lhs->from_pose_index != rhs->from_pose_index) {
                return lhs->from_pose_index < rhs->from_pose_index;
              }
              return lhs->to_pose_index < rhs->to_pose_index;
            });

  for (const liorf::pose_odom_belief* belief_ptr : beliefs) {
    const liorf::pose_odom_belief& belief = *belief_ptr;
    gtsam::Pose3 relative_pose;
    gtsam::Matrix6 relative_covariance = gtsam::Matrix6::Zero();
    if (!isValidBeliefEdge(belief, &relative_pose, &relative_covariance)) {
      ++state->skipped_invalid_total;
      continue;
    }

    if (state->initialized) {
      if (belief.to_pose_index <= state->last_to_index ||
          belief.from_pose_index < state->last_to_index) {
        ++state->skipped_duplicate_total;
        continue;
      }
      if (belief.from_pose_index != state->last_to_index) {
        ++state->skipped_noncontiguous_total;
        continue;
      }

      const gtsam::Matrix6 adjoint =
          relative_pose.inverse().AdjointMap();
      state->accumulated_covariance =
          symmetrizePoseCovariance(adjoint * state->accumulated_covariance *
                                       adjoint.transpose() +
                                   relative_covariance);
      state->accumulated_pose = state->accumulated_pose.compose(relative_pose);
      state->last_to_index = belief.to_pose_index;
      state->last_to_stamp_sec = belief.to_stamp_sec;
      ++state->accepted_edges;
    } else {
      state->initialized = true;
      state->chain_start_index = belief.from_pose_index;
      state->last_to_index = belief.to_pose_index;
      state->chain_start_stamp_sec = belief.from_stamp_sec;
      state->last_to_stamp_sec = belief.to_stamp_sec;
      state->accumulated_pose = relative_pose;
      state->accumulated_covariance = relative_covariance;
      state->accepted_edges = 1u;
    }

    AccumulatedBeliefCovarianceSnapshot snapshot =
        makeAccumulatedCovarianceSnapshot(*state, belief.to_stamp_sec);
    snapshot.has_relative_covariance = true;
    snapshot.has_accumulated_covariance = true;
    snapshot.relative_pose_covariance = relative_covariance;
    snapshot.relative_translation_covariance =
        translationCovarianceFromPoseCovariance(relative_covariance);
    snapshot.accumulated_translation_covariance =
        translationCovarianceFromPoseCovariance(state->accumulated_covariance);
    snapshot.edge_duration_sec = belief.to_stamp_sec - belief.from_stamp_sec;
    snapshot.latest_from_index = belief.from_pose_index;
    snapshot.latest_to_index = belief.to_pose_index;
    snapshots->push_back(snapshot);
  }

  snapshots->push_back(
      makeAccumulatedCovarianceSnapshot(*state, beliefMessageStampSec(msg)));
}

double beliefMessageStampSec(const liorf::pose_odom_belief_array& msg) {
  const double header_stamp_sec = msg.header.stamp.toSec();
  if (std::isfinite(header_stamp_sec) && header_stamp_sec > 0.0) {
    return header_stamp_sec;
  }

  double latest_belief_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  for (const auto& belief : msg.beliefs) {
    const double to_stamp_sec =
        belief.to_stamp_sec > 0.0 ? belief.to_stamp_sec
                                  : belief.header.stamp.toSec();
    if (std::isfinite(to_stamp_sec) && to_stamp_sec > 0.0 &&
        (!std::isfinite(latest_belief_stamp_sec) ||
         to_stamp_sec > latest_belief_stamp_sec)) {
      latest_belief_stamp_sec = to_stamp_sec;
    }
  }
  if (std::isfinite(latest_belief_stamp_sec)) {
    return latest_belief_stamp_sec;
  }

  return ros::Time::now().toSec();
}

BeliefTrafficStats summarizeBeliefTraffic(
    const liorf::pose_odom_belief_array& msg,
    BeliefTrafficState* state) {
  CHECK_NOTNULL(state);
  BeliefTrafficStats stats;
  stats.beliefs_per_message = msg.beliefs.size();
  stats.message_stamp_sec = beliefMessageStampSec(msg);
  if (std::isfinite(state->last_stamp_sec)) {
    stats.message_interval_sec = stats.message_stamp_sec - state->last_stamp_sec;
  }
  state->last_stamp_sec = stats.message_stamp_sec;
  ++state->messages_total;
  state->beliefs_total += stats.beliefs_per_message;
  stats.messages_total = state->messages_total;
  stats.beliefs_total = state->beliefs_total;

  double duration_sum = 0.0;
  double duration_min = std::numeric_limits<double>::infinity();
  double duration_max = -std::numeric_limits<double>::infinity();
  size_t duration_count = 0u;
  double covariance_trace_sum = 0.0;
  double covariance_frobenius_sum = 0.0;
  size_t covariance_count = 0u;

  for (const auto& belief : msg.beliefs) {
    const double to_stamp_sec =
        belief.to_stamp_sec > 0.0 ? belief.to_stamp_sec
                                  : belief.header.stamp.toSec();
    const double from_stamp_sec = belief.from_stamp_sec;
    if (std::isfinite(from_stamp_sec) && std::isfinite(to_stamp_sec) &&
        to_stamp_sec >= from_stamp_sec) {
      const double duration_sec = to_stamp_sec - from_stamp_sec;
      duration_sum += duration_sec;
      duration_min = std::min(duration_min, duration_sec);
      duration_max = std::max(duration_max, duration_sec);
      ++duration_count;
    }

    bool covariance_finite = true;
    double trace = 0.0;
    double squared_norm = 0.0;
    for (size_t row = 0u; row < 6u; ++row) {
      for (size_t col = 0u; col < 6u; ++col) {
        const double value = belief.covariance[row * 6u + col];
        covariance_finite = covariance_finite && std::isfinite(value);
        squared_norm += value * value;
        if (row == col) {
          trace += value;
        }
      }
    }
    if (covariance_finite) {
      covariance_trace_sum += trace;
      covariance_frobenius_sum += std::sqrt(squared_norm);
      ++covariance_count;
    }

    stats.latest_source_agent = static_cast<double>(belief.source_agent);
    stats.latest_from_stamp_sec = from_stamp_sec;
    stats.latest_to_stamp_sec = to_stamp_sec;
  }

  if (duration_count > 0u) {
    stats.edge_duration_mean_sec =
        duration_sum / static_cast<double>(duration_count);
    stats.edge_duration_min_sec = duration_min;
    stats.edge_duration_max_sec = duration_max;
  }
  if (covariance_count > 0u) {
    stats.covariance_trace_mean =
        covariance_trace_sum / static_cast<double>(covariance_count);
    stats.covariance_frobenius_mean =
        covariance_frobenius_sum / static_cast<double>(covariance_count);
  }

  return stats;
}

bool loadGroundTruthTrajectory(const std::string& path,
                               std::vector<GroundTruthPose>* poses) {
  CHECK_NOTNULL(poses);
  poses->clear();
  if (path.empty()) {
    return false;
  }

  std::ifstream file(path);
  if (!file.is_open()) {
    LOG(WARNING) << "Could not open Rerun ground-truth trajectory: " << path;
    return false;
  }

  std::string line;
  size_t line_number = 0u;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream iss(line);
    double stamp_sec = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!(iss >> stamp_sec >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      LOG(WARNING) << "Skipping malformed Rerun ground-truth row "
                   << line_number << " in " << path;
      continue;
    }

    const double q_norm =
        std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (!std::isfinite(stamp_sec) || !std::isfinite(x) ||
        !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(q_norm) ||
        q_norm < kMinQuaternionNorm) {
      continue;
    }

    poses->push_back(
        {stamp_sec,
         gtsam::Pose3(gtsam::Rot3::Quaternion(qw / q_norm,
                                              qx / q_norm,
                                              qy / q_norm,
                                              qz / q_norm),
                      gtsam::Point3(x, y, z))});
  }

  std::sort(poses->begin(),
            poses->end(),
            [](const GroundTruthPose& lhs, const GroundTruthPose& rhs) {
              return lhs.stamp_sec < rhs.stamp_sec;
            });

  if (poses->empty()) {
    LOG(WARNING) << "No valid Rerun ground-truth poses loaded from " << path;
    return false;
  }

  LOG(INFO) << "Loaded " << poses->size()
            << " Rerun ground-truth poses from " << path << ".";
  return true;
}

gtsam::Pose3 transformStampedToPose3(
    const geometry_msgs::TransformStamped& tf_msg) {
  const auto& t = tf_msg.transform.translation;
  const auto& q = tf_msg.transform.rotation;
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                      gtsam::Point3(t.x, t.y, t.z));
}

double pathLength2D(const std::vector<gtsam::Point3>& points) {
  if (points.size() < 2u) {
    return 0.0;
  }

  double length = 0.0;
  for (size_t i = 1u; i < points.size(); ++i) {
    const double dx = points[i].x() - points[i - 1u].x();
    const double dy = points[i].y() - points[i - 1u].y();
    length += std::hypot(dx, dy);
  }
  return length;
}

struct AlignmentEstimate {
  gtsam::Pose3 target_T_source;
  double rmse_m = std::numeric_limits<double>::quiet_NaN();
  double mean_m = std::numeric_limits<double>::quiet_NaN();
  double max_m = std::numeric_limits<double>::quiet_NaN();
  double determinant = std::numeric_limits<double>::quiet_NaN();
};

struct Se2AlignmentEstimate {
  double yaw_rad = 0.0;
  gtsam::Point3 translation = gtsam::Point3(0.0, 0.0, 0.0);
  double rmse_m = std::numeric_limits<double>::quiet_NaN();
  double mean_m = std::numeric_limits<double>::quiet_NaN();
  double max_m = std::numeric_limits<double>::quiet_NaN();
};

bool estimateSe3Alignment(const std::vector<gtsam::Point3>& source_points,
                          const std::vector<gtsam::Point3>& target_points,
                          AlignmentEstimate* estimate) {
  CHECK_NOTNULL(estimate);
  if (source_points.size() != target_points.size() ||
      source_points.size() < 3u) {
    return false;
  }

  Eigen::Vector3d source_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_mean = Eigen::Vector3d::Zero();
  for (size_t i = 0u; i < source_points.size(); ++i) {
    source_mean += Eigen::Vector3d(source_points[i].x(),
                                   source_points[i].y(),
                                   source_points[i].z());
    target_mean += Eigen::Vector3d(target_points[i].x(),
                                   target_points[i].y(),
                                   target_points[i].z());
  }

  const double inv_count = 1.0 / static_cast<double>(source_points.size());
  source_mean *= inv_count;
  target_mean *= inv_count;

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (size_t i = 0u; i < source_points.size(); ++i) {
    const Eigen::Vector3d source_centered =
        Eigen::Vector3d(source_points[i].x(),
                        source_points[i].y(),
                        source_points[i].z()) -
        source_mean;
    const Eigen::Vector3d target_centered =
        Eigen::Vector3d(target_points[i].x(),
                        target_points[i].y(),
                        target_points[i].z()) -
        target_mean;
    covariance += source_centered * target_centered.transpose();
  }

  if (!covariance.allFinite() || covariance.norm() < 1e-9) {
    return false;
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success) {
    return false;
  }

  Eigen::Matrix3d u = svd.matrixU();
  Eigen::Matrix3d v = svd.matrixV();
  Eigen::Matrix3d rotation = v * u.transpose();
  if (rotation.determinant() < 0.0) {
    v.col(2) *= -1.0;
    rotation = v * u.transpose();
  }
  if (!rotation.allFinite()) {
    return false;
  }

  const Eigen::Vector3d translation = target_mean - rotation * source_mean;
  estimate->target_T_source =
      gtsam::Pose3(gtsam::Rot3(rotation),
                   gtsam::Point3(translation.x(),
                                 translation.y(),
                                 translation.z()));
  estimate->determinant = rotation.determinant();

  double squared_error_sum = 0.0;
  double error_sum = 0.0;
  double max_error = 0.0;
  for (size_t i = 0u; i < source_points.size(); ++i) {
    const Eigen::Vector3d source(source_points[i].x(),
                                 source_points[i].y(),
                                 source_points[i].z());
    const Eigen::Vector3d target(target_points[i].x(),
                                 target_points[i].y(),
                                 target_points[i].z());
    const double error = (rotation * source + translation - target).norm();
    squared_error_sum += error * error;
    error_sum += error;
    max_error = std::max(max_error, error);
  }
  estimate->rmse_m =
      std::sqrt(squared_error_sum / static_cast<double>(source_points.size()));
  estimate->mean_m = error_sum / static_cast<double>(source_points.size());
  estimate->max_m = max_error;
  return true;
}

bool estimateSe2Alignment(const std::vector<gtsam::Point3>& source_points,
                          const std::vector<gtsam::Point3>& target_points,
                          Se2AlignmentEstimate* estimate) {
  CHECK_NOTNULL(estimate);
  if (source_points.size() != target_points.size() ||
      source_points.size() < 2u) {
    return false;
  }

  Eigen::Vector3d source_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_mean = Eigen::Vector3d::Zero();
  for (size_t i = 0u; i < source_points.size(); ++i) {
    source_mean += Eigen::Vector3d(source_points[i].x(),
                                   source_points[i].y(),
                                   source_points[i].z());
    target_mean += Eigen::Vector3d(target_points[i].x(),
                                   target_points[i].y(),
                                   target_points[i].z());
  }
  const double inv_count = 1.0 / static_cast<double>(source_points.size());
  source_mean *= inv_count;
  target_mean *= inv_count;

  double a = 0.0;
  double b = 0.0;
  for (size_t i = 0u; i < source_points.size(); ++i) {
    const double sx = source_points[i].x() - source_mean.x();
    const double sy = source_points[i].y() - source_mean.y();
    const double tx = target_points[i].x() - target_mean.x();
    const double ty = target_points[i].y() - target_mean.y();
    a += sx * tx + sy * ty;
    b += sx * ty - sy * tx;
  }
  if (!std::isfinite(a) || !std::isfinite(b) ||
      std::hypot(a, b) < 1e-9) {
    return false;
  }

  const double yaw = std::atan2(b, a);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const Eigen::Vector3d rotated_source_mean(c * source_mean.x() -
                                                s * source_mean.y(),
                                            s * source_mean.x() +
                                                c * source_mean.y(),
                                            source_mean.z());
  const Eigen::Vector3d translation = target_mean - rotated_source_mean;

  double squared_error_sum = 0.0;
  double error_sum = 0.0;
  double max_error = 0.0;
  for (size_t i = 0u; i < source_points.size(); ++i) {
    const Eigen::Vector3d source(source_points[i].x(),
                                 source_points[i].y(),
                                 source_points[i].z());
    const Eigen::Vector3d target(target_points[i].x(),
                                 target_points[i].y(),
                                 target_points[i].z());
    const Eigen::Vector3d aligned(c * source.x() - s * source.y(),
                                  s * source.x() + c * source.y(),
                                  source.z());
    const double error = (aligned + translation - target).norm();
    squared_error_sum += error * error;
    error_sum += error;
    max_error = std::max(max_error, error);
  }

  estimate->yaw_rad = yaw;
  estimate->translation =
      gtsam::Point3(translation.x(), translation.y(), translation.z());
  estimate->rmse_m =
      std::sqrt(squared_error_sum / static_cast<double>(source_points.size()));
  estimate->mean_m = error_sum / static_cast<double>(source_points.size());
  estimate->max_m = max_error;
  return true;
}

gtsam::Pose3 transformPoseSe2(const gtsam::Pose3& pose,
                              const Se2AlignmentEstimate& alignment,
                              const gtsam::Point3& origin) {
  const gtsam::Rot3 yaw_rotation = gtsam::Rot3::Rz(alignment.yaw_rad);
  const Eigen::Vector3d translation(alignment.translation.x(),
                                    alignment.translation.y(),
                                    alignment.translation.z());
  const Eigen::Vector3d origin_v(origin.x(), origin.y(), origin.z());
  const Eigen::Vector3d p = yaw_rotation.matrix() * pose.translation() +
                            translation - origin_v;
  return gtsam::Pose3(yaw_rotation.compose(pose.rotation()),
                      gtsam::Point3(p.x(), p.y(), p.z()));
}

gtsam::Pose3 transformPoseSe3ToLocal(const gtsam::Pose3& pose,
                                     const AlignmentEstimate& alignment,
                                     const gtsam::Point3& origin) {
  const gtsam::Pose3 aligned_pose = alignment.target_T_source * pose;
  const Eigen::Vector3d origin_v(origin.x(), origin.y(), origin.z());
  const Eigen::Vector3d p = aligned_pose.translation() - origin_v;
  return gtsam::Pose3(aligned_pose.rotation(),
                      gtsam::Point3(p.x(), p.y(), p.z()));
}

gtsam::Point3 transformPointSe3ToLocal(const gtsam::Point3& point,
                                       const AlignmentEstimate& alignment,
                                       const gtsam::Point3& origin) {
  const gtsam::Point3 aligned_point =
      alignment.target_T_source.transformFrom(point);
  return gtsam::Point3(aligned_point.x() - origin.x(),
                       aligned_point.y() - origin.y(),
                       aligned_point.z() - origin.z());
}

void translateTrajectory(std::vector<gtsam::Pose3>* poses,
                         const Eigen::Vector3d& translation) {
  CHECK_NOTNULL(poses);
  for (gtsam::Pose3& pose : *poses) {
    const Eigen::Vector3d p = pose.translation() + translation;
    pose = gtsam::Pose3(pose.rotation(), gtsam::Point3(p.x(), p.y(), p.z()));
  }
}

void anchorTrajectoryStartToOrigin(std::vector<gtsam::Pose3>* poses) {
  CHECK_NOTNULL(poses);
  if (poses->empty()) {
    return;
  }
  translateTrajectory(poses, -poses->front().translation());
}

class RerunTopicVisualizer {
 public:
  void publishRawCbsDashboardBlueprint() {
    if (!raw_cbs_dashboard_blueprint_enable_ || !blueprint_stream_) {
      return;
    }

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_scene",
                     "3D",
                     "Scene",
                     "/",
                     {
                         "+ /aligned/**",
                         "+ /kimera/**",
                         "+ /glim/**",
                         "+ /ground_truth/**",
                         "- /aligned/metadata/**",
                         "- /kimera/cbs/**",
                         "- /glim/cbs/**",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_video",
                     "2D",
                     "Video",
                     "/video",
                     {
                         "+ /video/**",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_g2k_counts",
                     "TimeSeries",
                     "G->K Counts",
                     "/cbs/g2k",
                     {
                         "+ /cbs/g2k/beliefs_per_message",
                         "+ /cbs/g2k/beliefs_total",
                         "+ /cbs/g2k/messages_total",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_g2k_quality",
                     "TimeSeries",
                     "G->K Quality",
                     "/cbs/g2k",
                     {
                         "+ /cbs/g2k/message_interval_sec",
                         "+ /cbs/g2k/edge_duration_mean_sec",
                         "+ /cbs/g2k/edge_duration_min_sec",
                         "+ /cbs/g2k/edge_duration_max_sec",
                         "+ /cbs/g2k/covariance_trace_mean",
                         "+ /cbs/g2k/covariance_frobenius_mean",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_k2g_counts",
                     "TimeSeries",
                     "K->G Counts",
                     "/cbs/k2g",
                     {
                         "+ /cbs/k2g/beliefs_per_message",
                         "+ /cbs/k2g/beliefs_total",
                         "+ /cbs/k2g/messages_total",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_k2g_quality",
                     "TimeSeries",
                     "K->G Quality",
                     "/cbs/k2g",
                     {
                         "+ /cbs/k2g/message_interval_sec",
                         "+ /cbs/k2g/edge_duration_mean_sec",
                         "+ /cbs/k2g/edge_duration_min_sec",
                         "+ /cbs/k2g/edge_duration_max_sec",
                         "+ /cbs/k2g/covariance_trace_mean",
                         "+ /cbs/k2g/covariance_frobenius_mean",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_glim",
                     "TimeSeries",
                     "GLIM CBS",
                     "/glim/cbs",
                     {
                         "+ /glim/cbs/**",
                         "- /glim/cbs/timing/**",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_kimera",
                     "TimeSeries",
                     "Kimera CBS",
                     "/kimera/cbs",
                     {
                         "+ /kimera/cbs/**",
                         "- /kimera/cbs/timing/**",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_covariance",
                     "TimeSeries",
                     "Relative Belief Covariance",
                     "/metrics/relative_belief_covariance",
                     {
                         "+ /metrics/relative_belief_covariance/**",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_accumulated_covariance",
                     "TimeSeries",
                     "Accumulated Covariance",
                     "/metrics/accumulated_covariance",
                     {
                         "+ /metrics/accumulated_covariance/**",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_lag_alignment",
                     "TimeSeries",
                     "Lag and Alignment",
                     "/visualization",
                     {
                         "+ /visualization/**",
                         "+ /ground_truth/alignment/**",
                         "+ /aligned/metadata/**",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_glim_runtime",
                     "TimeSeries",
                     "GLIM Runtime",
                     "/glim",
                     {
                         "+ /glim/timing/**",
                         "+ /glim/cbs/timing/**",
                         "- /__properties/**",
                     });

    logBlueprintView(blueprint_stream_.get(),
                     "raw_cbs_kimera_runtime",
                     "TimeSeries",
                     "Kimera Runtime",
                     "/kimera",
                     {
                         "+ /kimera/timing/**",
                         "+ /kimera/keyframe_id",
                         "+ /kimera/factor_graph/**",
                         "+ /kimera/cbs/timing/**",
                         "+ /kimera/cbs/marginalization_graph/**",
                         "- /__properties/**",
                     });

    const std::string scene_view = blueprintViewPath("raw_cbs_scene");
    const std::string video_view = blueprintViewPath("raw_cbs_video");
    const std::string g2k_traffic_view =
        blueprintViewPath("raw_cbs_g2k_traffic");
    const std::string k2g_traffic_view =
        blueprintViewPath("raw_cbs_k2g_traffic");
    const std::string glim_view = blueprintViewPath("raw_cbs_glim");
    const std::string kimera_view = blueprintViewPath("raw_cbs_kimera");
    const std::string covariance_view = blueprintViewPath("raw_cbs_covariance");
    const std::string accumulated_covariance_view =
        blueprintViewPath("raw_cbs_accumulated_covariance");
    const std::string lag_alignment_view =
        blueprintViewPath("raw_cbs_lag_alignment");
    const std::string glim_runtime_view =
        blueprintViewPath("raw_cbs_glim_runtime");
    const std::string kimera_runtime_view =
        blueprintViewPath("raw_cbs_kimera_runtime");

    logBlueprintContainer(blueprint_stream_.get(),
                          "raw_cbs_media",
                          rerun::blueprint::components::ContainerKind::Horizontal,
                          "Media",
                          {scene_view, video_view},
                          {1.7f, 1.0f});

    logBlueprintContainer(
        blueprint_stream_.get(),
        "raw_cbs_traffic_row",
        rerun::blueprint::components::ContainerKind::Horizontal,
        "Directional Traffic",
        {g2k_traffic_view, k2g_traffic_view},
        {1.0f, 1.0f});

    logBlueprintContainer(
        blueprint_stream_.get(),
        "raw_cbs_local_row",
        rerun::blueprint::components::ContainerKind::Horizontal,
        "Local CBS",
        {glim_view, kimera_view},
        {1.0f, 1.0f});

    logBlueprintContainer(
        blueprint_stream_.get(),
        "raw_cbs_runtime_row",
        rerun::blueprint::components::ContainerKind::Horizontal,
        "Runtime",
        {glim_runtime_view, kimera_runtime_view},
        {1.0f, 1.0f});

    logBlueprintContainer(
        blueprint_stream_.get(),
        "raw_cbs_covariance_row",
        rerun::blueprint::components::ContainerKind::Horizontal,
        "Covariance",
        {covariance_view, accumulated_covariance_view, lag_alignment_view},
        {1.0f, 1.0f, 1.0f});

    const std::string media_container = blueprintContainerPath("raw_cbs_media");
    const std::string traffic_container =
        blueprintContainerPath("raw_cbs_traffic_row");
    const std::string local_container =
        blueprintContainerPath("raw_cbs_local_row");
    const std::string runtime_container =
        blueprintContainerPath("raw_cbs_runtime_row");
    const std::string covariance_container =
        blueprintContainerPath("raw_cbs_covariance_row");

    logBlueprintContainer(blueprint_stream_.get(),
                          "raw_cbs_root",
                          rerun::blueprint::components::ContainerKind::Vertical,
                          "Raw CBS Dashboard",
                          {media_container,
                           traffic_container,
                           local_container,
                           runtime_container,
                           covariance_container},
                          {},
                          {1.6f, 1.0f, 1.0f, 1.0f, 1.0f});

    blueprint_stream_->set_time_sequence("blueprint", 0);

    blueprint_stream_->log(
        "viewport",
        rerun::blueprint::archetypes::ViewportBlueprint()
            .with_root_container(rerun::blueprint::components::RootContainer(
                makeDeterministicUuid("container:raw_cbs_root")))
            .with_auto_layout(rerun::blueprint::components::AutoLayout(false))
            .with_auto_views(rerun::blueprint::components::AutoViews(false)));

    blueprint_stream_->log(
        "time_panel",
        rerun::blueprint::archetypes::TimePanelBlueprint()
            .with_play_state(
                rerun::blueprint::components::PlayState::Following)
            .with_loop_mode(rerun::blueprint::components::LoopMode::Off));

    (void)blueprint_stream_->flush_blocking(2.0f);
  }

  RerunTopicVisualizer()
      : private_nh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        kimera_("kimera"),
        liorf_("liorf"),
        liorf_local_map_("liorf_local_map"),
        liorf_current_scan_("liorf_current_scan"),
        image_("image"),
        kimera_landmarks_("kimera_landmarks"),
        cbs_g2k_("glim_to_kimera", "cbs/g2k", "glim_sent"),
        cbs_k2g_("kimera_to_glim", "cbs/k2g", "kimera_sent"),
        visualizer_(nullptr) {
    private_nh_.param<std::string>(
        "kimera_odom_topic", kimera_odom_topic_, "/kimera_vio_ros/odometry");
    private_nh_.param<std::string>(
        "liorf_odom_topic", liorf_odom_topic_, "/liorf/mapping/odometry");
    private_nh_.param<std::string>(
        "secondary_entity_prefix", secondary_entity_prefix_, "liorf");
    private_nh_.param<std::string>(
        "secondary_frame_name", secondary_frame_name_, "lidar_link");
    secondary_entity_prefix_ =
        normalizeEntityPrefix(secondary_entity_prefix_, "liorf");
    if (secondary_frame_name_.empty()) {
      secondary_frame_name_ = "base_link";
    }
    private_nh_.param<double>("publish_rate_hz", publish_rate_hz_, 5.0);
    private_nh_.param<bool>("uncertainty_enable", uncertainty_enable_, true);
    private_nh_.param<bool>("raw_covariance_enable",
                            raw_covariance_enable_,
                            true);
    private_nh_.param<double>(
        "kimera_uncertainty_scale", kimera_uncertainty_scale_, 1.25);
    private_nh_.param<double>("secondary_uncertainty_scale",
                              secondary_uncertainty_scale_,
                              1.25);
    private_nh_.param<double>("aligned_kimera_uncertainty_scale",
                              aligned_kimera_uncertainty_scale_,
                              kimera_uncertainty_scale_);
    if (!std::isfinite(kimera_uncertainty_scale_) ||
        kimera_uncertainty_scale_ <= 0.0) {
      kimera_uncertainty_scale_ = 1.25;
    }
    if (!std::isfinite(secondary_uncertainty_scale_) ||
        secondary_uncertainty_scale_ <= 0.0) {
      secondary_uncertainty_scale_ = 1.25;
    }
    if (!std::isfinite(aligned_kimera_uncertainty_scale_) ||
        aligned_kimera_uncertainty_scale_ <= 0.0) {
      aligned_kimera_uncertainty_scale_ = kimera_uncertainty_scale_;
    }
    private_nh_.param<bool>("lag_scalars_enable", lag_scalars_enable_, true);
    private_nh_.param<bool>(
        "world_alignment_enable", world_alignment_enable_, true);
    private_nh_.param<int>("max_trajectory_len", max_trajectory_len_, 2000);
    private_nh_.param<bool>(
        "liorf_point_clouds_enable", liorf_point_clouds_enable_, true);
    private_nh_.param<bool>("liorf_local_map_enable",
                            liorf_local_map_enable_,
                            true);
    private_nh_.param<bool>("liorf_current_scan_enable",
                            liorf_current_scan_enable_,
                            true);
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
    private_nh_.param<bool>("image_stream_enable",
                            image_stream_enable_,
                            false);
    private_nh_.param<std::string>("image_topic", image_topic_, "");
    private_nh_.param<std::string>("image_entity_path",
                                   image_entity_path_,
                                   "video/rgb/image");
    private_nh_.param<double>("image_max_hz", image_max_hz_, 10.0);
    private_nh_.param<int>("image_queue_size", image_queue_size_, 2);
    private_nh_.param<bool>("secondary_point_cloud_common_alignment_enable",
                            secondary_point_cloud_common_alignment_enable_,
                            true);
    private_nh_.param<bool>("secondary_point_cloud_publish_raw_enable",
                            secondary_point_cloud_publish_raw_enable_,
                            false);
    private_nh_.param<double>("secondary_point_cloud_tf_lookup_timeout_sec",
                              secondary_point_cloud_tf_lookup_timeout_sec_,
                              0.05);
    private_nh_.param<bool>(
        "kimera_landmarks_enable", kimera_landmarks_enable_, true);
    private_nh_.param<std::string>("kimera_landmarks_topic",
                                   kimera_landmarks_topic_,
                                   "/kimera_vio_ros/landmarks");
    private_nh_.param<int>(
        "kimera_landmarks_max_points", kimera_landmarks_max_points_, 3000);
    private_nh_.param<bool>(
        "cbs_metrics_enable", cbs_metrics_enable_, false);
    private_nh_.param<bool>("accumulated_covariance_enable",
                            accumulated_covariance_enable_,
                            true);
    private_nh_.param<bool>(
        "publish_raw_kimera_enable", publish_raw_kimera_enable_, true);
    private_nh_.param<bool>(
        "publish_secondary_enable", publish_secondary_enable_, true);
    private_nh_.param<bool>("publish_aligned_kimera_enable",
                            publish_aligned_kimera_enable_,
                            true);
    private_nh_.param<bool>("publish_legacy_online_alignment_enable",
                            publish_legacy_online_alignment_enable_,
                            false);
    private_nh_.param<std::string>("cbs_g2k_odom_belief_topic",
                                   cbs_g2k_odom_belief_topic_,
                                   "/kimera/cbs/odom_belief_in");
    private_nh_.param<std::string>("cbs_k2g_odom_belief_topic",
                                   cbs_k2g_odom_belief_topic_,
                                   "/kimera/cbs/odom_belief_out");
    private_nh_.param<bool>(
        "ground_truth_enable", ground_truth_enable_, false);
    private_nh_.param<bool>("publish_common_aligned_overlay_enable",
                            publish_common_aligned_overlay_enable_,
                            true);
    private_nh_.param<bool>("anchor_common_aligned_start_enable",
                            anchor_common_aligned_start_enable_,
                            false);
    private_nh_.param<bool>("common_aligned_provisional_start_enable",
                            common_aligned_provisional_start_enable_,
                            false);
    private_nh_.param<bool>("freeze_common_aligned_overlay_alignment_enable",
                            freeze_common_aligned_overlay_alignment_enable_,
                            false);
    private_nh_.param<std::string>("ground_truth_path", ground_truth_path_, "");
    private_nh_.param<double>("ground_truth_max_timestamp_diff_sec",
                              ground_truth_max_timestamp_diff_sec_,
                              0.75);
    private_nh_.param<int>("ground_truth_alignment_min_pairs",
                           ground_truth_alignment_min_pairs_,
                           8);
    private_nh_.param<double>("ground_truth_alignment_min_path_length_m",
                              ground_truth_alignment_min_path_length_m_,
                              3.0);
    private_nh_.param<double>("ground_truth_window_duration_sec",
                              ground_truth_window_duration_sec_,
                              60.0);
    private_nh_.param<int>("odometry_queue_size", odometry_queue_size_, 512);
    private_nh_.param<int>("point_cloud_queue_size", point_cloud_queue_size_, 2);
    private_nh_.param<int>("cbs_metrics_queue_size", cbs_metrics_queue_size_, 200);
    private_nh_.param<std::string>("rerun_recording_id", recording_id_, "");
    private_nh_.param<std::string>("rerun_host", rerun_host_, "auto");
    private_nh_.param<bool>("raw_cbs_dashboard_blueprint_enable",
                            raw_cbs_dashboard_blueprint_enable_,
                            false);

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      LOG(WARNING) << "Invalid publish_rate_hz=" << publish_rate_hz_
                   << "; falling back to 5 Hz.";
      publish_rate_hz_ = 5.0;
    }
    ground_truth_alignment_min_pairs_ =
        std::max(2, ground_truth_alignment_min_pairs_);
    max_trajectory_len_ = std::max(2, max_trajectory_len_);
    odometry_queue_size_ = std::max(1, odometry_queue_size_);
    point_cloud_queue_size_ = std::max(1, point_cloud_queue_size_);
    image_queue_size_ = std::max(1, image_queue_size_);
    cbs_metrics_queue_size_ = std::max(1, cbs_metrics_queue_size_);

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
    if (raw_cbs_dashboard_blueprint_enable_) {
      blueprint_stream_ = std::make_unique<rerun::RecordingStream>(
          "cbsms", recording_id_, rerun::StoreKind::Blueprint);
      blueprint_stream_->connect_grpc(rerun_host_).exit_on_failure();
      publishRawCbsDashboardBlueprint();
    }
    if (ground_truth_enable_) {
      ground_truth_loaded_ =
          loadGroundTruthTrajectory(ground_truth_path_, &ground_truth_);
    }

    kimera_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        kimera_odom_topic_,
        odometry_queue_size_,
        &RerunTopicVisualizer::kimeraCallback,
        this,
        ros::TransportHints().tcpNoDelay());
    liorf_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        liorf_odom_topic_,
        odometry_queue_size_,
        &RerunTopicVisualizer::liorfCallback,
        this,
        ros::TransportHints().tcpNoDelay());
    if (liorf_point_clouds_enable_) {
      if (liorf_local_map_enable_) {
        liorf_local_map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            liorf_local_map_topic_,
            point_cloud_queue_size_,
            &RerunTopicVisualizer::liorfLocalMapCallback,
            this,
            ros::TransportHints().tcpNoDelay());
      }
      if (liorf_current_scan_enable_) {
        liorf_current_scan_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            liorf_current_scan_topic_,
            point_cloud_queue_size_,
            &RerunTopicVisualizer::liorfCurrentScanCallback,
            this,
            ros::TransportHints().tcpNoDelay());
      }
    }
    if (image_stream_enable_ && !image_topic_.empty()) {
      image_sub_ = nh_.subscribe<sensor_msgs::CompressedImage>(
          image_topic_,
          image_queue_size_,
          &RerunTopicVisualizer::imageCallback,
          this,
          ros::TransportHints().tcpNoDelay());
    }
    if (kimera_landmarks_enable_) {
      kimera_landmarks_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          kimera_landmarks_topic_,
          point_cloud_queue_size_,
          &RerunTopicVisualizer::kimeraLandmarksCallback,
          this,
          ros::TransportHints().tcpNoDelay());
    }
    if (cbs_metrics_enable_) {
      cbs_g2k_sub_ = nh_.subscribe<liorf::pose_odom_belief_array>(
          cbs_g2k_odom_belief_topic_,
          cbs_metrics_queue_size_,
          &RerunTopicVisualizer::cbsG2kCallback,
          this,
          ros::TransportHints().tcpNoDelay());
      cbs_k2g_sub_ = nh_.subscribe<liorf::pose_odom_belief_array>(
          cbs_k2g_odom_belief_topic_,
          cbs_metrics_queue_size_,
          &RerunTopicVisualizer::cbsK2gCallback,
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
              << "', secondary_entity_prefix='" << secondary_entity_prefix_
              << "', secondary_frame_name='" << secondary_frame_name_
              << "', publish_rate_hz=" << publish_rate_hz_
              << ", odometry_queue_size=" << odometry_queue_size_
              << ", point_cloud_queue_size=" << point_cloud_queue_size_
              << ", cbs_metrics_queue_size=" << cbs_metrics_queue_size_
              << ", liorf_point_clouds_enable="
              << (liorf_point_clouds_enable_ ? "true" : "false")
              << ", image_stream_enable="
              << (image_stream_enable_ ? "true" : "false")
              << ", image_topic='" << image_topic_
              << "', image_entity_path='" << image_entity_path_
              << "', image_max_hz=" << image_max_hz_
              << ", kimera_landmarks_enable="
              << (kimera_landmarks_enable_ ? "true" : "false")
              << ", cbs_metrics_enable="
              << (cbs_metrics_enable_ ? "true" : "false")
              << ", accumulated_covariance_enable="
              << (accumulated_covariance_enable_ ? "true" : "false")
              << ", cbs_g2k_odom_belief_topic='"
              << cbs_g2k_odom_belief_topic_
              << "', cbs_k2g_odom_belief_topic='"
              << cbs_k2g_odom_belief_topic_
              << "', publish_raw_kimera_enable="
              << (publish_raw_kimera_enable_ ? "true" : "false")
              << ", publish_secondary_enable="
              << (publish_secondary_enable_ ? "true" : "false")
              << ", publish_aligned_kimera_enable="
              << (publish_aligned_kimera_enable_ ? "true" : "false")
              << ", publish_legacy_online_alignment_enable="
              << (publish_legacy_online_alignment_enable_ ? "true" : "false")
              << ", ground_truth_enable="
              << (ground_truth_enable_ ? "true" : "false")
              << ", ground_truth_loaded="
              << (ground_truth_loaded_ ? "true" : "false")
              << ", publish_common_aligned_overlay_enable="
              << (publish_common_aligned_overlay_enable_ ? "true" : "false")
              << ", anchor_common_aligned_start_enable="
              << (anchor_common_aligned_start_enable_ ? "true" : "false")
              << ", common_aligned_provisional_start_enable="
              << (common_aligned_provisional_start_enable_ ? "true"
                                                           : "false")
              << ", freeze_common_aligned_overlay_alignment_enable="
              << (freeze_common_aligned_overlay_alignment_enable_ ? "true"
                                                                  : "false")
              << ", ground_truth_alignment_min_pairs="
              << ground_truth_alignment_min_pairs_
              << ", ground_truth_alignment_min_path_length_m="
              << ground_truth_alignment_min_path_length_m_
              << ", ground_truth_window_duration_sec="
              << ground_truth_window_duration_sec_ << ".";
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

  void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    updateImage(msg, &image_);
  }

  void kimeraLandmarksCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    updateCloud(msg, &kimera_landmarks_);
  }

  void cbsG2kCallback(const liorf::pose_odom_belief_arrayConstPtr& msg) {
    updateBeliefTraffic(msg, &cbs_g2k_);
  }

  void cbsK2gCallback(const liorf::pose_odom_belief_arrayConstPtr& msg) {
    updateBeliefTraffic(msg, &cbs_k2g_);
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

    gtsam::Pose3 pose;
    if (odometryToPose(*msg, &pose) &&
        (!std::isfinite(source->last_timed_trajectory_stamp_sec) ||
         stamp_sec > source->last_timed_trajectory_stamp_sec)) {
      source->timed_trajectory.push_back({stamp_sec, pose});
      source->last_timed_trajectory_stamp_sec = stamp_sec;
      if (source->timed_trajectory.size() >
          static_cast<size_t>(max_trajectory_len_)) {
        const size_t extra = source->timed_trajectory.size() -
                             static_cast<size_t>(max_trajectory_len_);
        source->timed_trajectory.erase(
            source->timed_trajectory.begin(),
            source->timed_trajectory.begin() + extra);
      }
    }
  }

  void updateCloud(const sensor_msgs::PointCloud2::ConstPtr& msg,
                   CloudState* cloud) {
    CHECK_NOTNULL(msg);
    CHECK_NOTNULL(cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    cloud->latest = *msg;
    cloud->has_latest = true;
  }

  void updateImage(const sensor_msgs::CompressedImage::ConstPtr& msg,
                   ImageState* image) {
    CHECK_NOTNULL(msg);
    CHECK_NOTNULL(image);
    std::lock_guard<std::mutex> lock(mutex_);
    image->latest = *msg;
    image->has_latest = true;
  }

  void updateBeliefTraffic(const liorf::pose_odom_belief_arrayConstPtr& msg,
                           BeliefTrafficState* state) {
    if (!msg || !state) {
      return;
    }

    BeliefTrafficStats stats;
    std::vector<AccumulatedBeliefCovarianceSnapshot> covariance_snapshots;
    std::string entity;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats = summarizeBeliefTraffic(*msg, state);
      if (accumulated_covariance_enable_) {
        appendAccumulatedBeliefCovarianceSnapshots(
            *msg, &state->accumulated_covariance, &covariance_snapshots);
      }
      entity = state->entity;
    }
    publishBeliefTraffic(entity, stats);
    for (const AccumulatedBeliefCovarianceSnapshot& snapshot :
         covariance_snapshots) {
      publishAccumulatedBeliefCovariance(snapshot);
    }
  }

  void publishBeliefTraffic(const std::string& entity,
                            const BeliefTrafficStats& stats) {
    const uint64_t stamp_nsec = stampSecToNSec(stats.message_stamp_sec);
    if (stamp_nsec > 0ull) {
      visualizer_->setTimeNSec(stamp_nsec);
    }
    visualizer_->drawScalar(entity + "/beliefs_per_message",
                            static_cast<double>(stats.beliefs_per_message));
    visualizer_->drawScalar(entity + "/beliefs_total",
                            static_cast<double>(stats.beliefs_total));
    visualizer_->drawScalar(entity + "/messages_total",
                            static_cast<double>(stats.messages_total));

    drawFiniteScalar(entity + "/message_stamp_sec", stats.message_stamp_sec);
    drawFiniteScalar(entity + "/message_interval_sec",
                     stats.message_interval_sec);
    drawFiniteScalar(entity + "/edge_duration_mean_sec",
                     stats.edge_duration_mean_sec);
    drawFiniteScalar(entity + "/edge_duration_min_sec",
                     stats.edge_duration_min_sec);
    drawFiniteScalar(entity + "/edge_duration_max_sec",
                     stats.edge_duration_max_sec);
    drawFiniteScalar(entity + "/covariance_trace_mean",
                     stats.covariance_trace_mean);
    drawFiniteScalar(entity + "/covariance_frobenius_mean",
                     stats.covariance_frobenius_mean);
    drawFiniteScalar(entity + "/latest_source_agent",
                     stats.latest_source_agent);
    drawFiniteScalar(entity + "/latest_from_stamp_sec",
                     stats.latest_from_stamp_sec);
    drawFiniteScalar(entity + "/latest_to_stamp_sec",
                     stats.latest_to_stamp_sec);
  }

  void drawFiniteScalar(const std::string& path, const double value) {
    if (std::isfinite(value)) {
      visualizer_->drawScalar(path, value);
    }
  }

  void publishTranslationCovarianceMetrics(
      const std::string& base_path,
      const Eigen::Matrix3d& translation_covariance,
      const std::string& metric_prefix) {
    if (!isUsableCovariance(translation_covariance)) {
      return;
    }

    const double trace_m2 = translation_covariance.trace();
    const double frobenius_m2 = translation_covariance.norm();
    const Eigen::Matrix3d symmetric_covariance =
        0.5 * (translation_covariance + translation_covariance.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(
        symmetric_covariance);

    drawFiniteScalar(base_path + "/" + metric_prefix + "translation_trace_m2",
                     trace_m2);
    drawFiniteScalar(base_path + "/" + metric_prefix + "translation_trace_cm2",
                     trace_m2 * 1.0e4);
    drawFiniteScalar(base_path + "/" + metric_prefix +
                         "translation_frobenius_m2",
                     frobenius_m2);
    drawFiniteScalar(base_path + "/" + metric_prefix +
                         "translation_frobenius_cm2",
                     frobenius_m2 * 1.0e4);
    if (trace_m2 > 0.0) {
      drawFiniteScalar(base_path + "/" + metric_prefix +
                           "translation_trace_log10_m2",
                       std::log10(trace_m2));
    }

    const double sigma_x_cm =
        std::sqrt(std::max(0.0, translation_covariance(0, 0))) * 100.0;
    const double sigma_y_cm =
        std::sqrt(std::max(0.0, translation_covariance(1, 1))) * 100.0;
    const double sigma_z_cm =
        std::sqrt(std::max(0.0, translation_covariance(2, 2))) * 100.0;
    drawFiniteScalar(base_path + "/" + metric_prefix + "sigma_x_cm",
                     sigma_x_cm);
    drawFiniteScalar(base_path + "/" + metric_prefix + "sigma_y_cm",
                     sigma_y_cm);
    drawFiniteScalar(base_path + "/" + metric_prefix + "sigma_z_cm",
                     sigma_z_cm);
    drawFiniteScalar(base_path + "/" + metric_prefix + "sigma_mean_cm",
                     (sigma_x_cm + sigma_y_cm + sigma_z_cm) / 3.0);

    if (eig.info() == Eigen::Success) {
      const auto eigenvalues = eig.eigenvalues();
      drawFiniteScalar(base_path + "/" + metric_prefix + "sigma_min_cm",
                       std::sqrt(std::max(0.0, eigenvalues.minCoeff())) *
                           100.0);
      drawFiniteScalar(base_path + "/" + metric_prefix + "sigma_max_cm",
                       std::sqrt(std::max(0.0, eigenvalues.maxCoeff())) *
                           100.0);
    }
  }

  void publishAccumulatedBeliefCovariance(
      const AccumulatedBeliefCovarianceSnapshot& snapshot) {
    const uint64_t stamp_nsec = stampSecToNSec(snapshot.stamp_sec);
    if (stamp_nsec > 0ull) {
      visualizer_->setTimeNSec(stamp_nsec);
    }

    const std::string accumulated_base =
        "metrics/accumulated_covariance/" + snapshot.source_name;
    visualizer_->drawScalar(accumulated_base + "/initialized",
                            snapshot.initialized ? 1.0 : 0.0);
    visualizer_->drawScalar(accumulated_base + "/edge_count",
                            static_cast<double>(snapshot.edge_count));
    visualizer_->drawScalar(
        accumulated_base + "/skipped_duplicate_total",
        static_cast<double>(snapshot.skipped_duplicate_total));
    visualizer_->drawScalar(
        accumulated_base + "/skipped_noncontiguous_total",
        static_cast<double>(snapshot.skipped_noncontiguous_total));
    visualizer_->drawScalar(
        accumulated_base + "/skipped_invalid_total",
        static_cast<double>(snapshot.skipped_invalid_total));
    drawFiniteScalar(accumulated_base + "/chain_duration_sec",
                     snapshot.chain_duration_sec);
    visualizer_->drawScalar(accumulated_base + "/latest_to_index",
                            static_cast<double>(snapshot.latest_to_index));
    if (snapshot.has_accumulated_covariance) {
      publishTranslationCovarianceMetrics(
          accumulated_base, snapshot.accumulated_translation_covariance, "");
    }

    if (!snapshot.has_relative_covariance) {
      return;
    }

    const std::string relative_base =
        "metrics/relative_belief_covariance/" + snapshot.source_name;
    visualizer_->drawScalar(relative_base + "/latest_from_index",
                            static_cast<double>(snapshot.latest_from_index));
    visualizer_->drawScalar(relative_base + "/latest_to_index",
                            static_cast<double>(snapshot.latest_to_index));
    drawFiniteScalar(relative_base + "/latest_edge_duration_sec",
                     snapshot.edge_duration_sec);
    const gtsam::Matrix3 rotation_covariance =
        snapshot.relative_pose_covariance.block<3, 3>(0, 0);
    const double rotation_trace_rad2 = rotation_covariance.trace();
    constexpr double kRadToDeg = 57.2957795130823208768;
    drawFiniteScalar(relative_base + "/latest_rotation_trace_rad2",
                     rotation_trace_rad2);
    drawFiniteScalar(relative_base + "/latest_rotation_trace_deg2",
                     rotation_trace_rad2 * kRadToDeg * kRadToDeg);
    drawFiniteScalar(relative_base + "/latest_rotation_variance_x_rad2",
                     snapshot.relative_pose_covariance(0, 0));
    drawFiniteScalar(relative_base + "/latest_rotation_variance_y_rad2",
                     snapshot.relative_pose_covariance(1, 1));
    drawFiniteScalar(relative_base + "/latest_rotation_variance_z_rad2",
                     snapshot.relative_pose_covariance(2, 2));
    drawFiniteScalar(relative_base + "/latest_translation_variance_x_m2",
                     snapshot.relative_pose_covariance(3, 3));
    drawFiniteScalar(relative_base + "/latest_translation_variance_y_m2",
                     snapshot.relative_pose_covariance(4, 4));
    drawFiniteScalar(relative_base + "/latest_translation_variance_z_m2",
                     snapshot.relative_pose_covariance(5, 5));
    publishTranslationCovarianceMetrics(
        relative_base, snapshot.relative_translation_covariance, "latest_");
  }

  void publishPosteriorCovarianceMetrics(
      const std::string& source_name,
      const Eigen::Matrix3d& translation_covariance) {
    if (!translation_covariance.allFinite()) {
      return;
    }

    const double trace_m2 = translation_covariance.trace();
    const double frobenius_m2 = translation_covariance.norm();
    const Eigen::Matrix3d symmetric_covariance =
        0.5 * (translation_covariance + translation_covariance.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(
        symmetric_covariance);

    const std::string base_path =
        "metrics/posterior_covariance/" + source_name;
    drawFiniteScalar(base_path + "/translation_trace_m2", trace_m2);
    drawFiniteScalar(base_path + "/translation_trace_cm2", trace_m2 * 1.0e4);
    drawFiniteScalar(base_path + "/translation_frobenius_m2", frobenius_m2);
    drawFiniteScalar(base_path + "/translation_frobenius_cm2",
                     frobenius_m2 * 1.0e4);
    if (trace_m2 > 0.0) {
      drawFiniteScalar(base_path + "/translation_trace_log10_m2",
                       std::log10(trace_m2));
    }

    const double sigma_x_cm =
        std::sqrt(std::max(0.0, translation_covariance(0, 0))) * 100.0;
    const double sigma_y_cm =
        std::sqrt(std::max(0.0, translation_covariance(1, 1))) * 100.0;
    const double sigma_z_cm =
        std::sqrt(std::max(0.0, translation_covariance(2, 2))) * 100.0;
    double sigma_max_cm = std::numeric_limits<double>::quiet_NaN();
    drawFiniteScalar(base_path + "/sigma_x_cm", sigma_x_cm);
    drawFiniteScalar(base_path + "/sigma_y_cm", sigma_y_cm);
    drawFiniteScalar(base_path + "/sigma_z_cm", sigma_z_cm);
    drawFiniteScalar(base_path + "/sigma_mean_cm",
                     (sigma_x_cm + sigma_y_cm + sigma_z_cm) / 3.0);

    if (eig.info() == Eigen::Success) {
      const auto eigenvalues = eig.eigenvalues();
      drawFiniteScalar(base_path + "/sigma_min_cm",
                       std::sqrt(std::max(0.0, eigenvalues.minCoeff())) *
                           100.0);
      sigma_max_cm =
          std::sqrt(std::max(0.0, eigenvalues.maxCoeff())) * 100.0;
      drawFiniteScalar(base_path + "/sigma_max_cm", sigma_max_cm);
    }

    if (trace_m2 > 1.0e-8 &&
        covariance_trace_reference_m2_.find(source_name) ==
            covariance_trace_reference_m2_.end()) {
      covariance_trace_reference_m2_[source_name] = trace_m2;
    }
    const auto reference_iter =
        covariance_trace_reference_m2_.find(source_name);
    if (reference_iter != covariance_trace_reference_m2_.end() &&
        reference_iter->second > 0.0) {
      const double reference_m2 = reference_iter->second;
      drawFiniteScalar(base_path + "/translation_trace_reference_m2",
                       reference_m2);
      drawFiniteScalar(base_path + "/translation_trace_delta_m2",
                       trace_m2 - reference_m2);
      drawFiniteScalar(base_path + "/translation_trace_delta_cm2",
                       (trace_m2 - reference_m2) * 1.0e4);
      drawFiniteScalar(base_path + "/translation_trace_delta_percent",
                       100.0 * (trace_m2 / reference_m2 - 1.0));
    }

    if (std::isfinite(sigma_max_cm) && sigma_max_cm > 1.0e-5 &&
        covariance_sigma_max_reference_cm_.find(source_name) ==
            covariance_sigma_max_reference_cm_.end()) {
      covariance_sigma_max_reference_cm_[source_name] = sigma_max_cm;
    }
    const auto sigma_reference_iter =
        covariance_sigma_max_reference_cm_.find(source_name);
    if (std::isfinite(sigma_max_cm) &&
        sigma_reference_iter != covariance_sigma_max_reference_cm_.end()) {
      const double reference_cm = sigma_reference_iter->second;
      drawFiniteScalar(base_path + "/sigma_max_reference_cm", reference_cm);
      drawFiniteScalar(base_path + "/sigma_max_delta_cm",
                       sigma_max_cm - reference_cm);
      drawFiniteScalar(base_path + "/sigma_max_delta_mm",
                       (sigma_max_cm - reference_cm) * 10.0);
    }
  }

  void publishPosteriorCovarianceMetricsFromOdometry(
      const nav_msgs::Odometry& msg,
      const std::string& source_name) {
    if (!uncertainty_enable_) {
      return;
    }

    const gtsam::Matrix6 pose_covariance = poseCovarianceFromOdometry(msg);
    const Eigen::Matrix3d covariance =
        translationCovarianceFromPoseCovariance(pose_covariance);
    if (!isUsableCovariance(covariance)) {
      return;
    }

    visualizer_->setTimeNSec(stampToNSec(msg.header.stamp));
    publishPosteriorCovarianceMetrics(source_name, covariance);
    visualizer_->drawScalar("metrics/posterior_covariance/" + source_name +
                                "/uncertainty_frobenius_norm",
                            covariance.norm());
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
      publishPosteriorCovarianceMetricsFromOdometry(kimera_msg, "kimera");
    }
    if (liorf_pose_valid) {
      publishPosteriorCovarianceMetricsFromOdometry(liorf_msg,
                                                    secondary_entity_prefix_);
    }

    if (kimera_pose_valid && publish_raw_kimera_enable_) {
      publishSource(kimera_msg,
                    kimera_pose,
                    "kimera",
                    "base_link",
                    Eigen::Vector4f(40.f, 220.f, 80.f, 220.f),
                    &kimera_,
                    kimera_uncertainty_scale_);
    }
    if (liorf_pose_valid && publish_secondary_enable_) {
      publishSource(liorf_msg,
                    liorf_pose,
                    secondary_entity_prefix_,
                    secondary_frame_name_,
                    Eigen::Vector4f(245.f, 180.f, 20.f, 220.f),
                    &liorf_,
                    secondary_uncertainty_scale_);
    }

    if (publish_legacy_online_alignment_enable_ && liorf_pose_valid) {
      publishGroundTruthAligned(liorf_msg, liorf_pose);
    }

    if (publish_legacy_online_alignment_enable_ && world_alignment_enable_ &&
        publish_aligned_kimera_enable_ && kimera_pose_valid &&
        liorf_pose_valid) {
      publishAlignedKimera(kimera_msg, kimera_pose, liorf_msg, liorf_pose);
    }

    if (liorf_pose_valid) {
      publishCommonAlignedOverlay();
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
        visualizer_->drawScalar("visualization/lag/" +
                                    secondary_entity_prefix_ +
                                    "_stamp_age_sec",
                                (now - liorf_msg.header.stamp).toSec());
        if (std::isfinite(liorf_interval_sec)) {
          visualizer_->drawScalar(
              "visualization/rate/" + secondary_entity_prefix_ +
                  "_stamp_interval_sec",
              liorf_interval_sec);
        }
      }
      if (has_kimera && has_liorf) {
        visualizer_->drawScalar(
            "visualization/lag/kimera_minus_" + secondary_entity_prefix_ +
                "_stamp_sec",
            kimera_msg.header.stamp.toSec() - liorf_msg.header.stamp.toSec());
      }
    }

    if (liorf_point_clouds_enable_) {
      if (liorf_local_map_enable_) {
        publishCloudLatest(liorf_local_map_,
                           secondary_entity_prefix_ + "/local_map",
                           Eigen::Vector4f(80.f, 180.f, 255.f, 90.f),
                           1.0f,
                           liorf_local_map_max_points_);
      }
      if (liorf_current_scan_enable_) {
        publishCloudLatest(liorf_current_scan_,
                           secondary_entity_prefix_ + "/current_scan",
                           Eigen::Vector4f(255.f, 255.f, 255.f, 180.f),
                           1.5f,
                           liorf_current_scan_max_points_);
      }
    }
    if (image_stream_enable_) {
      publishImageLatest();
    }
    if (kimera_landmarks_enable_) {
      publishKimeraLandmarksLatest();
    }
  }

  bool estimateCommonFrameAlignment(
      const std::vector<TimedPose>& source_history,
      const double window_start_stamp_sec,
      const double window_end_stamp_sec,
      const gtsam::Point3& origin,
      AlignmentEstimate* alignment,
      std::vector<gtsam::Pose3>* aligned_poses,
      int* pair_count,
      double* max_nearest_diff_sec) const {
    CHECK_NOTNULL(alignment);
    CHECK_NOTNULL(aligned_poses);
    CHECK_NOTNULL(pair_count);
    CHECK_NOTNULL(max_nearest_diff_sec);
    *pair_count = 0;
    *max_nearest_diff_sec = 0.0;
    aligned_poses->clear();
    if (source_history.size() < 2u) {
      return false;
    }

    std::vector<gtsam::Point3> source_points;
    std::vector<gtsam::Point3> target_points;
    int last_gt_index = -1;
    for (const TimedPose& sample : source_history) {
      if (sample.stamp_sec < window_start_stamp_sec ||
          sample.stamp_sec > window_end_stamp_sec) {
        continue;
      }
      double nearest_diff_sec = std::numeric_limits<double>::infinity();
      const int gt_index =
          nearestGroundTruthIndex(sample.stamp_sec, &nearest_diff_sec);
      if (gt_index < 0 || gt_index == last_gt_index ||
          nearest_diff_sec > ground_truth_max_timestamp_diff_sec_) {
        continue;
      }
      last_gt_index = gt_index;
      *max_nearest_diff_sec =
          std::max(*max_nearest_diff_sec, nearest_diff_sec);
      source_points.push_back(sample.pose.translation());
      target_points.push_back(ground_truth_[gt_index].pose.translation());
    }

    *pair_count = static_cast<int>(source_points.size());
    if (*pair_count < ground_truth_alignment_min_pairs_ ||
        pathLength2D(source_points) < ground_truth_alignment_min_path_length_m_) {
      return false;
    }
    if (!estimateSe3Alignment(source_points, target_points, alignment)) {
      return false;
    }

    aligned_poses->reserve(source_history.size());
    for (const TimedPose& sample : source_history) {
      if (sample.stamp_sec < window_start_stamp_sec ||
          sample.stamp_sec > window_end_stamp_sec) {
        continue;
      }
      aligned_poses->push_back(
          transformPoseSe3ToLocal(sample.pose, *alignment, origin));
    }
    return aligned_poses->size() > 1u;
  }

  bool estimateProvisionalCommonFrameAlignment(
      const std::vector<TimedPose>& source_history,
      const double window_start_stamp_sec,
      const double window_end_stamp_sec,
      const gtsam::Point3& origin,
      AlignmentEstimate* alignment,
      std::vector<gtsam::Pose3>* aligned_poses,
      int* pair_count,
      double* max_nearest_diff_sec) const {
    CHECK_NOTNULL(alignment);
    CHECK_NOTNULL(aligned_poses);
    CHECK_NOTNULL(pair_count);
    CHECK_NOTNULL(max_nearest_diff_sec);
    *pair_count = 0;
    *max_nearest_diff_sec = 0.0;
    aligned_poses->clear();
    if (source_history.empty()) {
      return false;
    }

    const TimedPose* anchor_sample = nullptr;
    int anchor_gt_index = -1;
    double anchor_nearest_diff_sec = std::numeric_limits<double>::infinity();
    for (const TimedPose& sample : source_history) {
      if (sample.stamp_sec < window_start_stamp_sec ||
          sample.stamp_sec > window_end_stamp_sec) {
        continue;
      }
      double nearest_diff_sec = std::numeric_limits<double>::infinity();
      const int gt_index =
          nearestGroundTruthIndex(sample.stamp_sec, &nearest_diff_sec);
      if (gt_index < 0 ||
          nearest_diff_sec > ground_truth_max_timestamp_diff_sec_) {
        continue;
      }
      anchor_sample = &sample;
      anchor_gt_index = gt_index;
      anchor_nearest_diff_sec = nearest_diff_sec;
      break;
    }
    if (anchor_sample == nullptr || anchor_gt_index < 0) {
      return false;
    }

    const gtsam::Pose3& gt_pose = ground_truth_[anchor_gt_index].pose;
    alignment->target_T_source = gt_pose * anchor_sample->pose.inverse();
    alignment->rmse_m = 0.0;
    alignment->mean_m = 0.0;
    alignment->max_m = 0.0;
    alignment->determinant =
        alignment->target_T_source.rotation().matrix().determinant();
    *pair_count = 1;
    *max_nearest_diff_sec = anchor_nearest_diff_sec;

    aligned_poses->reserve(source_history.size());
    for (const TimedPose& sample : source_history) {
      if (sample.stamp_sec < window_start_stamp_sec ||
          sample.stamp_sec > window_end_stamp_sec) {
        continue;
      }
      aligned_poses->push_back(
          transformPoseSe3ToLocal(sample.pose, *alignment, origin));
    }
    return !aligned_poses->empty();
  }

  bool transformCommonFrameHistory(
      const std::vector<TimedPose>& source_history,
      const double window_start_stamp_sec,
      const double window_end_stamp_sec,
      const gtsam::Point3& origin,
      const AlignmentEstimate& alignment,
      std::vector<gtsam::Pose3>* aligned_poses) const {
    CHECK_NOTNULL(aligned_poses);
    aligned_poses->clear();
    if (source_history.size() < 2u) {
      return false;
    }

    aligned_poses->reserve(source_history.size());
    for (const TimedPose& sample : source_history) {
      if (sample.stamp_sec < window_start_stamp_sec ||
          sample.stamp_sec > window_end_stamp_sec) {
        continue;
      }
      aligned_poses->push_back(
          transformPoseSe3ToLocal(sample.pose, alignment, origin));
    }
    return aligned_poses->size() > 1u;
  }

  void drawCommonFrameAlignmentScalars(
      const std::string& entity_prefix,
      const AlignmentEstimate& alignment,
      const int pair_count,
      const double max_nearest_diff_sec,
      const bool provisional = false) {
    visualizer_->drawScalar(entity_prefix + "/alignment/mode_se3", 1.0);
    visualizer_->drawScalar(entity_prefix + "/alignment/provisional",
                            provisional ? 1.0 : 0.0);
    visualizer_->drawScalar(entity_prefix + "/alignment/pairs",
                            static_cast<double>(pair_count));
    visualizer_->drawScalar(entity_prefix +
                                "/alignment/max_nearest_stamp_delta_sec",
                            max_nearest_diff_sec);
    visualizer_->drawScalar(entity_prefix + "/alignment/rmse_m",
                            alignment.rmse_m);
    visualizer_->drawScalar(entity_prefix + "/alignment/mean_m",
                            alignment.mean_m);
    visualizer_->drawScalar(entity_prefix + "/alignment/max_m",
                            alignment.max_m);
    visualizer_->drawScalar(entity_prefix + "/alignment/rotation_determinant",
                            alignment.determinant);
  }

  void publishCommonAlignedOverlay() {
    if (!publish_common_aligned_overlay_enable_ || !world_alignment_enable_ ||
        !ground_truth_enable_ || !ground_truth_loaded_) {
      return;
    }

    std::vector<TimedPose> kimera_history;
    std::vector<TimedPose> secondary_history;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      kimera_history = kimera_.timed_trajectory;
      secondary_history = liorf_.timed_trajectory;
    }
    const bool has_secondary_history = !secondary_history.empty();
    const bool has_kimera_history = !kimera_history.empty();
    if (!has_secondary_history && !has_kimera_history) {
      return;
    }

    double first_common_stamp_sec = std::numeric_limits<double>::infinity();
    double latest_common_stamp_sec = -std::numeric_limits<double>::infinity();
    if (has_secondary_history) {
      first_common_stamp_sec =
          std::min(first_common_stamp_sec, secondary_history.front().stamp_sec);
      latest_common_stamp_sec =
          std::max(latest_common_stamp_sec, secondary_history.back().stamp_sec);
    }
    if (has_kimera_history) {
      first_common_stamp_sec =
          std::min(first_common_stamp_sec, kimera_history.front().stamp_sec);
      latest_common_stamp_sec =
          std::max(latest_common_stamp_sec, kimera_history.back().stamp_sec);
    }
    if (!std::isfinite(first_common_stamp_sec) ||
        !std::isfinite(latest_common_stamp_sec) ||
        latest_common_stamp_sec + 1e-9 < first_common_stamp_sec) {
      return;
    }

    double window_end_stamp_sec = latest_common_stamp_sec;
    if (ground_truth_window_duration_sec_ > 0.0) {
      window_end_stamp_sec =
          std::min(window_end_stamp_sec,
                   first_common_stamp_sec + ground_truth_window_duration_sec_);
    }
    if (std::isfinite(last_common_aligned_published_until_stamp_sec_) &&
        window_end_stamp_sec <=
            last_common_aligned_published_until_stamp_sec_ + 1e-6) {
      return;
    }

    double origin_diff_sec = std::numeric_limits<double>::infinity();
    int origin_index =
        nearestGroundTruthIndex(first_common_stamp_sec, &origin_diff_sec);
    if (origin_index < 0) {
      return;
    }
    const gtsam::Point3 origin = ground_truth_[origin_index].pose.translation();

    std::vector<gtsam::Pose3> gt_local_poses;
    gt_local_poses.reserve(ground_truth_.size());
    for (const GroundTruthPose& gt_pose : ground_truth_) {
      if (gt_pose.stamp_sec + ground_truth_max_timestamp_diff_sec_ <
              first_common_stamp_sec ||
          gt_pose.stamp_sec - ground_truth_max_timestamp_diff_sec_ >
              window_end_stamp_sec) {
        continue;
      }
      const Eigen::Vector3d local =
          gt_pose.pose.translation() - origin;
      const gtsam::Pose3 local_pose(
          gt_pose.pose.rotation(),
          gtsam::Point3(local.x(), local.y(), local.z()));
      gt_local_poses.push_back(local_pose);
    }
    if (gt_local_poses.empty()) {
      return;
    }

    AlignmentEstimate secondary_alignment;
    AlignmentEstimate kimera_alignment;
    std::vector<gtsam::Pose3> aligned_secondary_poses;
    std::vector<gtsam::Pose3> aligned_kimera_poses;
    int secondary_pairs = 0;
    int kimera_pairs = 0;
    double secondary_max_nearest_diff_sec = 0.0;
    double kimera_max_nearest_diff_sec = 0.0;
    bool secondary_ok = false;
    bool secondary_provisional = false;
    if (has_secondary_history && freeze_common_aligned_overlay_alignment_enable_ &&
        secondary_common_alignment_frozen_) {
      secondary_alignment = frozen_secondary_common_alignment_;
      secondary_pairs = frozen_secondary_common_alignment_pairs_;
      secondary_max_nearest_diff_sec =
          frozen_secondary_common_alignment_max_nearest_diff_sec_;
      secondary_ok = transformCommonFrameHistory(secondary_history,
                                                 first_common_stamp_sec,
                                                 window_end_stamp_sec,
                                                 origin,
                                                 secondary_alignment,
                                                 &aligned_secondary_poses);
    } else if (has_secondary_history) {
      secondary_ok = estimateCommonFrameAlignment(
          secondary_history,
          first_common_stamp_sec,
          window_end_stamp_sec,
          origin,
          &secondary_alignment,
          &aligned_secondary_poses,
          &secondary_pairs,
          &secondary_max_nearest_diff_sec);
      if (secondary_ok && freeze_common_aligned_overlay_alignment_enable_) {
        frozen_secondary_common_alignment_ = secondary_alignment;
        frozen_secondary_common_alignment_pairs_ = secondary_pairs;
        frozen_secondary_common_alignment_max_nearest_diff_sec_ =
            secondary_max_nearest_diff_sec;
        secondary_common_alignment_frozen_ = true;
        LOG(INFO) << "Froze Rerun common-frame alignment for "
                  << secondary_entity_prefix_ << ". pairs=" << secondary_pairs
                  << ", rmse_m=" << secondary_alignment.rmse_m
                  << ", max_timestamp_delta_sec="
                  << secondary_max_nearest_diff_sec << ".";
      }
    }
    if (!secondary_ok && has_secondary_history &&
        common_aligned_provisional_start_enable_) {
      secondary_ok = estimateProvisionalCommonFrameAlignment(
          secondary_history,
          first_common_stamp_sec,
          window_end_stamp_sec,
          origin,
          &secondary_alignment,
          &aligned_secondary_poses,
          &secondary_pairs,
          &secondary_max_nearest_diff_sec);
      secondary_provisional = secondary_ok;
    }

    bool kimera_ok = false;
    bool kimera_provisional = false;
    if (has_kimera_history) {
      if (freeze_common_aligned_overlay_alignment_enable_ &&
          kimera_common_alignment_frozen_) {
        kimera_alignment = frozen_kimera_common_alignment_;
        kimera_pairs = frozen_kimera_common_alignment_pairs_;
        kimera_max_nearest_diff_sec =
            frozen_kimera_common_alignment_max_nearest_diff_sec_;
        kimera_ok = transformCommonFrameHistory(kimera_history,
                                                first_common_stamp_sec,
                                                window_end_stamp_sec,
                                                origin,
                                                kimera_alignment,
                                                &aligned_kimera_poses);
      } else {
        kimera_ok = estimateCommonFrameAlignment(kimera_history,
                                                first_common_stamp_sec,
                                                window_end_stamp_sec,
                                                origin,
                                                &kimera_alignment,
                                                &aligned_kimera_poses,
                                                &kimera_pairs,
                                                &kimera_max_nearest_diff_sec);
        if (kimera_ok && freeze_common_aligned_overlay_alignment_enable_) {
          frozen_kimera_common_alignment_ = kimera_alignment;
          frozen_kimera_common_alignment_pairs_ = kimera_pairs;
          frozen_kimera_common_alignment_max_nearest_diff_sec_ =
              kimera_max_nearest_diff_sec;
          kimera_common_alignment_frozen_ = true;
          LOG(INFO) << "Froze Rerun common-frame alignment for Kimera. pairs="
                    << kimera_pairs << ", rmse_m=" << kimera_alignment.rmse_m
                    << ", max_timestamp_delta_sec="
                    << kimera_max_nearest_diff_sec << ".";
        }
      }
    }
    if (!kimera_ok && has_kimera_history &&
        common_aligned_provisional_start_enable_) {
      kimera_ok = estimateProvisionalCommonFrameAlignment(
          kimera_history,
          first_common_stamp_sec,
          window_end_stamp_sec,
          origin,
          &kimera_alignment,
          &aligned_kimera_poses,
          &kimera_pairs,
          &kimera_max_nearest_diff_sec);
      kimera_provisional = kimera_ok;
    }
    if (!secondary_ok && !kimera_ok) {
      return;
    }
    if (anchor_common_aligned_start_enable_) {
      anchorTrajectoryStartToOrigin(&gt_local_poses);
      if (secondary_ok) {
        anchorTrajectoryStartToOrigin(&aligned_secondary_poses);
      }
      if (kimera_ok) {
        anchorTrajectoryStartToOrigin(&aligned_kimera_poses);
      }
    }
    std::vector<gtsam::Point3> gt_local_points;
    gt_local_points.reserve(gt_local_poses.size());
    for (const gtsam::Pose3& gt_pose : gt_local_poses) {
      gt_local_points.push_back(gt_pose.translation());
    }

    const uint64_t stamp_nsec = stampSecToNSec(window_end_stamp_sec);
    visualizer_->setTimeNSec(stamp_nsec);
    const Eigen::Vector4f gt_rgba(65.f, 140.f, 255.f, 220.f);
    const Eigen::Vector4f secondary_rgba(245.f, 180.f, 20.f, 220.f);
    const Eigen::Vector4f kimera_rgba(40.f, 220.f, 80.f, 220.f);
    visualizer_->drawTrajectory(
        "aligned/ground_truth/trajectory", gt_local_poses, gt_rgba, 2.0f);
    visualizer_->drawPoints(
        "aligned/ground_truth/samples", gt_local_points, gt_rgba, 0.12f);
    if (secondary_ok) {
      const std::string secondary_prefix = "aligned/" + secondary_entity_prefix_;
      visualizer_->drawTf(secondary_prefix + "/" + secondary_frame_name_,
                          aligned_secondary_poses.back(),
                          0.5f);
      visualizer_->drawTrajectory(secondary_prefix + "/trajectory",
                                  aligned_secondary_poses,
                                  secondary_rgba,
                                  1.75f);
      drawCommonFrameAlignmentScalars(secondary_prefix,
                                      secondary_alignment,
                                      secondary_pairs,
                                      secondary_max_nearest_diff_sec,
                                      secondary_provisional);
      latest_secondary_common_alignment_ = secondary_alignment;
      latest_common_origin_ = origin;
      secondary_common_alignment_ready_for_clouds_ = true;
    }
    if (kimera_ok) {
      visualizer_->drawTf("aligned/kimera/base_link",
                          aligned_kimera_poses.back(),
                          0.5f);
      visualizer_->drawTrajectory("aligned/kimera/trajectory",
                                  aligned_kimera_poses,
                                  kimera_rgba,
                                  1.75f);
      drawCommonFrameAlignmentScalars("aligned/kimera",
                                      kimera_alignment,
                                      kimera_pairs,
                                      kimera_max_nearest_diff_sec,
                                      kimera_provisional);
    }
    if (secondary_ok) {
      visualizer_->drawScalar("aligned/metadata/" +
                                  secondary_entity_prefix_ +
                                  "_alignment_pairs",
                              static_cast<double>(secondary_pairs));
      visualizer_->drawScalar("aligned/metadata/" +
                                  secondary_entity_prefix_ +
                                  "_alignment_rmse_m",
                              secondary_alignment.rmse_m);
    }
    if (kimera_ok) {
      visualizer_->drawScalar("aligned/metadata/kimera_alignment_pairs",
                              static_cast<double>(kimera_pairs));
      visualizer_->drawScalar("aligned/metadata/kimera_alignment_rmse_m",
                              kimera_alignment.rmse_m);
    }
    if (secondary_entity_prefix_ == "glim") {
      visualizer_->drawScalar("aligned/metadata/common_alignment_glim_to_gt",
                              0.0);
    }
    visualizer_->drawScalar("aligned/metadata/metric_per_estimator_alignment",
                            1.0);
    visualizer_->drawScalar("aligned/metadata/common_frame_ready", 1.0);
    visualizer_->drawScalar("aligned/metadata/window_start_stamp_sec",
                            first_common_stamp_sec);
    visualizer_->drawScalar("aligned/metadata/window_end_stamp_sec",
                            window_end_stamp_sec);
    visualizer_->drawScalar("aligned/metadata/origin_gt_stamp_sec",
                            ground_truth_[origin_index].stamp_sec);
    visualizer_->drawScalar("aligned/metadata/origin_gt_diff_sec",
                            origin_diff_sec);
    last_common_aligned_published_until_stamp_sec_ = window_end_stamp_sec;
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

  bool takeImageForPublish(sensor_msgs::CompressedImage* msg) {
    CHECK_NOTNULL(msg);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!image_.has_latest) {
      return false;
    }

    const double stamp_sec = image_.latest.header.stamp.toSec();
    if (std::isfinite(image_.last_published_stamp_sec) &&
        stamp_sec <= image_.last_published_stamp_sec) {
      return false;
    }
    if (std::isfinite(image_max_hz_) && image_max_hz_ > 0.0 &&
        std::isfinite(image_.last_published_stamp_sec) &&
        stamp_sec - image_.last_published_stamp_sec <
            (1.0 / image_max_hz_) - 1e-6) {
      return false;
    }

    image_.last_published_stamp_sec = stamp_sec;
    *msg = image_.latest;
    return true;
  }

  bool lookupSecondarySourceFrameToCloudFrame(
      const sensor_msgs::PointCloud2& msg, gtsam::Pose3* source_T_cloud) {
    CHECK_NOTNULL(source_T_cloud);

    nav_msgs::Odometry secondary_msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!liorf_.has_latest) {
        return false;
      }
      secondary_msg = liorf_.latest;
    }

    gtsam::Pose3 world_T_body;
    if (!odometryToPose(secondary_msg, &world_T_body)) {
      return false;
    }

    const std::string target_frame =
        !secondary_msg.child_frame_id.empty() ? secondary_msg.child_frame_id
                                              : secondary_frame_name_;
    const std::string source_frame = msg.header.frame_id;
    if (target_frame.empty() || source_frame.empty()) {
      return false;
    }
    if (target_frame == source_frame) {
      *source_T_cloud = world_T_body;
      return true;
    }

    try {
      const geometry_msgs::TransformStamped tf_msg =
          tf_buffer_.lookupTransform(
              target_frame,
              source_frame,
              msg.header.stamp,
              ros::Duration(secondary_point_cloud_tf_lookup_timeout_sec_));
      *source_T_cloud = world_T_body.compose(transformStampedToPose3(tf_msg));
      return true;
    } catch (const std::exception& e) {
      LOG(WARNING) << "Rerun point-cloud transform unavailable for "
                   << secondary_entity_prefix_ << ": " << target_frame
                   << " <- " << source_frame << " at "
                   << msg.header.stamp.toSec()
                   << " sec: " << e.what();
      return false;
    }
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

    if (secondary_point_cloud_common_alignment_enable_ &&
        secondary_common_alignment_ready_for_clouds_) {
      gtsam::Pose3 source_T_cloud;
      if (lookupSecondarySourceFrameToCloudFrame(msg, &source_T_cloud)) {
        std::vector<gtsam::Point3> aligned_points;
        aligned_points.reserve(points.size());
        for (const auto& point : points) {
          const gtsam::Point3 source_point = source_T_cloud.transformFrom(point);
          aligned_points.push_back(transformPointSe3ToLocal(
              source_point,
              latest_secondary_common_alignment_,
              latest_common_origin_));
        }

        if (secondary_point_cloud_publish_raw_enable_) {
          visualizer_->drawPoints(entity_path, points, rgba, radius);
        } else if (!cloud.raw_entity_cleared) {
          visualizer_->clearEntity(entity_path);
          cloud.raw_entity_cleared = true;
        }
        visualizer_->drawPoints("aligned/" + entity_path,
                                aligned_points,
                                rgba,
                                radius);
        return;
      }

      ROS_WARN_STREAM_THROTTLE(
          2.0,
          "Failed to align secondary cloud for Rerun entity " << entity_path);
      if (!secondary_point_cloud_publish_raw_enable_) {
        return;
      }
    }

    if (secondary_point_cloud_common_alignment_enable_ &&
        !secondary_common_alignment_ready_for_clouds_ &&
        !secondary_point_cloud_publish_raw_enable_) {
      return;
    }

    visualizer_->drawPoints(entity_path, points, rgba, radius);
    cloud.raw_entity_cleared = false;
  }

  void publishImageLatest() {
    sensor_msgs::CompressedImage msg;
    if (!takeImageForPublish(&msg) || msg.data.empty()) {
      return;
    }

    const std::vector<uint8_t> encoded(msg.data.begin(), msg.data.end());
    const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
      LOG(WARNING) << "Could not decode compressed Rerun image from topic "
                   << image_topic_ << ".";
      return;
    }

    visualizer_->setTimeNSec(stampToNSec(msg.header.stamp));
    visualizer_->drawImage(image_entity_path_, image, false);
    ++image_published_count_;
    visualizer_->drawScalar("video/metadata/live_image_count",
                            static_cast<double>(image_published_count_));
  }

  int nearestGroundTruthIndex(const double stamp_sec,
                              double* nearest_diff_sec) const {
    CHECK_NOTNULL(nearest_diff_sec);
    *nearest_diff_sec = std::numeric_limits<double>::infinity();
    if (!std::isfinite(stamp_sec) || ground_truth_.empty()) {
      return -1;
    }

    const auto iter = std::lower_bound(
        ground_truth_.begin(),
        ground_truth_.end(),
        stamp_sec,
        [](const GroundTruthPose& pose, const double stamp) {
          return pose.stamp_sec < stamp;
        });

    int best_index = -1;
    auto consider = [&](std::vector<GroundTruthPose>::const_iterator candidate) {
      if (candidate == ground_truth_.end()) {
        return;
      }
      const double diff = std::abs(candidate->stamp_sec - stamp_sec);
      if (diff < *nearest_diff_sec) {
        *nearest_diff_sec = diff;
        best_index = static_cast<int>(
            std::distance(ground_truth_.begin(), candidate));
      }
    };

    consider(iter);
    if (iter != ground_truth_.begin()) {
      consider(std::prev(iter));
    }
    return best_index;
  }

  void publishGroundTruthAligned(const nav_msgs::Odometry& secondary_msg,
                                 const gtsam::Pose3& secondary_pose) {
    if (!ground_truth_enable_ || !ground_truth_loaded_) {
      return;
    }

    const double secondary_stamp_sec = secondary_msg.header.stamp.toSec();
    if (!std::isfinite(secondary_stamp_sec)) {
      return;
    }
    if (!std::isfinite(last_ground_truth_alignment_observation_stamp_sec_) ||
        secondary_stamp_sec >
            last_ground_truth_alignment_observation_stamp_sec_) {
      ground_truth_alignment_observations_.push_back(
          {secondary_stamp_sec, secondary_pose});
      last_ground_truth_alignment_observation_stamp_sec_ = secondary_stamp_sec;
    }

    std::vector<gtsam::Point3> gt_alignment_points;
    std::vector<gtsam::Point3> secondary_alignment_points;
    gt_alignment_points.reserve(ground_truth_alignment_observations_.size());
    secondary_alignment_points.reserve(
        ground_truth_alignment_observations_.size());

    int last_gt_index = -1;
    double max_nearest_diff_sec = 0.0;
    for (const TimedPose& observation : ground_truth_alignment_observations_) {
      double nearest_diff_sec = std::numeric_limits<double>::infinity();
      const int gt_index =
          nearestGroundTruthIndex(observation.stamp_sec, &nearest_diff_sec);
      if (gt_index < 0 || gt_index == last_gt_index ||
          nearest_diff_sec > ground_truth_max_timestamp_diff_sec_) {
        continue;
      }
      last_gt_index = gt_index;
      max_nearest_diff_sec = std::max(max_nearest_diff_sec, nearest_diff_sec);
      gt_alignment_points.push_back(ground_truth_[gt_index].pose.translation());
      secondary_alignment_points.push_back(observation.pose.translation());
    }

    const double alignment_path_length_m =
        pathLength2D(secondary_alignment_points);
    if (static_cast<int>(gt_alignment_points.size()) <
            ground_truth_alignment_min_pairs_ ||
        alignment_path_length_m < ground_truth_alignment_min_path_length_m_) {
      if (!ground_truth_wait_logged_) {
        LOG(INFO) << "Waiting for Rerun ground-truth alignment; have "
                  << gt_alignment_points.size() << " associated GT/"
                  << secondary_entity_prefix_ << " pairs over "
                  << alignment_path_length_m << " m.";
        ground_truth_wait_logged_ = true;
      }
      return;
    }

    AlignmentEstimate alignment;
    if (!estimateSe3Alignment(gt_alignment_points,
                              secondary_alignment_points,
                              &alignment)) {
      if (!ground_truth_wait_logged_) {
        LOG(WARNING) << "Could not estimate Rerun ground-truth SE(3) "
                     << "alignment.";
        ground_truth_wait_logged_ = true;
      }
      return;
    }

    std::vector<gtsam::Pose3> aligned_poses;
    std::vector<gtsam::Point3> aligned_points;
    aligned_poses.reserve(ground_truth_.size());
    aligned_points.reserve(ground_truth_.size());
    const double window_start_stamp_sec =
        ground_truth_alignment_observations_.front().stamp_sec;
    double window_end_stamp_sec = secondary_stamp_sec;
    if (ground_truth_window_duration_sec_ > 0.0) {
      window_end_stamp_sec =
          std::min(window_end_stamp_sec,
                   window_start_stamp_sec + ground_truth_window_duration_sec_);
    }
    if (!std::isfinite(window_end_stamp_sec) ||
        window_end_stamp_sec <= window_start_stamp_sec) {
      return;
    }
    if (std::isfinite(last_ground_truth_published_until_stamp_sec_) &&
        window_end_stamp_sec <=
            last_ground_truth_published_until_stamp_sec_ + 1e-6) {
      return;
    }
    for (const GroundTruthPose& gt_pose : ground_truth_) {
      if (gt_pose.stamp_sec + ground_truth_max_timestamp_diff_sec_ <
              window_start_stamp_sec ||
          gt_pose.stamp_sec - ground_truth_max_timestamp_diff_sec_ >
              window_end_stamp_sec) {
        continue;
      }
      const gtsam::Pose3 aligned_pose =
          alignment.target_T_source * gt_pose.pose;
      aligned_poses.push_back(aligned_pose);
      aligned_points.push_back(aligned_pose.translation());
    }
    if (aligned_poses.size() < 2u) {
      return;
    }

    const Eigen::Vector4f gt_rgba(65.f, 140.f, 255.f, 220.f);
    const uint64_t ground_truth_stamp_nsec =
        stampSecToNSec(window_end_stamp_sec);
    visualizer_->setTimeNSec(ground_truth_stamp_nsec > 0ull
                                 ? ground_truth_stamp_nsec
                                 : stampToNSec(secondary_msg.header.stamp));
    visualizer_->drawTrajectory(
        "ground_truth/trajectory", aligned_poses, gt_rgba, 2.0f);
    visualizer_->drawPoints(
        "ground_truth/samples", aligned_points, gt_rgba, 0.12f);
    visualizer_->drawScalar("ground_truth/alignment/initialized", 1.0);
    visualizer_->drawScalar("ground_truth/alignment/pairs",
                            static_cast<double>(gt_alignment_points.size()));
    visualizer_->drawScalar("ground_truth/alignment/max_nearest_stamp_delta_sec",
                            max_nearest_diff_sec);
    visualizer_->drawScalar("ground_truth/alignment/path_length_m",
                            alignment_path_length_m);
    visualizer_->drawScalar("ground_truth/alignment/rmse_m",
                            alignment.rmse_m);
    visualizer_->drawScalar("ground_truth/alignment/mean_m",
                            alignment.mean_m);
    visualizer_->drawScalar("ground_truth/alignment/max_m", alignment.max_m);
    visualizer_->drawScalar("ground_truth/alignment/rotation_determinant",
                            alignment.determinant);
    visualizer_->drawScalar("ground_truth/alignment/source_stamp_sec",
                            secondary_msg.header.stamp.toSec());
    visualizer_->drawScalar("ground_truth/alignment/window_duration_sec",
                            window_end_stamp_sec - window_start_stamp_sec);
    visualizer_->drawScalar("ground_truth/alignment/published_until_stamp_sec",
                            window_end_stamp_sec);
    last_ground_truth_published_until_stamp_sec_ = window_end_stamp_sec;

    if (!ground_truth_published_) {
      LOG(INFO) << "Published Rerun ground truth aligned to "
                << secondary_entity_prefix_ << " using online SE(3) "
                << "alignment. pairs=" << gt_alignment_points.size()
                << ", window_sec="
                << (window_end_stamp_sec - window_start_stamp_sec)
                << ", rmse_m=" << alignment.rmse_m
                << ", max_timestamp_delta_sec=" << max_nearest_diff_sec
                << ".";
      ground_truth_published_ = true;
    }
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

    const Eigen::Vector4f rgba(210.f, 80.f, 255.f, 180.f);
    visualizer_->setTimeNSec(stampToNSec(msg.header.stamp));
    if (publish_raw_kimera_enable_) {
      visualizer_->drawPoints("kimera/landmarks", landmarks, rgba, 2.f);
    }

    if (!alignment_initialized_ || !publish_aligned_kimera_enable_) {
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
                     SourceState* source,
                     double uncertainty_scale) {
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
            static_cast<float>(uncertainty_scale));
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
      LOG(INFO) << "Rerun topic visualizer initialized Kimera->"
                << secondary_entity_prefix_
                << " world alignment. timestamp_delta_sec="
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
      const gtsam::Matrix6 pose_covariance =
          poseCovarianceFromOdometry(kimera_msg);
      const Eigen::Matrix3d rotation = liorf_T_kimera_world_.rotation().matrix();
      const gtsam::Matrix6 aligned_pose_covariance =
          rotatePoseCovariance6x6(pose_covariance, rotation);
      if (raw_covariance_enable_) {
        drawRawPoseCovariance6x6(
            visualizer_.get(),
            "kimera_aligned/current_pose/raw_pose_covariance_6x6",
            aligned_pose_covariance);
      }
      const Eigen::Matrix3d covariance =
          translationCovarianceFromPoseCovariance(aligned_pose_covariance);
      if (isUsableCovariance(covariance)) {
        publishPosteriorCovarianceMetrics("kimera_aligned", covariance);
        visualizer_->drawUncertainty(
            "kimera_aligned/current_pose/uncertainty",
            aligned_pose,
            covariance,
            rgba,
            static_cast<float>(aligned_kimera_uncertainty_scale_));
        visualizer_->drawScalar(
            "kimera_aligned/current_pose/uncertainty_frobenius_norm",
            covariance.norm());
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber kimera_sub_;
  ros::Subscriber liorf_sub_;
  ros::Subscriber liorf_local_map_sub_;
  ros::Subscriber liorf_current_scan_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber kimera_landmarks_sub_;
  ros::Subscriber cbs_g2k_sub_;
  ros::Subscriber cbs_k2g_sub_;
  ros::WallTimer timer_;
  std::mutex mutex_;
  SourceState kimera_;
  SourceState liorf_;
  CloudState liorf_local_map_;
  CloudState liorf_current_scan_;
  ImageState image_;
  CloudState kimera_landmarks_;
  BeliefTrafficState cbs_g2k_;
  BeliefTrafficState cbs_k2g_;
  std::unique_ptr<RosRerunVisualizer> visualizer_;
  std::unique_ptr<rerun::RecordingStream> blueprint_stream_;

  std::string kimera_odom_topic_;
  std::string liorf_odom_topic_;
  std::string secondary_entity_prefix_ = "liorf";
  std::string secondary_frame_name_ = "lidar_link";
  std::string liorf_local_map_topic_;
  std::string liorf_current_scan_topic_;
  std::string kimera_landmarks_topic_;
  std::string cbs_g2k_odom_belief_topic_;
  std::string cbs_k2g_odom_belief_topic_;
  std::string ground_truth_path_;
  std::string image_topic_;
  std::string image_entity_path_ = "video/rgb/image";
  std::string recording_id_;
  std::string rerun_host_;
  double publish_rate_hz_ = 5.0;
  double kimera_uncertainty_scale_ = 1.25;
  double secondary_uncertainty_scale_ = 1.25;
  double aligned_kimera_uncertainty_scale_ = 1.25;
  double ground_truth_max_timestamp_diff_sec_ = 0.75;
  double ground_truth_alignment_min_path_length_m_ = 3.0;
  double ground_truth_window_duration_sec_ = 60.0;
  double image_max_hz_ = 10.0;
  double secondary_point_cloud_tf_lookup_timeout_sec_ = 0.05;
  bool uncertainty_enable_ = true;
  bool raw_covariance_enable_ = true;
  bool lag_scalars_enable_ = true;
  bool world_alignment_enable_ = true;
  bool liorf_point_clouds_enable_ = true;
  bool liorf_local_map_enable_ = true;
  bool liorf_current_scan_enable_ = true;
  bool image_stream_enable_ = false;
  bool secondary_point_cloud_common_alignment_enable_ = true;
  bool secondary_point_cloud_publish_raw_enable_ = false;
  bool kimera_landmarks_enable_ = true;
  bool cbs_metrics_enable_ = false;
  bool accumulated_covariance_enable_ = true;
  bool publish_raw_kimera_enable_ = true;
  bool publish_secondary_enable_ = true;
  bool publish_aligned_kimera_enable_ = true;
  bool publish_legacy_online_alignment_enable_ = false;
  bool publish_common_aligned_overlay_enable_ = true;
  bool anchor_common_aligned_start_enable_ = false;
  bool common_aligned_provisional_start_enable_ = false;
  bool freeze_common_aligned_overlay_alignment_enable_ = false;
  bool ground_truth_enable_ = false;
  bool raw_cbs_dashboard_blueprint_enable_ = false;
  bool ground_truth_loaded_ = false;
  bool ground_truth_published_ = false;
  bool ground_truth_wait_logged_ = false;
  int max_trajectory_len_ = 2000;
  int odometry_queue_size_ = 512;
  int point_cloud_queue_size_ = 2;
  int image_queue_size_ = 2;
  int cbs_metrics_queue_size_ = 200;
  int ground_truth_alignment_min_pairs_ = 8;
  int liorf_local_map_max_points_ = 10000;
  int liorf_current_scan_max_points_ = 20000;
  int kimera_landmarks_max_points_ = 3000;

  bool alignment_initialized_ = false;
  double alignment_timestamp_delta_sec_ = 0.0;
  double last_aligned_kimera_stamp_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  gtsam::Pose3 liorf_T_kimera_world_;
  std::vector<gtsam::Pose3> aligned_kimera_trajectory_;
  bool secondary_common_alignment_frozen_ = false;
  bool kimera_common_alignment_frozen_ = false;
  AlignmentEstimate frozen_secondary_common_alignment_;
  AlignmentEstimate frozen_kimera_common_alignment_;
  AlignmentEstimate latest_secondary_common_alignment_;
  gtsam::Point3 latest_common_origin_;
  int frozen_secondary_common_alignment_pairs_ = 0;
  int frozen_kimera_common_alignment_pairs_ = 0;
  double frozen_secondary_common_alignment_max_nearest_diff_sec_ = 0.0;
  double frozen_kimera_common_alignment_max_nearest_diff_sec_ = 0.0;
  bool secondary_common_alignment_ready_for_clouds_ = false;
  size_t image_published_count_ = 0u;
  std::map<std::string, double> covariance_trace_reference_m2_;
  std::map<std::string, double> covariance_sigma_max_reference_cm_;
  std::vector<GroundTruthPose> ground_truth_;
  std::vector<TimedPose> ground_truth_alignment_observations_;
  double last_ground_truth_alignment_observation_stamp_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_ground_truth_published_until_stamp_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_common_aligned_published_until_stamp_sec_ =
      std::numeric_limits<double>::quiet_NaN();
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
