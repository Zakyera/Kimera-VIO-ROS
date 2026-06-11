#include "kimera_vio_ros/RosRerunVisualizer.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct TimedPose {
  double stamp_sec = 0.0;
  gtsam::Pose3 pose;
  gtsam::Matrix6 pose_covariance = gtsam::Matrix6::Zero();
  bool has_covariance = false;
};

struct Trajectory {
  std::vector<TimedPose> poses;
};

struct TimedImage {
  double stamp_sec = 0.0;
  std::string entity_path;
  cv::Mat image;
};

struct Alignment {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> tokens;
  std::string token;
  bool in_quotes = false;
  for (char c : line) {
    if (c == '"') {
      in_quotes = !in_quotes;
    } else if (c == ',' && !in_quotes) {
      tokens.push_back(token);
      token.clear();
    } else {
      token.push_back(c);
    }
  }
  tokens.push_back(token);
  return tokens;
}

double parseDouble(const std::string& text,
                   const double fallback = std::numeric_limits<double>::quiet_NaN()) {
  if (text.empty()) {
    return fallback;
  }
  try {
    return std::stod(text);
  } catch (const std::exception&) {
    return fallback;
  }
}

uint64_t stampSecToNSec(const double stamp_sec) {
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return 0ull;
  }
  return static_cast<uint64_t>(std::llround(stamp_sec * 1.0e9));
}

double normalizeStamp(double stamp) {
  if (stamp > 1.0e12) {
    stamp *= 1.0e-9;
  }
  return stamp;
}

Trajectory readTum(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open TUM trajectory: " + path);
  }

  Trajectory trajectory;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream stream(line);
    double stamp;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
    if (!(stream >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      continue;
    }
    TimedPose pose;
    pose.stamp_sec = stamp;
    pose.pose = gtsam::Pose3(gtsam::Rot3::Quaternion(qw, qx, qy, qz),
                             gtsam::Point3(x, y, z));
    trajectory.poses.push_back(pose);
  }
  if (trajectory.poses.empty()) {
    throw std::runtime_error("No TUM poses loaded from: " + path);
  }
  return trajectory;
}

Trajectory readOdometryCsv(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open odometry CSV: " + path);
  }

  std::string header_line;
  if (!std::getline(file, header_line)) {
    throw std::runtime_error("Empty odometry CSV: " + path);
  }
  const std::vector<std::string> headers = splitCsvLine(header_line);
  std::unordered_map<std::string, size_t> index;
  for (size_t i = 0; i < headers.size(); ++i) {
    index.emplace(headers[i], i);
  }

  auto get = [&](const std::vector<std::string>& row,
                 const std::string& key,
                 const double fallback = std::numeric_limits<double>::quiet_NaN()) {
    const auto iter = index.find(key);
    if (iter == index.end() || iter->second >= row.size()) {
      return fallback;
    }
    return parseDouble(row[iter->second], fallback);
  };

  Trajectory trajectory;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> row = splitCsvLine(line);
    const double stamp = normalizeStamp(
        get(row, "field.header.stamp", get(row, "%time", 0.0)));
    const double x = get(row, "field.pose.pose.position.x");
    const double y = get(row, "field.pose.pose.position.y");
    const double z = get(row, "field.pose.pose.position.z");
    const double qx = get(row, "field.pose.pose.orientation.x");
    const double qy = get(row, "field.pose.pose.orientation.y");
    const double qz = get(row, "field.pose.pose.orientation.z");
    const double qw = get(row, "field.pose.pose.orientation.w");
    if (!std::isfinite(stamp) || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(z) || !std::isfinite(qx) || !std::isfinite(qy) ||
        !std::isfinite(qz) || !std::isfinite(qw)) {
      continue;
    }

    TimedPose pose;
    pose.stamp_sec = stamp;
    pose.pose = gtsam::Pose3(gtsam::Rot3::Quaternion(qw, qx, qy, qz),
                             gtsam::Point3(x, y, z));

    static const int remapping[6] = {3, 4, 5, 0, 1, 2};
    bool covariance_ok = true;
    for (int row_idx = 0; row_idx < 6; ++row_idx) {
      for (int col_idx = 0; col_idx < 6; ++col_idx) {
        const int ros_row = remapping[row_idx];
        const int ros_col = remapping[col_idx];
        const double value = get(
            row,
            "field.pose.covariance" + std::to_string(ros_row * 6 + ros_col));
        pose.pose_covariance(row_idx, col_idx) = value;
        covariance_ok = covariance_ok && std::isfinite(value);
      }
    }
    pose.has_covariance = covariance_ok;
    trajectory.poses.push_back(pose);
  }
  if (trajectory.poses.empty()) {
    throw std::runtime_error("No odometry poses loaded from: " + path);
  }
  std::sort(trajectory.poses.begin(),
            trajectory.poses.end(),
            [](const TimedPose& a, const TimedPose& b) {
              return a.stamp_sec < b.stamp_sec;
            });
  return trajectory;
}

int nearestPoseIndex(const Trajectory& trajectory,
                     const double stamp_sec,
                     double* diff_sec) {
  if (trajectory.poses.empty()) {
    return -1;
  }
  const auto iter = std::lower_bound(
      trajectory.poses.begin(),
      trajectory.poses.end(),
      stamp_sec,
      [](const TimedPose& pose, const double stamp) {
        return pose.stamp_sec < stamp;
      });

  int best = -1;
  double best_diff = std::numeric_limits<double>::infinity();
  auto consider = [&](std::vector<TimedPose>::const_iterator candidate) {
    if (candidate == trajectory.poses.end()) {
      return;
    }
    const double diff = std::abs(candidate->stamp_sec - stamp_sec);
    if (diff < best_diff) {
      best_diff = diff;
      best = static_cast<int>(std::distance(trajectory.poses.begin(), candidate));
    }
  };
  consider(iter);
  if (iter != trajectory.poses.begin()) {
    consider(std::prev(iter));
  }
  if (diff_sec) {
    *diff_sec = best_diff;
  }
  return best;
}

Alignment estimateSe3(const std::vector<Eigen::Vector3d>& source,
                      const std::vector<Eigen::Vector3d>& target) {
  if (source.size() != target.size() || source.size() < 3u) {
    throw std::runtime_error("Need at least three paired points for SE3 alignment");
  }

  Eigen::Vector3d source_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_mean = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < source.size(); ++i) {
    source_mean += source[i];
    target_mean += target[i];
  }
  source_mean /= static_cast<double>(source.size());
  target_mean /= static_cast<double>(target.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < source.size(); ++i) {
    covariance += (source[i] - source_mean) * (target[i] - target_mean).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix3d v = svd.matrixV();
    v.col(2) *= -1.0;
    rotation = v * svd.matrixU().transpose();
  }

  Alignment alignment;
  alignment.rotation = rotation;
  alignment.translation = target_mean - rotation * source_mean;
  return alignment;
}

Alignment estimateAlignmentToGroundTruth(const Trajectory& source,
                                         const Trajectory& gt,
                                         const double max_diff_sec,
                                         int* pair_count,
                                         double* rmse_m) {
  std::vector<Eigen::Vector3d> source_points;
  std::vector<Eigen::Vector3d> gt_points;

  int last_gt_index = -1;
  for (const TimedPose& pose : source.poses) {
    double diff_sec = std::numeric_limits<double>::infinity();
    const int gt_index = nearestPoseIndex(gt, pose.stamp_sec, &diff_sec);
    if (gt_index < 0 || gt_index == last_gt_index || diff_sec > max_diff_sec) {
      continue;
    }
    last_gt_index = gt_index;
    source_points.push_back(pose.pose.translation());
    gt_points.push_back(gt.poses[gt_index].pose.translation());
  }

  Alignment alignment = estimateSe3(source_points, gt_points);
  double sse = 0.0;
  for (size_t i = 0; i < source_points.size(); ++i) {
    const Eigen::Vector3d aligned =
        alignment.rotation * source_points[i] + alignment.translation;
    sse += (aligned - gt_points[i]).squaredNorm();
  }
  if (pair_count) {
    *pair_count = static_cast<int>(source_points.size());
  }
  if (rmse_m) {
    *rmse_m = std::sqrt(sse / static_cast<double>(source_points.size()));
  }
  return alignment;
}

gtsam::Pose3 transformPose(const TimedPose& pose,
                           const Alignment& alignment,
                           const Eigen::Vector3d& origin) {
  const Eigen::Vector3d p =
      alignment.rotation * pose.pose.translation() + alignment.translation - origin;
  const gtsam::Rot3 r(alignment.rotation * pose.pose.rotation().matrix());
  return gtsam::Pose3(r, gtsam::Point3(p));
}

gtsam::Pose3 translatePose(const gtsam::Pose3& pose,
                           const Eigen::Vector3d& offset) {
  return gtsam::Pose3(pose.rotation(),
                      gtsam::Point3(pose.translation() + offset));
}

Eigen::Matrix3d alignedTranslationCovariance(const TimedPose& pose,
                                             const Alignment& alignment) {
  const Eigen::Matrix3d covariance =
      pose.pose_covariance.block<3, 3>(3, 3);
  return alignment.rotation * covariance * alignment.rotation.transpose();
}

void drawCovarianceScalars(VIO::RosRerunVisualizer* visualizer,
                           const std::string& base,
                           const Eigen::Matrix3d& covariance) {
  const Eigen::Matrix3d sym = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(sym);
  if (eig.info() != Eigen::Success) {
    return;
  }
  const Eigen::Vector3d eigenvalues = eig.eigenvalues().cwiseMax(0.0);
  const double sigma_min_m = std::sqrt(eigenvalues.minCoeff());
  const double sigma_max_m = std::sqrt(eigenvalues.maxCoeff());
  const double sigma_mean_m =
      (std::sqrt(eigenvalues(0)) + std::sqrt(eigenvalues(1)) +
       std::sqrt(eigenvalues(2))) /
      3.0;

  visualizer->drawScalar(base + "/trace_m2", covariance.trace());
  visualizer->drawScalar(base + "/frobenius_m2", covariance.norm());
  visualizer->drawScalar(base + "/sigma_min_cm", sigma_min_m * 100.0);
  visualizer->drawScalar(base + "/sigma_mean_cm", sigma_mean_m * 100.0);
  visualizer->drawScalar(base + "/sigma_max_cm", sigma_max_m * 100.0);
  visualizer->drawScalar(base + "/sigma_x_cm",
                         std::sqrt(std::max(0.0, covariance(0, 0))) * 100.0);
  visualizer->drawScalar(base + "/sigma_y_cm",
                         std::sqrt(std::max(0.0, covariance(1, 1))) * 100.0);
  visualizer->drawScalar(base + "/sigma_z_cm",
                         std::sqrt(std::max(0.0, covariance(2, 2))) * 100.0);
}

void drawFullGroundTruth(VIO::RosRerunVisualizer* visualizer,
                         const Trajectory& gt,
                         const Eigen::Vector3d& origin,
                         const uint64_t start_nsec,
                         const std::string& entity_path =
                             "aligned/ground_truth/trajectory",
                         const Eigen::Vector4f& color =
                             Eigen::Vector4f(65.f, 140.f, 255.f, 220.f),
                         const double min_stamp_sec =
                             -std::numeric_limits<double>::infinity()) {
  std::vector<gtsam::Pose3> gt_poses;
  gt_poses.reserve(gt.poses.size());
  for (const TimedPose& pose : gt.poses) {
    if (pose.stamp_sec < min_stamp_sec) {
      continue;
    }
    const Eigen::Vector3d p = pose.pose.translation() - origin;
    gt_poses.emplace_back(pose.pose.rotation(), gtsam::Point3(p));
  }
  if (gt_poses.empty()) {
    return;
  }
  visualizer->setTimeNSec(start_nsec);
  visualizer->drawTrajectory(entity_path, gt_poses, color, 2.0f);
}

Eigen::Vector3d firstTranslationAtOrAfter(const Trajectory& trajectory,
                                          const double stamp_sec) {
  const auto iter = std::lower_bound(
      trajectory.poses.begin(),
      trajectory.poses.end(),
      stamp_sec,
      [](const TimedPose& pose, const double stamp) {
        return pose.stamp_sec < stamp;
      });
  if (iter == trajectory.poses.end()) {
    return trajectory.poses.back().pose.translation();
  }
  return iter->pose.translation();
}

struct PlaybackEvent {
  enum class Kind { kPose, kImage };

  double stamp_sec = 0.0;
  Kind kind = Kind::kPose;
  std::string name;
  TimedPose pose;
  Alignment alignment;
  Eigen::Vector4f color;
  TimedImage image;
  std::string entity_prefix;
  std::string covariance_metric_prefix;
};

struct RunArtifacts {
  std::string run_dir;
  Trajectory gt;
  Trajectory glim;
  Trajectory kimera;
  Alignment glim_alignment;
  Alignment kimera_alignment;
  int glim_pairs = 0;
  int kimera_pairs = 0;
  double glim_rmse_m = 0.0;
  double kimera_rmse_m = 0.0;
};

RunArtifacts loadRunArtifacts(const std::string& run_dir,
                              const double max_association_diff_sec) {
  if (run_dir.empty()) {
    throw std::runtime_error("Run directory is required");
  }

  const std::string tum_dir = run_dir + "/trajectories/tum";
  const std::string trajectory_dir = run_dir + "/trajectories";

  RunArtifacts run;
  run.run_dir = run_dir;
  run.gt = readTum(tum_dir + "/ground_truth.tum");
  run.glim = readOdometryCsv(trajectory_dir + "/glim_odometry.csv");
  run.kimera = readOdometryCsv(trajectory_dir + "/kimera_odometry.csv");
  run.glim_alignment = estimateAlignmentToGroundTruth(run.glim,
                                                       run.gt,
                                                       max_association_diff_sec,
                                                       &run.glim_pairs,
                                                       &run.glim_rmse_m);
  run.kimera_alignment =
      estimateAlignmentToGroundTruth(run.kimera,
                                     run.gt,
                                     max_association_diff_sec,
                                     &run.kimera_pairs,
                                     &run.kimera_rmse_m);
  return run;
}

void appendTrajectoryEvents(const Trajectory& trajectory,
                            const Alignment& alignment,
                            const std::string& name,
                            const std::string& entity_prefix,
                            const std::string& covariance_metric_prefix,
                            const Eigen::Vector4f& color,
                            std::vector<PlaybackEvent>* events) {
  for (const TimedPose& pose : trajectory.poses) {
    PlaybackEvent event;
    event.stamp_sec = pose.stamp_sec;
    event.kind = PlaybackEvent::Kind::kPose;
    event.name = name;
    event.pose = pose;
    event.alignment = alignment;
    event.color = color;
    event.entity_prefix = entity_prefix;
    event.covariance_metric_prefix = covariance_metric_prefix;
    events->push_back(event);
  }
}

std::vector<TimedImage> readCompressedImageTopic(const std::string& bag_path,
                                                 const std::string& topic,
                                                 const std::string& entity_path,
                                                 const double start_stamp_sec,
                                                 const double end_stamp_sec,
                                                 const double max_hz) {
  std::vector<TimedImage> images;
  if (bag_path.empty() || topic.empty() || entity_path.empty()) {
    return images;
  }
  if (!std::isfinite(start_stamp_sec) || !std::isfinite(end_stamp_sec) ||
      end_stamp_sec <= start_stamp_sec) {
    return images;
  }

  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  rosbag::View view(bag,
                    rosbag::TopicQuery(std::vector<std::string>{topic}),
                    ros::Time(start_stamp_sec),
                    ros::Time(end_stamp_sec));

  const double min_interval_sec =
      std::isfinite(max_hz) && max_hz > 0.0 ? 1.0 / max_hz : 0.0;
  double last_kept_stamp_sec = -std::numeric_limits<double>::infinity();
  for (const rosbag::MessageInstance& message : view) {
    sensor_msgs::CompressedImageConstPtr compressed =
        message.instantiate<sensor_msgs::CompressedImage>();
    if (!compressed) {
      continue;
    }
    double stamp_sec = compressed->header.stamp.toSec();
    if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
      stamp_sec = message.getTime().toSec();
    }
    if (!std::isfinite(stamp_sec) || stamp_sec < start_stamp_sec ||
        stamp_sec > end_stamp_sec) {
      continue;
    }
    if (stamp_sec - last_kept_stamp_sec < min_interval_sec) {
      continue;
    }

    const cv::Mat encoded(1,
                          static_cast<int>(compressed->data.size()),
                          CV_8UC1,
                          const_cast<uint8_t*>(compressed->data.data()));
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
      continue;
    }
    images.push_back({stamp_sec, entity_path, image});
    last_kept_stamp_sec = stamp_sec;
  }
  return images;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "artifact_rerun_visualizer_node");
  ros::NodeHandle private_nh("~");

  std::string run_dir;
  std::string rerun_host;
  std::string recording_id;
  double max_association_diff_sec = 0.1;
  double rate_scale = 1.0;
  double image_max_hz = 10.0;
  bool draw_ground_truth_static = true;
  bool image_replay_enable = false;
  bool comparison_replay_enable = false;
  bool comparison_start_anchor_enable = false;
  std::string image_bag_path;
  std::string left_image_topic;
  std::string left_image_entity_path;
  std::string right_image_topic;
  std::string right_image_entity_path;
  std::string cbs_off_run_dir;
  std::string cbs_on_run_dir;
  std::string comparison_entity_root;
  private_nh.param<std::string>("run_dir", run_dir, "");
  private_nh.param<std::string>(
      "rerun_host", rerun_host, "rerun+http://127.0.0.1:9876/proxy");
  private_nh.param<std::string>(
      "rerun_recording_id", recording_id, "glim_kimera_artifact_aligned");
  private_nh.param<double>(
      "max_association_diff_sec", max_association_diff_sec, 0.1);
  private_nh.param<double>("rate_scale", rate_scale, 3.0);
  private_nh.param<bool>(
      "draw_ground_truth_static", draw_ground_truth_static, true);
  private_nh.param<bool>("image_replay_enable", image_replay_enable, false);
  private_nh.param<std::string>("image_bag_path", image_bag_path, "");
  private_nh.param<std::string>(
      "left_image_topic", left_image_topic, "/Alpha/left_camera/compressed");
  private_nh.param<std::string>(
      "left_image_entity_path", left_image_entity_path, "video/left/image");
  private_nh.param<std::string>(
      "right_image_topic", right_image_topic, "");
  private_nh.param<std::string>(
      "right_image_entity_path", right_image_entity_path, "video/right/image");
  private_nh.param<double>("image_max_hz", image_max_hz, 10.0);
  private_nh.param<bool>(
      "comparison_replay_enable", comparison_replay_enable, false);
  private_nh.param<bool>(
      "comparison_start_anchor_enable", comparison_start_anchor_enable, false);
  private_nh.param<std::string>("cbs_off_run_dir", cbs_off_run_dir, "");
  private_nh.param<std::string>("cbs_on_run_dir", cbs_on_run_dir, "");
  private_nh.param<std::string>(
      "comparison_entity_root", comparison_entity_root, "comparison");

  if (!comparison_replay_enable && run_dir.empty()) {
    ROS_FATAL("run_dir parameter is required");
    return 1;
  }
  if (!std::isfinite(rate_scale) || rate_scale <= 0.0) {
    rate_scale = 1.0;
  }

  try {
    if (comparison_replay_enable) {
      const RunArtifacts cbs_off =
          loadRunArtifacts(cbs_off_run_dir, max_association_diff_sec);
      const RunArtifacts cbs_on =
          loadRunArtifacts(cbs_on_run_dir, max_association_diff_sec);

      double first_stamp = std::numeric_limits<double>::infinity();
      double last_stamp = 0.0;
      for (const auto* trajectory : {&cbs_off.glim,
                                     &cbs_off.kimera,
                                     &cbs_on.glim,
                                     &cbs_on.kimera}) {
        first_stamp = std::min(first_stamp, trajectory->poses.front().stamp_sec);
        last_stamp = std::max(last_stamp, trajectory->poses.back().stamp_sec);
      }

      double gt_diff = std::numeric_limits<double>::infinity();
      int origin_index = nearestPoseIndex(cbs_on.gt, first_stamp, &gt_diff);
      if (origin_index < 0) {
        origin_index = 0;
      }
      const Eigen::Vector3d origin =
          cbs_on.gt.poses[origin_index].pose.translation();

      VIO::RosRerunVisualizer visualizer("cbsms", recording_id, rerun_host);
      const uint64_t start_nsec = stampSecToNSec(first_stamp);
      visualizer.setTimeNSec(start_nsec);

      const std::string root =
          comparison_entity_root.empty() ? "comparison" : comparison_entity_root;
      visualizer.drawScalar(root + "/metadata/cbs_off_glim_alignment_pairs",
                            cbs_off.glim_pairs);
      visualizer.drawScalar(root + "/metadata/cbs_off_kimera_alignment_pairs",
                            cbs_off.kimera_pairs);
      visualizer.drawScalar(root + "/metadata/cbs_on_glim_alignment_pairs",
                            cbs_on.glim_pairs);
      visualizer.drawScalar(root + "/metadata/cbs_on_kimera_alignment_pairs",
                            cbs_on.kimera_pairs);
      visualizer.drawScalar(root + "/metadata/cbs_off_glim_alignment_rmse_m",
                            cbs_off.glim_rmse_m);
      visualizer.drawScalar(root + "/metadata/cbs_off_kimera_alignment_rmse_m",
                            cbs_off.kimera_rmse_m);
      visualizer.drawScalar(root + "/metadata/cbs_on_glim_alignment_rmse_m",
                            cbs_on.glim_rmse_m);
      visualizer.drawScalar(root + "/metadata/cbs_on_kimera_alignment_rmse_m",
                            cbs_on.kimera_rmse_m);

      if (draw_ground_truth_static) {
        const Eigen::Vector3d gt_origin =
            comparison_start_anchor_enable
                ? firstTranslationAtOrAfter(cbs_on.gt, first_stamp)
                : origin;
        drawFullGroundTruth(&visualizer,
                            cbs_on.gt,
                            gt_origin,
                            start_nsec,
                            root + "/ground_truth/trajectory",
                            Eigen::Vector4f(65.f, 140.f, 255.f, 235.f),
                            comparison_start_anchor_enable
                                ? first_stamp
                                : -std::numeric_limits<double>::infinity());
      }

      std::vector<PlaybackEvent> events;
      events.reserve(cbs_off.glim.poses.size() + cbs_off.kimera.poses.size() +
                     cbs_on.glim.poses.size() + cbs_on.kimera.poses.size());
      appendTrajectoryEvents(cbs_off.glim,
                             cbs_off.glim_alignment,
                             "cbs_off_glim",
                             root + "/cbs_off/glim",
                             "metrics/posterior_covariance/cbs_off/glim",
                             Eigen::Vector4f(255.f, 165.f, 0.f, 220.f),
                             &events);
      appendTrajectoryEvents(cbs_off.kimera,
                             cbs_off.kimera_alignment,
                             "cbs_off_kimera",
                             root + "/cbs_off/kimera",
                             "metrics/posterior_covariance/cbs_off/kimera",
                             Eigen::Vector4f(245.f, 70.f, 70.f, 220.f),
                             &events);
      appendTrajectoryEvents(cbs_on.glim,
                             cbs_on.glim_alignment,
                             "cbs_on_glim",
                             root + "/cbs_on/glim",
                             "metrics/posterior_covariance/cbs_on/glim",
                             Eigen::Vector4f(245.f, 220.f, 40.f, 235.f),
                             &events);
      appendTrajectoryEvents(cbs_on.kimera,
                             cbs_on.kimera_alignment,
                             "cbs_on_kimera",
                             root + "/cbs_on/kimera",
                             "metrics/posterior_covariance/cbs_on/kimera",
                             Eigen::Vector4f(40.f, 220.f, 80.f, 235.f),
                             &events);

      size_t image_count = 0u;
      if (image_replay_enable) {
        std::vector<TimedImage> left_images =
            readCompressedImageTopic(image_bag_path,
                                     left_image_topic,
                                     left_image_entity_path,
                                     first_stamp,
                                     last_stamp,
                                     image_max_hz);
        image_count += left_images.size();
        for (TimedImage& image : left_images) {
          PlaybackEvent event;
          event.stamp_sec = image.stamp_sec;
          event.kind = PlaybackEvent::Kind::kImage;
          event.image = std::move(image);
          events.push_back(std::move(event));
        }

        if (!right_image_topic.empty()) {
          std::vector<TimedImage> right_images =
              readCompressedImageTopic(image_bag_path,
                                       right_image_topic,
                                       right_image_entity_path,
                                       first_stamp,
                                       last_stamp,
                                       image_max_hz);
          image_count += right_images.size();
          for (TimedImage& image : right_images) {
            PlaybackEvent event;
            event.stamp_sec = image.stamp_sec;
            event.kind = PlaybackEvent::Kind::kImage;
            event.image = std::move(image);
            events.push_back(std::move(event));
          }
        }
      }

      std::sort(events.begin(), events.end(), [](const PlaybackEvent& a,
                                                 const PlaybackEvent& b) {
        return a.stamp_sec < b.stamp_sec;
      });

      std::unordered_map<std::string, std::vector<gtsam::Pose3>> paths;
      std::unordered_map<std::string, Eigen::Vector3d> start_anchor_offsets;
      double previous_stamp = events.front().stamp_sec;
      for (const PlaybackEvent& event : events) {
        if (!ros::ok()) {
          break;
        }
        const double sleep_sec =
            std::max(0.0, event.stamp_sec - previous_stamp) / rate_scale;
        if (sleep_sec > 0.0) {
          ros::Duration(sleep_sec).sleep();
        }
        previous_stamp = event.stamp_sec;

        visualizer.setTimeNSec(stampSecToNSec(event.stamp_sec));
        if (event.kind == PlaybackEvent::Kind::kImage) {
          visualizer.drawImage(event.image.entity_path, event.image.image, false);
          continue;
        }

        gtsam::Pose3 aligned_pose =
            transformPose(event.pose, event.alignment, origin);
        if (comparison_start_anchor_enable) {
          const auto offset_iter =
              start_anchor_offsets.find(event.entity_prefix);
          if (offset_iter == start_anchor_offsets.end()) {
            const Eigen::Vector3d offset = -aligned_pose.translation();
            start_anchor_offsets.emplace(event.entity_prefix, offset);
            aligned_pose = translatePose(aligned_pose, offset);
          } else {
            aligned_pose = translatePose(aligned_pose, offset_iter->second);
          }
        }
        std::vector<gtsam::Pose3>& path = paths[event.entity_prefix];
        path.push_back(aligned_pose);

        visualizer.drawTf(event.entity_prefix + "/base_link",
                          aligned_pose,
                          0.45f);
        if (path.size() > 1u) {
          visualizer.drawTrajectory(
              event.entity_prefix + "/trajectory", path, event.color, 1.9f);
        }
      }

      visualizer.setTimeNSec(stampSecToNSec(last_stamp));
      visualizer.drawScalar("video/metadata/image_count",
                            static_cast<double>(image_count));
      visualizer.drawScalar(root + "/metadata/start_anchor_enable",
                            comparison_start_anchor_enable ? 1.0 : 0.0);
      visualizer.drawScalar(root + "/metadata/replay_complete", 1.0);
      ROS_INFO_STREAM("Artifact comparison Rerun replay complete. recording_id='"
                      << recording_id << "', cbs_off_glim_poses="
                      << cbs_off.glim.poses.size()
                      << ", cbs_off_kimera_poses="
                      << cbs_off.kimera.poses.size()
                      << ", cbs_on_glim_poses=" << cbs_on.glim.poses.size()
                      << ", cbs_on_kimera_poses="
                      << cbs_on.kimera.poses.size()
                      << ", images=" << image_count << ".");
      return 0;
    }

    const std::string tum_dir = run_dir + "/trajectories/tum";
    const std::string trajectory_dir = run_dir + "/trajectories";

    const Trajectory gt = readTum(tum_dir + "/ground_truth.tum");
    const Trajectory glim = readOdometryCsv(trajectory_dir + "/glim_odometry.csv");
    const Trajectory kimera =
        readOdometryCsv(trajectory_dir + "/kimera_odometry.csv");

    int glim_pairs = 0;
    int kimera_pairs = 0;
    double glim_rmse_m = 0.0;
    double kimera_rmse_m = 0.0;
    const Alignment glim_alignment = estimateAlignmentToGroundTruth(
        glim, gt, max_association_diff_sec, &glim_pairs, &glim_rmse_m);
    const Alignment kimera_alignment = estimateAlignmentToGroundTruth(
        kimera, gt, max_association_diff_sec, &kimera_pairs, &kimera_rmse_m);

    double first_stamp = std::numeric_limits<double>::infinity();
    double last_stamp = 0.0;
    for (const auto* trajectory : {&glim, &kimera}) {
      first_stamp = std::min(first_stamp, trajectory->poses.front().stamp_sec);
      last_stamp = std::max(last_stamp, trajectory->poses.back().stamp_sec);
    }

    double gt_diff = std::numeric_limits<double>::infinity();
    int origin_index = nearestPoseIndex(gt, first_stamp, &gt_diff);
    if (origin_index < 0) {
      origin_index = 0;
    }
    const Eigen::Vector3d origin = gt.poses[origin_index].pose.translation();

    VIO::RosRerunVisualizer visualizer("cbsms", recording_id, rerun_host);
    const uint64_t start_nsec = stampSecToNSec(first_stamp);
    visualizer.setTimeNSec(start_nsec);
    visualizer.drawScalar("aligned/metadata/glim_alignment_pairs", glim_pairs);
    visualizer.drawScalar("aligned/metadata/kimera_alignment_pairs", kimera_pairs);
    visualizer.drawScalar("aligned/metadata/glim_alignment_rmse_m", glim_rmse_m);
    visualizer.drawScalar("aligned/metadata/kimera_alignment_rmse_m",
                          kimera_rmse_m);

    if (draw_ground_truth_static) {
      drawFullGroundTruth(&visualizer, gt, origin, start_nsec);
    }

    std::vector<PlaybackEvent> events;
    events.reserve(glim.poses.size() + kimera.poses.size());
    for (const TimedPose& pose : glim.poses) {
      PlaybackEvent event;
      event.stamp_sec = pose.stamp_sec;
      event.kind = PlaybackEvent::Kind::kPose;
      event.name = "glim";
      event.pose = pose;
      event.alignment = glim_alignment;
      event.color = Eigen::Vector4f(245.f, 180.f, 20.f, 220.f);
      events.push_back(event);
    }
    for (const TimedPose& pose : kimera.poses) {
      PlaybackEvent event;
      event.stamp_sec = pose.stamp_sec;
      event.kind = PlaybackEvent::Kind::kPose;
      event.name = "kimera";
      event.pose = pose;
      event.alignment = kimera_alignment;
      event.color = Eigen::Vector4f(40.f, 220.f, 80.f, 220.f);
      events.push_back(event);
    }

    size_t image_count = 0u;
    if (image_replay_enable) {
      std::vector<TimedImage> left_images =
          readCompressedImageTopic(image_bag_path,
                                   left_image_topic,
                                   left_image_entity_path,
                                   first_stamp,
                                   last_stamp,
                                   image_max_hz);
      image_count += left_images.size();
      for (TimedImage& image : left_images) {
        PlaybackEvent event;
        event.stamp_sec = image.stamp_sec;
        event.kind = PlaybackEvent::Kind::kImage;
        event.image = std::move(image);
        events.push_back(std::move(event));
      }

      if (!right_image_topic.empty()) {
        std::vector<TimedImage> right_images =
            readCompressedImageTopic(image_bag_path,
                                     right_image_topic,
                                     right_image_entity_path,
                                     first_stamp,
                                     last_stamp,
                                     image_max_hz);
        image_count += right_images.size();
        for (TimedImage& image : right_images) {
          PlaybackEvent event;
          event.stamp_sec = image.stamp_sec;
          event.kind = PlaybackEvent::Kind::kImage;
          event.image = std::move(image);
          events.push_back(std::move(event));
        }
      }
    }
    std::sort(events.begin(), events.end(), [](const PlaybackEvent& a,
                                               const PlaybackEvent& b) {
      return a.stamp_sec < b.stamp_sec;
    });

    std::vector<gtsam::Pose3> glim_path;
    std::vector<gtsam::Pose3> kimera_path;
    double previous_stamp = events.front().stamp_sec;
    for (const PlaybackEvent& event : events) {
      if (!ros::ok()) {
        break;
      }
      const double sleep_sec =
          std::max(0.0, event.stamp_sec - previous_stamp) / rate_scale;
      if (sleep_sec > 0.0) {
        ros::Duration(sleep_sec).sleep();
      }
      previous_stamp = event.stamp_sec;

      visualizer.setTimeNSec(stampSecToNSec(event.stamp_sec));
      if (event.kind == PlaybackEvent::Kind::kImage) {
        visualizer.drawImage(event.image.entity_path, event.image.image, false);
        continue;
      }

      const gtsam::Pose3 aligned_pose =
          transformPose(event.pose, event.alignment, origin);
      std::vector<gtsam::Pose3>& path =
          event.name == "glim" ? glim_path : kimera_path;
      path.push_back(aligned_pose);

      const std::string prefix = "aligned/" + event.name;
      visualizer.drawTf(prefix + "/base_link", aligned_pose, 0.5f);
      if (path.size() > 1u) {
        visualizer.drawTrajectory(
            prefix + "/trajectory", path, event.color, 1.75f);
      }
      if (event.pose.has_covariance) {
        const Eigen::Matrix3d covariance =
            alignedTranslationCovariance(event.pose, event.alignment);
        if (covariance.allFinite() &&
            covariance.norm() > std::numeric_limits<double>::epsilon()) {
          visualizer.drawUncertainty(prefix + "/current_pose/uncertainty",
                                     aligned_pose,
                                     covariance,
                                     event.color,
                                     1.25f);
          drawCovarianceScalars(&visualizer,
                                "metrics/posterior_covariance/" + event.name,
                                covariance);
        }
      }
    }

    visualizer.setTimeNSec(stampSecToNSec(last_stamp));
    visualizer.drawScalar("video/metadata/image_count",
                          static_cast<double>(image_count));
    visualizer.drawScalar("aligned/metadata/replay_complete", 1.0);
    ROS_INFO_STREAM("Artifact aligned Rerun replay complete. recording_id='"
                    << recording_id << "', glim_poses=" << glim.poses.size()
                    << ", kimera_poses=" << kimera.poses.size()
                    << ", glim_pairs=" << glim_pairs
                    << ", kimera_pairs=" << kimera_pairs
                    << ", images=" << image_count << ".");
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Artifact aligned Rerun replay failed: " << e.what());
    return 1;
  }

  return 0;
}
