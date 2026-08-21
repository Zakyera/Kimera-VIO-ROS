#include "kimera_vio_ros/RosRerunVisualizer.h"

#include <algorithm>

#include <aria_viz/visualizer_rerun.h>
#include <rerun.hpp>

namespace VIO {
namespace {

uint8_t rerunColorChannel(const float value) {
  return static_cast<uint8_t>(std::clamp(value, 0.0f, 255.0f));
}

rerun::components::Color rerunColor(const Eigen::Vector4f& rgba) {
  return rerun::components::Color(rerunColorChannel(rgba.x()),
                                  rerunColorChannel(rgba.y()),
                                  rerunColorChannel(rgba.z()),
                                  rerunColorChannel(rgba.w()));
}

}  // namespace

class RosRerunVisualizer::Impl {
 public:
  Impl(const std::string& app_id,
       const std::string& recording_id,
       const std::string& host)
      : visualizer_(aria::viz::VisualizerRerun::Params(
            app_id, recording_id, host)) {
    visualizer_.rec()->send_recording_name(recording_id);
  }

  aria::viz::VisualizerRerun visualizer_;
};

RosRerunVisualizer::RosRerunVisualizer(const std::string& app_id,
                                       const std::string& recording_id,
                                       const std::string& host)
    : impl_(std::make_unique<Impl>(app_id, recording_id, host)) {}

RosRerunVisualizer::~RosRerunVisualizer() = default;

void RosRerunVisualizer::setTimeNSec(size_t timestamp) {
  impl_->visualizer_.setTimeNSec(timestamp);
}

void RosRerunVisualizer::drawTf(const std::string& entity_path,
                                const gtsam::Pose3& pose,
                                float axis_length) {
  impl_->visualizer_.drawTf(entity_path, pose, axis_length);
}

void RosRerunVisualizer::drawScalar(const std::string& entity_path,
                                    double value) {
  impl_->visualizer_.drawScalar(entity_path, value);
}

void RosRerunVisualizer::clearEntity(const std::string& entity_path) {
  impl_->visualizer_.rec()->log(entity_path, rerun::Clear(false));
}

void RosRerunVisualizer::drawGraph(
    const std::string& entity_path,
    const std::vector<RerunGraphNode>& nodes,
    const std::vector<std::pair<std::string, std::string>>& edges,
    const bool show_labels) {
  std::vector<rerun::components::GraphNode> node_ids;
  std::vector<rerun::components::Position2D> positions;
  std::vector<rerun::components::Color> colors;
  std::vector<rerun::components::Text> labels;
  std::vector<rerun::components::Radius> radii;
  std::vector<rerun::components::ShowLabels> label_visibility;
  node_ids.reserve(nodes.size());
  positions.reserve(nodes.size());
  colors.reserve(nodes.size());
  labels.reserve(nodes.size());
  radii.reserve(nodes.size());
  label_visibility.reserve(nodes.size());

  for (const auto& node : nodes) {
    node_ids.emplace_back(node.id);
    positions.emplace_back(node.x, node.y);
    colors.emplace_back(node.red, node.green, node.blue, node.alpha);
    labels.emplace_back(node.label);
    radii.emplace_back(
        rerun::components::Radius::ui_points(node.radius_ui_points));
    label_visibility.emplace_back(show_labels || node.show_label);
  }

  std::vector<rerun::components::GraphEdge> graph_edges;
  graph_edges.reserve(edges.size());
  for (const auto& edge : edges) {
    graph_edges.emplace_back(edge.first, edge.second);
  }

  impl_->visualizer_.rec()->log(
      entity_path,
      rerun::GraphNodes(node_ids)
          .with_positions(positions)
          .with_colors(colors)
          .with_labels(labels)
          .with_many_show_labels(label_visibility)
          .with_radii(radii),
      rerun::GraphEdges(graph_edges));
}

void RosRerunVisualizer::drawPoints(
    const std::string& entity_path,
    const std::vector<gtsam::Point3>& points,
    const Eigen::Vector4f& rgba,
    float radius) {
  impl_->visualizer_.drawPoints(entity_path, points, rgba, radius);
}

void RosRerunVisualizer::drawLabeledPoints(
    const std::string& entity_path,
    const std::vector<gtsam::Point3>& points,
    const std::vector<std::string>& labels,
    const Eigen::Vector4f& rgba,
    const float radius_ui_points,
    const bool show_labels) {
  if (points.empty()) {
    clearEntity(entity_path);
    return;
  }

  std::vector<rerun::components::Position3D> positions;
  positions.reserve(points.size());
  for (const auto& point : points) {
    positions.emplace_back(static_cast<float>(point.x()),
                           static_cast<float>(point.y()),
                           static_cast<float>(point.z()));
  }

  std::vector<rerun::components::Text> rerun_labels;
  rerun_labels.reserve(labels.size());
  for (const auto& label : labels) {
    rerun_labels.emplace_back(label);
  }

  auto archetype =
      rerun::Points3D(positions)
          .with_colors(rerunColor(rgba))
          .with_radii(
              rerun::components::Radius::ui_points(radius_ui_points))
          .with_show_labels(show_labels);
  if (rerun_labels.size() == positions.size()) {
    archetype = std::move(archetype).with_labels(rerun_labels);
  }
  impl_->visualizer_.rec()->log(entity_path, archetype);
}

void RosRerunVisualizer::drawLineStrips(
    const std::string& entity_path,
    const std::vector<RerunLineStrip3D>& strips,
    const Eigen::Vector4f& rgba,
    const float line_width_ui_points,
    const bool show_labels) {
  // rerun::components::LineStrip3D keeps a view of the supplied point
  // collection until rec()->log() serializes it.  Building it from a local
  // vector inside the loop leaves every component pointing at freed storage.
  // Keep all point buffers alive until after the log call.
  std::vector<std::vector<rerun::datatypes::Vec3D>> point_storage;
  std::vector<rerun::components::LineStrip3D> rerun_strips;
  std::vector<rerun::components::Text> rerun_labels;
  point_storage.reserve(strips.size());
  rerun_strips.reserve(strips.size());
  rerun_labels.reserve(strips.size());

  for (const auto& strip : strips) {
    if (strip.points.size() < 2u) {
      continue;
    }
    auto& points = point_storage.emplace_back();
    points.reserve(strip.points.size());
    for (const auto& point : strip.points) {
      points.emplace_back(static_cast<float>(point.x()),
                          static_cast<float>(point.y()),
                          static_cast<float>(point.z()));
    }
    rerun_labels.emplace_back(strip.label);
  }

  for (const auto& points : point_storage) {
    rerun_strips.emplace_back(points);
  }

  if (rerun_strips.empty()) {
    clearEntity(entity_path);
    return;
  }

  impl_->visualizer_.rec()->log(
      entity_path,
      rerun::LineStrips3D(rerun_strips)
          .with_colors(rerunColor(rgba))
          .with_radii(rerun::components::Radius::ui_points(
              line_width_ui_points))
          .with_labels(rerun_labels)
          .with_show_labels(show_labels));
}

void RosRerunVisualizer::drawTrajectory(
    const std::string& entity_path,
    const std::vector<gtsam::Pose3>& poses,
    const Eigen::Vector4f& rgba,
    float line_width) {
  impl_->visualizer_.drawTrajectory(entity_path, poses, rgba, line_width);
}

void RosRerunVisualizer::drawFactors(
    const std::string& entity_path,
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values,
    const Eigen::Vector4f& rgba,
    float line_width) {
  impl_->visualizer_.drawFactors(
      entity_path, factors, values, rgba, line_width, false, true);
}

void RosRerunVisualizer::drawUncertainty(
    const std::string& entity_path,
    const gtsam::Pose3& pose,
    const Eigen::Matrix3d& covariance,
    const Eigen::Vector4f& rgba,
    float line_width) {
  impl_->visualizer_.drawUncertainty(
      entity_path, pose, covariance, rgba, line_width);
}

void RosRerunVisualizer::drawImage(const std::string& entity_path,
                                   const cv::Mat& image,
                                   const bool is_static) {
  impl_->visualizer_.drawImage(entity_path, image, is_static);
}

void RosRerunVisualizer::drawEncodedImage(
    const std::string& entity_path,
    const std::vector<uint8_t>& image,
    const std::string& media_type) {
  impl_->visualizer_.rec()->log(
      entity_path,
      rerun::EncodedImage::from_bytes(
          image, rerun::components::MediaType(media_type)));
}

}  // namespace VIO
