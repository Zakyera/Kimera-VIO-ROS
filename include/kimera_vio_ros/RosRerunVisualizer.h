/**
 * @file   RosRerunVisualizer.h
 * @brief  Small wrapper that keeps aria_viz logging macros out of Kimera code.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <opencv2/core/mat.hpp>

namespace VIO {

struct RerunGraphNode {
  std::string id;
  std::string label;
  float x = 0.0f;
  float y = 0.0f;
  uint8_t red = 255u;
  uint8_t green = 255u;
  uint8_t blue = 255u;
  uint8_t alpha = 255u;
  float radius_ui_points = 4.0f;
  bool show_label = false;
};

struct RerunLineStrip3D {
  std::vector<gtsam::Point3> points;
  std::string label;
};

class RosRerunVisualizer {
 public:
  RosRerunVisualizer(const std::string& app_id,
                     const std::string& recording_id,
                     const std::string& host);
  ~RosRerunVisualizer();

  void setTimeNSec(size_t timestamp);
  void drawTf(const std::string& entity_path,
              const gtsam::Pose3& pose,
              float axis_length);
  void drawScalar(const std::string& entity_path, double value);
  void clearEntity(const std::string& entity_path);
  void drawGraph(
      const std::string& entity_path,
      const std::vector<RerunGraphNode>& nodes,
      const std::vector<std::pair<std::string, std::string>>& edges,
      bool show_labels = false);
  void drawPoints(const std::string& entity_path,
                  const std::vector<gtsam::Point3>& points,
                  const Eigen::Vector4f& rgba,
                  float radius);
  void drawLabeledPoints(const std::string& entity_path,
                         const std::vector<gtsam::Point3>& points,
                         const std::vector<std::string>& labels,
                         const Eigen::Vector4f& rgba,
                         float radius_ui_points,
                         bool show_labels = false);
  void drawLineStrips(const std::string& entity_path,
                      const std::vector<RerunLineStrip3D>& strips,
                      const Eigen::Vector4f& rgba,
                      float line_width_ui_points,
                      bool show_labels = false);
  void drawTrajectory(const std::string& entity_path,
                      const std::vector<gtsam::Pose3>& poses,
                      const Eigen::Vector4f& rgba,
                      float line_width);
  void drawFactors(const std::string& entity_path,
                   const gtsam::NonlinearFactorGraph& factors,
                   const gtsam::Values& values,
                   const Eigen::Vector4f& rgba,
                   float line_width);
  void drawUncertainty(const std::string& entity_path,
                       const gtsam::Pose3& pose,
                       const Eigen::Matrix3d& covariance,
                       const Eigen::Vector4f& rgba,
                       float line_width);
  void drawImage(const std::string& entity_path,
                 const cv::Mat& image,
                 bool is_static = false);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace VIO
