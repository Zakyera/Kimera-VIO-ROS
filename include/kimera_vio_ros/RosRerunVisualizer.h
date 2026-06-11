/**
 * @file   RosRerunVisualizer.h
 * @brief  Small wrapper that keeps aria_viz logging macros out of Kimera code.
 */

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <opencv2/core/mat.hpp>

namespace VIO {

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
  void drawPoints(const std::string& entity_path,
                  const std::vector<gtsam::Point3>& points,
                  const Eigen::Vector4f& rgba,
                  float radius);
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
