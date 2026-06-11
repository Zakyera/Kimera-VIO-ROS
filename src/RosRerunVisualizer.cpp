#include "kimera_vio_ros/RosRerunVisualizer.h"

#include <aria_viz/visualizer_rerun.h>

namespace VIO {

class RosRerunVisualizer::Impl {
 public:
  Impl(const std::string& app_id,
       const std::string& recording_id,
       const std::string& host)
      : visualizer_(aria::viz::VisualizerRerun::Params(
            app_id, recording_id, host)) {}

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

void RosRerunVisualizer::drawPoints(
    const std::string& entity_path,
    const std::vector<gtsam::Point3>& points,
    const Eigen::Vector4f& rgba,
    float radius) {
  impl_->visualizer_.drawPoints(entity_path, points, rgba, radius);
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

}  // namespace VIO
