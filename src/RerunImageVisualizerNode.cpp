/* -----------------------------------------------------------------------------
 * Copyright 2026.
 * ----------------------------------------------------------------------------*/

#include "kimera_vio_ros/RosRerunVisualizer.h"

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/imgcodecs.hpp>

namespace VIO {
namespace {

uint64_t stampToNSec(const ros::Time& stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ull +
         static_cast<uint64_t>(stamp.nsec);
}

std::string makeRecordingId() {
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  localtime_r(&now, &local_time);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
  return std::string("rerun_image_visualizer_") + buffer;
}

class RerunImageVisualizer {
 public:
  RerunImageVisualizer() : private_nh_("~") {
    private_nh_.param<std::string>(
        "image_topic", image_topic_, "/camera/color/image_raw/compressed");
    private_nh_.param<std::string>(
        "image_entity_path", image_entity_path_, "video/rgb/image");
    private_nh_.param<double>("image_max_hz", image_max_hz_, 5.0);
    private_nh_.param<int>("image_queue_size", image_queue_size_, 30);
    private_nh_.param<std::string>("rerun_recording_id", recording_id_, "");
    private_nh_.param<std::string>(
        "rerun_host", rerun_host_, "rerun+http://127.0.0.1:9876/proxy");

    image_queue_size_ = std::max(1, image_queue_size_);
    if (recording_id_.empty()) {
      ros::param::param<std::string>(
          "/cbsms/rerun_recording_id", recording_id_, "");
    }
    if (recording_id_.empty()) {
      recording_id_ = makeRecordingId();
    }

    visualizer_ = std::make_unique<RosRerunVisualizer>(
        "cbsms", recording_id_, rerun_host_);
    image_sub_ = nh_.subscribe<sensor_msgs::CompressedImage>(
        image_topic_,
        image_queue_size_,
        &RerunImageVisualizer::imageCallback,
        this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("Dedicated Rerun image visualizer enabled. recording_id='"
                    << recording_id_ << "', host='" << rerun_host_
                    << "', image_topic='" << image_topic_
                    << "', image_entity_path='" << image_entity_path_
                    << "', image_max_hz=" << image_max_hz_
                    << ", image_queue_size=" << image_queue_size_ << ".");
  }

 private:
  void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    if (!msg || msg->data.empty()) {
      return;
    }

    const double stamp_sec = msg->header.stamp.toSec();
    if (std::isfinite(last_published_stamp_sec_) &&
        stamp_sec <= last_published_stamp_sec_) {
      return;
    }
    if (std::isfinite(image_max_hz_) && image_max_hz_ > 0.0 &&
        std::isfinite(last_published_stamp_sec_) &&
        stamp_sec - last_published_stamp_sec_ <
            (1.0 / image_max_hz_) - 1e-6) {
      return;
    }

    const std::vector<uint8_t> encoded(msg->data.begin(), msg->data.end());
    const std::string media_type =
        msg->format.find("png") != std::string::npos ? "image/png"
                                                      : "image/jpeg";

    const double interval_sec =
        std::isfinite(last_published_stamp_sec_)
            ? stamp_sec - last_published_stamp_sec_
            : std::numeric_limits<double>::quiet_NaN();
    visualizer_->setTimeNSec(stampToNSec(msg->header.stamp));
    visualizer_->drawEncodedImage(image_entity_path_, encoded, media_type);
    ++published_count_;
    visualizer_->drawScalar("video/metadata/live_image_count",
                            static_cast<double>(published_count_));
    if (std::isfinite(interval_sec)) {
      max_published_interval_sec_ =
          std::max(max_published_interval_sec_, interval_sec);
      visualizer_->drawScalar("video/metadata/live_image_interval_sec",
                              interval_sec);
    }
    last_published_stamp_sec_ = stamp_sec;

    if (published_count_ % 25u == 0u) {
      ROS_INFO_STREAM("RERUN_IMAGE_TIMING_ROW,"
                      << published_count_ << "," << std::setprecision(16)
                      << stamp_sec << "," << interval_sec << ","
                      << max_published_interval_sec_);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber image_sub_;
  std::unique_ptr<RosRerunVisualizer> visualizer_;

  std::string image_topic_;
  std::string image_entity_path_;
  std::string recording_id_;
  std::string rerun_host_;
  double image_max_hz_ = 5.0;
  double last_published_stamp_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  double max_published_interval_sec_ = 0.0;
  int image_queue_size_ = 30;
  size_t published_count_ = 0u;
};

}  // namespace
}  // namespace VIO

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "rerun_image_visualizer_node");
  VIO::RerunImageVisualizer visualizer;
  ros::spin();
  return 0;
}
