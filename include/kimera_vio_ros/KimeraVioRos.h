/* @file   KimeraVioRos.cpp
 * @brief  ROS Wrapper for Kimera-VIO
 * @author Antoni Rosinol
 */

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Trigger.h>

#include <atomic>
#include <deque>
#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/utils/Macros.h>
#include <mutex>
#include <string>
#include <vector>
#include <tf/transform_listener.h>

#include <liorf/pose_odom_belief_array.h>
#include "kimera_vio_ros/RosDataProviderInterface.h"
#include "kimera_vio_ros/RosDisplay.h"
#include "kimera_vio_ros/RosRerunVisualizer.h"
#include "kimera_vio_ros/RosVisualizer.h"
#include "kimera_vio_ros/RosLoopClosureVisualizer.h"
#include "kimera_vio_ros/LcdRegistrationServer.h"

namespace VIO {

class KimeraVioRos {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(KimeraVioRos);
  KIMERA_POINTER_TYPEDEFS(KimeraVioRos);

  KimeraVioRos();
  virtual ~KimeraVioRos();

 public:
  bool runKimeraVio();

 protected:
  bool spin();

  VIO::RosDataProviderInterface::UniquePtr createDataProvider(
      const VioParams& vio_params);

  void connectVIO();

  void bufferExternalOdometryBeliefs(
      const std::vector<ExternalOdometryBelief>& beliefs);

  void flushExternalOdometryBeliefsToPipeline();

  void initializeHeadlessCbsBeliefBridge();

  void initializeHeadlessOdometryPublisher();

  void initializeHeadlessLandmarksPublisher();

  void initializeHeadlessRerunVisualizer();

  void publishHeadlessBackendOutput(const BackendOutput::ConstPtr& output);

  void publishHeadlessOdometry(const BackendOutput::ConstPtr& output);

  void publishHeadlessLandmarks(const BackendOutput::ConstPtr& output);

  void publishHeadlessRerunBackendOutput(
      const BackendOutput::ConstPtr& output);

  void publishHeadlessOdometryBelief(const BackendOutput::ConstPtr& output);

  void poseOdomBeliefInCallback(
      const liorf::pose_odom_belief_arrayConstPtr& msg);

  double incomingOdomBeliefGateStampSec(
      const liorf::pose_odom_belief& belief) const;

  void initializeIncomingOdomBeliefReceiveGate(
      const liorf::pose_odom_belief_array& msg);

  bool isIncomingOdomBeliefBeforeReceiveGate(
      const liorf::pose_odom_belief& belief) const;

  void cbsBeliefReceiveClockCallback(const rosgraph_msgs::ClockConstPtr& msg);

  bool lookupExternalPoseFrameTransform(gtsam::Pose3* base_T_external,
                                        gtsam::Pose3* external_T_base);

  /**
   * @brief restartKimeraVio Callback for the rosservice to restart the pipeline
   * @param request
   * @param response
   * @return
   */
  bool restartKimeraVio(std_srvs::Trigger::Request& request,
                        std_srvs::Trigger::Response& response);

 protected:
  //! ROS
  ros::NodeHandle nh_private_;

  //! VIO
  VioParams::Ptr vio_params_;
  Pipeline::UniquePtr vio_pipeline_;

  //! External LCD service manager
  bool use_lcd_registration_server_;
  std::unique_ptr<LcdRegistrationServer> lcd_registration_server_;

  //! Data provider
  RosDataProviderInterface::UniquePtr data_provider_;

  //! Visualization
  bool use_rviz_;  //! whether we want to use rviz for visualization or opencv.
  RosDisplay::UniquePtr ros_display_;
  RosVisualizer::UniquePtr ros_visualizer_;
  RosLoopClosureVisualizer::Ptr ros_lcd_visualizer_;

  //! ROS Services
  ros::ServiceServer restart_vio_pipeline_srv_;
  std::atomic_bool restart_vio_pipeline_;

  std::mutex external_beliefs_mutex_;
  std::deque<ExternalOdometryBelief> pending_external_odom_beliefs_;
  size_t external_beliefs_queue_limit_ = 800u;

  bool headless_cbs_belief_bridge_enable_ = false;
  bool headless_odometry_publish_enable_ = false;
  bool headless_landmarks_publish_enable_ = false;
  int headless_landmarks_max_points_ = 3000;
  bool headless_rerun_visualizer_enable_ = false;
  bool headless_rerun_scalar_metrics_enable_ = false;
  bool headless_rerun_geometry_enable_ = true;
  bool headless_rerun_factor_graph_enable_ = true;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string headless_rerun_recording_id_;
  std::string headless_rerun_host_;
  std::string cbs_odom_belief_in_topic_;
  std::string cbs_odom_belief_out_topic_;
  std::string cbs_external_pose_frame_id_;
  double cbs_belief_receive_start_delay_sec_ = 0.0;
  bool cbs_belief_receive_gate_reference_set_ = false;
  double cbs_belief_receive_gate_reference_stamp_sec_ = 0.0;
  std::atomic<size_t> cbs_beliefs_receive_gate_dropped_per_rerun_frame_{0u};
  uint8_t cbs_agent_id_ = static_cast<uint8_t>('k');
  std::unique_ptr<RosRerunVisualizer> headless_rerun_visualizer_;
  std::vector<gtsam::Pose3> headless_rerun_trajectory_;
  int64_t headless_rerun_last_kf_id_ = -1;
  ros::Publisher headless_odometry_pub_;
  ros::Publisher headless_landmarks_pub_;
  ros::Publisher pose_odom_belief_out_pub_;
  ros::Subscriber pose_odom_belief_in_sub_;
  ros::Subscriber cbs_belief_receive_clock_sub_;
  tf::TransformListener tf_listener_;
};

}  // namespace VIO
