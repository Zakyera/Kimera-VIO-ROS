/* @file   KimeraVioRos.cpp
 * @brief  ROS Wrapper for Kimera-VIO
 * @author Antoni Rosinol
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// zy Step 1_a
// Adds message and string types needed by the Kimera-LIORF external pose bridge.
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64MultiArray.h>
#include <string>
#include <atomic>
#include <cstdint>


#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/utils/Macros.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"
#include "kimera_vio_ros/RosDisplay.h"
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
  
    // zy Step 2_a
  // Publishes Kimera external pose belief as ROS odometry for LIORF/CBS consumers.
  void publishExternalPoseBelief(
      const VioBackend::ExternalPoseBelief& belief);

  // zy Step 2_b
  // Receives external ROS prior and forwards it to Kimera backend prior queue.
  void externalPosePriorCallback(const nav_msgs::Odometry::ConstPtr& msg);

  // Publishes receiver pacing watermark used by external sender flow-control.
  void publishExternalPriorReceiverWatermark(const ros::TimerEvent& event);

  // zy Step 2_c
  // Normalizes source tags to stable lowercase names for source filtering/mapping.
  std::string normalizeExternalSourceTag(const std::string& source) const;


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

  // zy Step 1_b
  // Stores bridge runtime config and ROS handles so topic wiring is explicit and launch-driven.
  bool enable_external_pose_bridge_ = false;
  // zy Step 4_b
  // Keeps internal default consistent with launch-level LIORF-compatible handshake.
  std::string external_pose_belief_topic_ = "/liorf/cbs/external_pose_prior";
  // zy Step 5_a
  // Keeps fallback receive-topic aligned with LIORF's default belief publisher.
  std::string external_pose_prior_topic_ = "/liorf/cbs/external_pose_belief";
  std::string external_pose_belief_source_ = "kimera";
  std::string external_prior_default_source_ = "liorf";
  std::string external_exchange_frame_id_ = "odom";
  bool enable_external_receiver_watermark_ = true;
  std::string external_receiver_watermark_topic_ = "/kimera/cbs/receiver_watermark";
  double external_receiver_watermark_period_sec_ = 0.1;
  uint32_t external_pose_belief_seq_counter_ = 0;
  ros::Publisher pub_external_pose_belief_;
  ros::Publisher pub_external_receiver_watermark_;
  ros::Subscriber sub_external_pose_prior_;
  ros::Timer external_receiver_watermark_timer_;


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
};

}  // namespace VIO
