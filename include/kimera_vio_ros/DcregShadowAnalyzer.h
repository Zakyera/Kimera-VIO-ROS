#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>

#include <liorf/DcregBeliefShadowAnalysisArray.h>
#include <liorf/DcregEdgeHealthMetadataArray.h>
#include <liorf/pose_odom_belief_array.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera_vio_ros/DcregShadowAnalysis.h"
#include "kimera_vio_ros/NonBlockingSpscQueue.h"

namespace VIO::dcreg_shadow {

bool loadConfig(const ros::NodeHandle& private_node,
                const std::string& active_belief_topic,
                Config* config,
                bool* section_present,
                std::string* error);

class Analyzer {
 public:
  struct Statistics {
    std::uint64_t belief_arrays_received = 0u;
    std::uint64_t metadata_arrays_received = 0u;
    std::uint64_t receiver_observations_received = 0u;
    std::uint64_t shadow_arrays_published = 0u;
    std::uint64_t shadow_entries_published = 0u;
    std::uint64_t dropped_belief_arrays = 0u;
    std::uint64_t dropped_metadata_arrays = 0u;
    std::uint64_t dropped_receiver_observations = 0u;
    std::uint64_t dropped_shadow_arrays = 0u;
    std::uint64_t orphan_beliefs = 0u;
    std::uint64_t orphan_metadata = 0u;
    std::uint64_t orphan_receiver_observations = 0u;
    std::uint64_t digest_mismatches = 0u;
    std::uint64_t exact_duplicate_metadata = 0u;
    std::uint64_t conflicting_duplicate_metadata = 0u;
    std::uint64_t worker_failures = 0u;
    std::size_t maximum_belief_queue_depth = 0u;
    std::size_t maximum_metadata_queue_depth = 0u;
    std::size_t maximum_receiver_queue_depth = 0u;
    std::size_t maximum_pending_beliefs = 0u;
    std::size_t maximum_pending_metadata = 0u;
    std::size_t maximum_pending_receiver_observations = 0u;
    std::uint64_t producer_waits = 0u;
    std::uint64_t belief_enqueue_time_ns = 0u;
    std::uint64_t receiver_enqueue_time_ns = 0u;
    std::uint64_t maximum_belief_enqueue_time_ns = 0u;
    std::uint64_t maximum_receiver_enqueue_time_ns = 0u;
    std::uint64_t worker_time_ns = 0u;
    std::uint64_t association_time_ns = 0u;
    std::uint64_t residual_time_ns = 0u;
    std::uint64_t published_bytes = 0u;
  };

  Analyzer(ros::NodeHandle node, Config config);
  ~Analyzer();

  Analyzer(const Analyzer&) = delete;
  Analyzer& operator=(const Analyzer&) = delete;

  std::uint64_t nextBeliefReceiptSequence();
  bool tryEnqueueBeliefArray(
      const liorf::pose_odom_belief_array& message,
      std::uint64_t receipt_sequence);
  bool tryEnqueueReceiverObservation(
      const ExternalBeliefMatchDiagnostic& observation);

  const std::string& receiverSessionUuid() const {
    return receiver_session_uuid_;
  }
  Statistics statistics() const;
  void stop();

 private:
  struct MetadataObservation;
  struct ReceiverObservation;
  struct PendingMatchedArray;
  struct OutputEvent;

  using BeliefEventPtr = std::shared_ptr<const BeliefArrayObservation>;
  using MetadataEventPtr = std::shared_ptr<const MetadataObservation>;
  using ReceiverEventPtr = std::shared_ptr<const ReceiverObservation>;
  using OutputEventPtr = std::shared_ptr<const OutputEvent>;
  using ReceiptKey = std::pair<std::uint64_t, std::uint32_t>;
  using MetadataKey = std::pair<std::string, std::uint64_t>;

  void metadataCallback(
      const liorf::DcregEdgeHealthMetadataArrayConstPtr& message);
  void workerLoop();
  void drainInputQueues();
  void matchBeliefsAndMetadata();
  void finalizeReadyArrays(bool force_shutdown);
  void expireOrphans(bool force_shutdown);
  void queueBeliefWithoutMetadata(const BeliefEventPtr& belief,
                                  std::uint32_t reason_mask);
  void publishOutputQueue();
  void queueOutput(liorf::DcregBeliefShadowAnalysisArray message);
  void writeCsvHeader();
  void writeCsv(const liorf::DcregBeliefShadowAnalysisArray& message);

  Config config_;
  std::string receiver_session_uuid_;
  ros::CallbackQueue metadata_callback_queue_;
  std::unique_ptr<ros::NodeHandle> metadata_node_;
  std::unique_ptr<ros::AsyncSpinner> metadata_spinner_;
  ros::Subscriber metadata_subscriber_;
  ros::Publisher output_publisher_;

  std::unique_ptr<NonBlockingSpscQueue<BeliefEventPtr>> belief_queue_;
  std::unique_ptr<NonBlockingSpscQueue<MetadataEventPtr>> metadata_queue_;
  std::unique_ptr<NonBlockingSpscQueue<ReceiverEventPtr>> receiver_queue_;
  std::unique_ptr<NonBlockingSpscQueue<OutputEventPtr>> output_queue_;

  std::atomic<bool> stop_requested_{false};
  std::thread worker_;
  std::atomic<std::uint64_t> next_receipt_sequence_{1u};
  std::atomic<std::uint64_t> next_shadow_sequence_{1u};

  std::deque<BeliefEventPtr> pending_beliefs_;
  std::deque<MetadataEventPtr> pending_metadata_;
  std::deque<PendingMatchedArray> pending_matched_arrays_;
  std::map<ReceiptKey, ReceiverEventPtr> receiver_observations_;
  std::map<std::uint64_t, std::uint32_t> belief_association_reasons_;
  std::map<MetadataKey, sha256::Digest> metadata_fingerprints_;
  std::set<MetadataKey> conflicting_metadata_keys_;

  std::atomic<std::uint64_t> belief_arrays_received_{0u};
  std::atomic<std::uint64_t> metadata_arrays_received_{0u};
  std::atomic<std::uint64_t> receiver_observations_received_{0u};
  std::atomic<std::uint64_t> shadow_arrays_published_{0u};
  std::atomic<std::uint64_t> shadow_entries_published_{0u};
  std::atomic<std::uint64_t> dropped_belief_arrays_{0u};
  std::atomic<std::uint64_t> dropped_metadata_arrays_{0u};
  std::atomic<std::uint64_t> dropped_receiver_observations_{0u};
  std::atomic<std::uint64_t> dropped_shadow_arrays_{0u};
  std::atomic<std::uint64_t> orphan_beliefs_{0u};
  std::atomic<std::uint64_t> orphan_metadata_{0u};
  std::atomic<std::uint64_t> orphan_receiver_observations_{0u};
  std::atomic<std::uint64_t> digest_mismatches_{0u};
  std::atomic<std::uint64_t> exact_duplicate_metadata_{0u};
  std::atomic<std::uint64_t> conflicting_duplicate_metadata_{0u};
  std::atomic<std::uint64_t> worker_failures_{0u};
  std::atomic<std::size_t> maximum_belief_queue_depth_{0u};
  std::atomic<std::size_t> maximum_metadata_queue_depth_{0u};
  std::atomic<std::size_t> maximum_receiver_queue_depth_{0u};
  std::atomic<std::size_t> maximum_pending_beliefs_{0u};
  std::atomic<std::size_t> maximum_pending_metadata_{0u};
  std::atomic<std::size_t> maximum_pending_receiver_observations_{0u};
  std::atomic<std::uint64_t> belief_enqueue_time_ns_{0u};
  std::atomic<std::uint64_t> receiver_enqueue_time_ns_{0u};
  std::atomic<std::uint64_t> maximum_belief_enqueue_time_ns_{0u};
  std::atomic<std::uint64_t> maximum_receiver_enqueue_time_ns_{0u};
  std::atomic<std::uint64_t> worker_time_ns_{0u};
  std::atomic<std::uint64_t> association_time_ns_{0u};
  std::atomic<std::uint64_t> residual_time_ns_{0u};
  std::atomic<std::uint64_t> published_bytes_{0u};
  std::unique_ptr<std::ofstream> csv_stream_;
};

}  // namespace VIO::dcreg_shadow
