#include "kimera_vio_ros/DcregShadowAnalyzer.h"

#include <XmlRpcValue.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <utility>

#include <ros/serialization.h>

namespace VIO::dcreg_shadow {
namespace {

double wallNowSec() { return ros::WallTime::now().toSec(); }

std::uint64_t steadyNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::string makeSessionUuid() {
  std::random_device random;
  std::array<std::uint8_t, 16> bytes{};
  for (std::uint8_t& byte : bytes) {
    byte = static_cast<std::uint8_t>(random());
  }
  bytes[6] = static_cast<std::uint8_t>((bytes[6] & 0x0fu) | 0x40u);
  bytes[8] = static_cast<std::uint8_t>((bytes[8] & 0x3fu) | 0x80u);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t i = 0u; i < bytes.size(); ++i) {
    if (i == 4u || i == 6u || i == 8u || i == 10u) {
      stream << '-';
    }
    stream << std::setw(2) << static_cast<unsigned>(bytes[i]);
  }
  return stream.str();
}

std::string expandEnvironmentVariables(const std::string& input) {
  std::string output = input;
  std::size_t cursor = 0u;
  while ((cursor = output.find("${", cursor)) != std::string::npos) {
    const std::size_t end = output.find('}', cursor + 2u);
    if (end == std::string::npos) {
      break;
    }
    const std::string variable =
        output.substr(cursor + 2u, end - cursor - 2u);
    const char* value = std::getenv(variable.c_str());
    if (!value) {
      cursor = end + 1u;
      continue;
    }
    output.replace(cursor, end - cursor + 1u, value);
    cursor += std::char_traits<char>::length(value);
  }
  return output;
}

template <typename T>
void updateMaximum(std::atomic<T>* target, T value) {
  T observed = target->load(std::memory_order_relaxed);
  while (observed < value &&
         !target->compare_exchange_weak(observed,
                                        value,
                                        std::memory_order_relaxed,
                                        std::memory_order_relaxed)) {
  }
}

bool readBool(const XmlRpc::XmlRpcValue& root,
              const std::string& name,
              bool* value,
              std::string* error) {
  if (!root.hasMember(name)) {
    return true;
  }
  if (root[name].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    *error = name + " must be boolean";
    return false;
  }
  *value = static_cast<bool>(root[name]);
  return true;
}

bool readString(const XmlRpc::XmlRpcValue& root,
                const std::string& name,
                std::string* value,
                std::string* error) {
  if (!root.hasMember(name)) {
    return true;
  }
  if (root[name].getType() != XmlRpc::XmlRpcValue::TypeString) {
    *error = name + " must be string";
    return false;
  }
  *value = static_cast<std::string>(root[name]);
  return true;
}

bool readDouble(const XmlRpc::XmlRpcValue& root,
                const std::string& name,
                double* value,
                std::string* error) {
  if (!root.hasMember(name)) {
    return true;
  }
  const auto type = root[name].getType();
  if (type == XmlRpc::XmlRpcValue::TypeDouble) {
    *value = static_cast<double>(root[name]);
    return true;
  }
  if (type == XmlRpc::XmlRpcValue::TypeInt) {
    *value = static_cast<int>(root[name]);
    return true;
  }
  *error = name + " must be numeric";
  return false;
}

bool readSize(const XmlRpc::XmlRpcValue& root,
              const std::string& name,
              std::size_t* value,
              std::string* error) {
  if (!root.hasMember(name)) {
    return true;
  }
  if (root[name].getType() != XmlRpc::XmlRpcValue::TypeInt) {
    *error = name + " must be integer";
    return false;
  }
  const int parsed = static_cast<int>(root[name]);
  if (parsed <= 0) {
    *error = name + " must be greater than zero";
    return false;
  }
  *value = static_cast<std::size_t>(parsed);
  return true;
}

}  // namespace

struct Analyzer::MetadataObservation {
  double receipt_wall_time_sec = 0.0;
  liorf::DcregEdgeHealthMetadataArray message;
  sha256::Digest serialized_digest{};
};

struct Analyzer::ReceiverObservation {
  double receipt_wall_time_sec = 0.0;
  ExternalBeliefMatchDiagnostic diagnostic;
};

struct Analyzer::PendingMatchedArray {
  BeliefEventPtr belief;
  MetadataEventPtr metadata;
  double matched_wall_time_sec = 0.0;
};

struct Analyzer::OutputEvent {
  liorf::DcregBeliefShadowAnalysisArray message;
};

bool loadConfig(const ros::NodeHandle& private_node,
                const std::string& active_belief_topic,
                Config* config,
                bool* section_present,
                std::string* error) {
  if (!config || !section_present || !error) {
    return false;
  }
  *config = Config();
  config->belief_topic = active_belief_topic;
  *section_present = false;
  error->clear();

  XmlRpc::XmlRpcValue root;
  if (!private_node.getParam("dcreg_shadow_analysis", root)) {
    return true;
  }
  *section_present = true;
  if (root.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    *error = "dcreg_shadow_analysis must be a mapping";
    return false;
  }
  const std::set<std::string> supported = {
      "enabled",
      "belief_topic",
      "metadata_topic",
      "output_topic",
      "structured_csv_path",
      "maximum_pending_beliefs",
      "maximum_pending_metadata",
      "maximum_receiver_states",
      "maximum_receiver_observations",
      "association_timeout_sec",
      "compute_raw_disagreement",
      "compute_receiver_relative_covariance",
      "compute_normalized_disagreement",
      "beta_receiver_covariance",
      "model_floor_rotation",
      "model_floor_translation",
      "pseudoinverse_relative_threshold",
      "negative_eigenvalue_tolerance",
      "publication_queue_capacity",
      "overflow_policy"};
  for (auto it = root.begin(); it != root.end(); ++it) {
    if (supported.count(it->first) == 0u) {
      *error = "unsupported dcreg_shadow_analysis field: " + it->first;
      return false;
    }
  }

  if (!readBool(root, "enabled", &config->enabled, error) ||
      !readString(root, "belief_topic", &config->belief_topic, error) ||
      !readString(root, "metadata_topic", &config->metadata_topic, error) ||
      !readString(root, "output_topic", &config->output_topic, error) ||
      !readString(root,
                  "structured_csv_path",
                  &config->structured_csv_path,
                  error) ||
      !readSize(root,
                "maximum_pending_beliefs",
                &config->maximum_pending_beliefs,
                error) ||
      !readSize(root,
                "maximum_pending_metadata",
                &config->maximum_pending_metadata,
                error) ||
      !readSize(root,
                "maximum_receiver_observations",
                &config->maximum_receiver_observations,
                error) ||
      !readDouble(root,
                  "association_timeout_sec",
                  &config->association_timeout_sec,
                  error) ||
      !readBool(root,
                "compute_raw_disagreement",
                &config->compute_raw_disagreement,
                error) ||
      !readBool(root,
                "compute_receiver_relative_covariance",
                &config->compute_receiver_relative_covariance,
                error) ||
      !readBool(root,
                "compute_normalized_disagreement",
                &config->compute_normalized_disagreement,
                error) ||
      !readDouble(root,
                  "beta_receiver_covariance",
                  &config->beta_receiver_covariance,
                  error) ||
      !readDouble(root,
                  "model_floor_rotation",
                  &config->model_floor_rotation,
                  error) ||
      !readDouble(root,
                  "model_floor_translation",
                  &config->model_floor_translation,
                  error) ||
      !readDouble(root,
                  "pseudoinverse_relative_threshold",
                  &config->pseudoinverse_relative_threshold,
                  error) ||
      !readDouble(root,
                  "negative_eigenvalue_tolerance",
                  &config->negative_eigenvalue_tolerance,
                  error) ||
      !readSize(root,
                "publication_queue_capacity",
                &config->publication_queue_capacity,
                error) ||
      !readString(root, "overflow_policy", &config->overflow_policy, error)) {
    return false;
  }

  config->structured_csv_path =
      expandEnvironmentVariables(config->structured_csv_path);
  if (config->structured_csv_path.find("${") != std::string::npos) {
    *error = "structured_csv_path contains an unresolved environment variable";
    return false;
  }
  if (root.hasMember("maximum_receiver_states")) {
    std::size_t ignored = 0u;
    if (!readSize(root, "maximum_receiver_states", &ignored, error)) {
      return false;
    }
    config->maximum_receiver_observations = ignored;
  }

  if (!config->enabled) {
    return true;
  }
  if (config->belief_topic != active_belief_topic) {
    *error = "belief_topic must equal the active Kimera CBS input topic";
  } else if (config->metadata_topic.empty() || config->output_topic.empty()) {
    *error = "metadata_topic and output_topic must be non-empty";
  } else if (!(config->association_timeout_sec > 0.0) ||
             !std::isfinite(config->association_timeout_sec)) {
    *error = "association_timeout_sec must be finite and greater than zero";
  } else if (!config->compute_raw_disagreement) {
    *error = "Stage 2B requires compute_raw_disagreement=true";
  } else if (config->compute_receiver_relative_covariance) {
    *error = "exact receiver relative covariance is not exposed passively; "
             "compute_receiver_relative_covariance must remain false";
  } else if (config->compute_normalized_disagreement) {
    *error = "normalized disagreement requires exact receiver relative "
             "covariance and must remain false";
  } else if (!(config->beta_receiver_covariance > 0.0) ||
             config->model_floor_rotation < 0.0 ||
             config->model_floor_translation < 0.0 ||
             !(config->pseudoinverse_relative_threshold > 0.0) ||
             config->negative_eigenvalue_tolerance < 0.0) {
    *error = "invalid covariance-diagnostic numerical parameter";
  } else if (config->overflow_policy != "drop_newest") {
    *error = "only overflow_policy=drop_newest is supported";
  }
  return error->empty();
}

Analyzer::Analyzer(ros::NodeHandle node, Config config)
    : config_(std::move(config)), receiver_session_uuid_(makeSessionUuid()) {
  belief_queue_ = std::make_unique<NonBlockingSpscQueue<BeliefEventPtr>>(
      config_.publication_queue_capacity);
  metadata_queue_ = std::make_unique<NonBlockingSpscQueue<MetadataEventPtr>>(
      config_.publication_queue_capacity);
  receiver_queue_ = std::make_unique<NonBlockingSpscQueue<ReceiverEventPtr>>(
      config_.publication_queue_capacity);
  output_queue_ = std::make_unique<NonBlockingSpscQueue<OutputEventPtr>>(
      config_.publication_queue_capacity);
  output_publisher_ =
      node.advertise<liorf::DcregBeliefShadowAnalysisArray>(
          config_.output_topic, 10, false);
  // Keep metadata transport callbacks off Kimera's estimator callback queue.
  // This queue owns only passive Stage 2B traffic; callback work is bounded by
  // the drop-newest SPSC queue and never runs on the VIO callback thread.
  metadata_node_ = std::make_unique<ros::NodeHandle>(node);
  metadata_node_->setCallbackQueue(&metadata_callback_queue_);
  metadata_subscriber_ =
      metadata_node_->subscribe<liorf::DcregEdgeHealthMetadataArray>(
          config_.metadata_topic,
          50,
          &Analyzer::metadataCallback,
          this,
          ros::TransportHints().tcpNoDelay());
  metadata_spinner_ =
      std::make_unique<ros::AsyncSpinner>(1u, &metadata_callback_queue_);
  metadata_spinner_->start();
  worker_ = std::thread(&Analyzer::workerLoop, this);
}

Analyzer::~Analyzer() { stop(); }

std::uint64_t Analyzer::nextBeliefReceiptSequence() {
  return next_receipt_sequence_.fetch_add(1u, std::memory_order_relaxed);
}

bool Analyzer::tryEnqueueBeliefArray(
    const liorf::pose_odom_belief_array& message,
    std::uint64_t receipt_sequence) {
  const std::uint64_t start_ns = steadyNowNs();
  auto event = std::make_shared<BeliefArrayObservation>();
  event->receipt_sequence = receipt_sequence;
  event->receipt_wall_time_sec = wallNowSec();
  event->message = message;
  if (!belief_queue_->tryPush(std::move(event))) {
    const std::uint64_t duration_ns = steadyNowNs() - start_ns;
    belief_enqueue_time_ns_.fetch_add(duration_ns,
                                      std::memory_order_relaxed);
    updateMaximum(&maximum_belief_enqueue_time_ns_, duration_ns);
    dropped_belief_arrays_.fetch_add(1u, std::memory_order_relaxed);
    return false;
  }
  belief_arrays_received_.fetch_add(1u, std::memory_order_relaxed);
  updateMaximum(&maximum_belief_queue_depth_,
                belief_queue_->approximateSize());
  const std::uint64_t duration_ns = steadyNowNs() - start_ns;
  belief_enqueue_time_ns_.fetch_add(duration_ns, std::memory_order_relaxed);
  updateMaximum(&maximum_belief_enqueue_time_ns_, duration_ns);
  return true;
}

void Analyzer::metadataCallback(
    const liorf::DcregEdgeHealthMetadataArrayConstPtr& message) {
  if (!message) {
    return;
  }
  auto event = std::make_shared<MetadataObservation>();
  event->receipt_wall_time_sec = wallNowSec();
  event->message = *message;
  const std::uint32_t serialized_size =
      ros::serialization::serializationLength(event->message);
  std::vector<std::uint8_t> serialized(serialized_size);
  ros::serialization::OStream stream(serialized.data(), serialized.size());
  ros::serialization::serialize(stream, event->message);
  event->serialized_digest = sha256::hash(serialized);
  if (!metadata_queue_->tryPush(std::move(event))) {
    dropped_metadata_arrays_.fetch_add(1u, std::memory_order_relaxed);
    return;
  }
  metadata_arrays_received_.fetch_add(1u, std::memory_order_relaxed);
  updateMaximum(&maximum_metadata_queue_depth_,
                metadata_queue_->approximateSize());
}

bool Analyzer::tryEnqueueReceiverObservation(
    const ExternalBeliefMatchDiagnostic& observation) {
  const std::uint64_t start_ns = steadyNowNs();
  auto event = std::make_shared<ReceiverObservation>();
  event->receipt_wall_time_sec = wallNowSec();
  event->diagnostic = observation;
  if (!receiver_queue_->tryPush(std::move(event))) {
    const std::uint64_t duration_ns = steadyNowNs() - start_ns;
    receiver_enqueue_time_ns_.fetch_add(duration_ns,
                                        std::memory_order_relaxed);
    updateMaximum(&maximum_receiver_enqueue_time_ns_, duration_ns);
    dropped_receiver_observations_.fetch_add(1u,
                                             std::memory_order_relaxed);
    return false;
  }
  receiver_observations_received_.fetch_add(1u,
                                             std::memory_order_relaxed);
  updateMaximum(&maximum_receiver_queue_depth_,
                receiver_queue_->approximateSize());
  const std::uint64_t duration_ns = steadyNowNs() - start_ns;
  receiver_enqueue_time_ns_.fetch_add(duration_ns,
                                      std::memory_order_relaxed);
  updateMaximum(&maximum_receiver_enqueue_time_ns_, duration_ns);
  return true;
}

void Analyzer::drainInputQueues() {
  BeliefEventPtr belief;
  while (belief_queue_->tryPop(&belief)) {
    pending_beliefs_.push_back(std::move(belief));
  }
  MetadataEventPtr metadata;
  while (metadata_queue_->tryPop(&metadata)) {
    const MetadataKey key{metadata->message.sender_session_uuid,
                          metadata->message.belief_publication_sequence};
    const auto fingerprint = metadata_fingerprints_.find(key);
    if (fingerprint != metadata_fingerprints_.end()) {
      if (fingerprint->second == metadata->serialized_digest) {
        exact_duplicate_metadata_.fetch_add(1u,
                                            std::memory_order_relaxed);
      } else {
        conflicting_duplicate_metadata_.fetch_add(
            1u, std::memory_order_relaxed);
        conflicting_metadata_keys_.insert(key);
      }
      continue;
    }
    metadata_fingerprints_[key] = metadata->serialized_digest;
    pending_metadata_.push_back(std::move(metadata));
  }
  ReceiverEventPtr receiver;
  while (receiver_queue_->tryPop(&receiver)) {
    if (!receiver->diagnostic.shadow_identity_valid) {
      orphan_receiver_observations_.fetch_add(1u,
                                               std::memory_order_relaxed);
      continue;
    }
    const ReceiptKey key{receiver->diagnostic.shadow_receipt_sequence,
                         receiver->diagnostic.shadow_belief_ordinal};
    auto existing = receiver_observations_.find(key);
    if (existing == receiver_observations_.end() ||
        (!existing->second->diagnostic.terminal &&
         receiver->diagnostic.terminal)) {
      receiver_observations_[key] = std::move(receiver);
    }
  }

  while (pending_beliefs_.size() > config_.maximum_pending_beliefs) {
    belief_association_reasons_.erase(
        pending_beliefs_.front()->receipt_sequence);
    pending_beliefs_.pop_front();
    orphan_beliefs_.fetch_add(1u, std::memory_order_relaxed);
  }
  while (pending_metadata_.size() > config_.maximum_pending_metadata) {
    const MetadataKey key{
        pending_metadata_.front()->message.sender_session_uuid,
        pending_metadata_.front()->message.belief_publication_sequence};
    metadata_fingerprints_.erase(key);
    conflicting_metadata_keys_.erase(key);
    pending_metadata_.pop_front();
    orphan_metadata_.fetch_add(1u, std::memory_order_relaxed);
  }
  while (receiver_observations_.size() >
         config_.maximum_receiver_observations) {
    receiver_observations_.erase(receiver_observations_.begin());
    orphan_receiver_observations_.fetch_add(1u,
                                             std::memory_order_relaxed);
  }
  updateMaximum(&maximum_pending_beliefs_, pending_beliefs_.size());
  updateMaximum(&maximum_pending_metadata_, pending_metadata_.size());
  updateMaximum(&maximum_pending_receiver_observations_,
                receiver_observations_.size());
}

void Analyzer::matchBeliefsAndMetadata() {
  const std::uint64_t start_ns = steadyNowNs();
  for (std::size_t metadata_index = 0u;
       metadata_index < pending_metadata_.size();) {
    const auto& metadata = pending_metadata_[metadata_index];
    std::size_t matched_belief_index = pending_beliefs_.size();
    std::size_t exact_match_count = 0u;
    bool saw_digest_mismatch = false;
    for (std::size_t belief_index = 0u;
         belief_index < pending_beliefs_.size();
         ++belief_index) {
      const auto& belief = pending_beliefs_[belief_index];
      if (belief->message.header.stamp !=
              metadata->message.belief_array_header_stamp ||
          belief->message.header.frame_id !=
              metadata->message.belief_array_frame_id) {
        continue;
      }
      std::uint32_t reason = 0u;
      if (metadataMatchesBeliefArray(
              belief->message, metadata->message, &reason)) {
        matched_belief_index = belief_index;
        ++exact_match_count;
        continue;
      }
      belief_association_reasons_[belief->receipt_sequence] |= reason;
      if ((reason & liorf::DcregBeliefShadowAnalysis::
                        REASON_PAYLOAD_DIGEST_MISMATCH) != 0u) {
        saw_digest_mismatch = true;
      }
    }
    if (exact_match_count > 1u) {
      for (const auto& belief : pending_beliefs_) {
        if (belief->message.header.stamp ==
                metadata->message.belief_array_header_stamp &&
            belief->message.header.frame_id ==
                metadata->message.belief_array_frame_id) {
          belief_association_reasons_[belief->receipt_sequence] |=
              liorf::DcregBeliefShadowAnalysis::
                  REASON_METADATA_IDENTITY_CONFLICT;
        }
      }
      ++metadata_index;
      continue;
    }
    if (matched_belief_index == pending_beliefs_.size()) {
      if (saw_digest_mismatch) {
        digest_mismatches_.fetch_add(1u, std::memory_order_relaxed);
      }
      ++metadata_index;
      continue;
    }
    PendingMatchedArray matched;
    matched.belief = pending_beliefs_[matched_belief_index];
    matched.metadata = metadata;
    matched.matched_wall_time_sec = wallNowSec();
    const std::uint64_t matched_receipt_sequence =
        matched.belief->receipt_sequence;
    pending_matched_arrays_.push_back(std::move(matched));
    pending_beliefs_.erase(pending_beliefs_.begin() + matched_belief_index);
    belief_association_reasons_.erase(matched_receipt_sequence);
    pending_metadata_.erase(pending_metadata_.begin() + metadata_index);
  }
  association_time_ns_.fetch_add(steadyNowNs() - start_ns,
                                 std::memory_order_relaxed);
}

void Analyzer::queueBeliefWithoutMetadata(
    const BeliefEventPtr& belief,
    const std::uint32_t reason_mask) {
  if (!belief) {
    return;
  }
  liorf::DcregBeliefShadowAnalysisArray output;
  output.header = belief->message.header;
  output.schema_version = kSchemaVersion;
  output.semantics_version = kSemanticsVersion;
  output.receiver_session_uuid = receiver_session_uuid_;
  output.entries.reserve(belief->message.beliefs.size());
  for (std::uint32_t ordinal = 0u;
       ordinal < belief->message.beliefs.size();
       ++ordinal) {
    const ReceiptKey key{belief->receipt_sequence, ordinal};
    const auto receiver = receiver_observations_.find(key);
    const ExternalBeliefMatchDiagnostic* diagnostic =
        receiver == receiver_observations_.end()
            ? nullptr
            : &receiver->second->diagnostic;
    auto entry = makeShadowEntry(belief->message.beliefs[ordinal],
                                 ordinal,
                                 nullptr,
                                 diagnostic,
                                 "",
                                 0u,
                                 receiver_session_uuid_,
                                 next_shadow_sequence_.fetch_add(
                                     1u, std::memory_order_relaxed),
                                 config_);
    entry.invalid_reason_mask |=
        reason_mask | liorf::DcregBeliefShadowAnalysis::REASON_EXPIRED;
    entry.processing_latency_ms =
        1000.0 * (wallNowSec() - belief->receipt_wall_time_sec);
    output.entries.push_back(std::move(entry));
    if (receiver != receiver_observations_.end()) {
      receiver_observations_.erase(receiver);
    }
  }
  output.cumulative_dropped_shadow_array_count =
      dropped_shadow_arrays_.load(std::memory_order_relaxed);
  output.cumulative_orphan_belief_count =
      orphan_beliefs_.load(std::memory_order_relaxed) + 1u;
  output.cumulative_orphan_metadata_count =
      orphan_metadata_.load(std::memory_order_relaxed);
  output.cumulative_orphan_receiver_observation_count =
      orphan_receiver_observations_.load(std::memory_order_relaxed);
  queueOutput(std::move(output));
}

void Analyzer::finalizeReadyArrays(bool force_shutdown) {
  const double now = wallNowSec();
  for (auto it = pending_matched_arrays_.begin();
       it != pending_matched_arrays_.end();) {
    bool all_terminal = true;
    for (const auto& metadata_entry : it->metadata->message.entries) {
      const ReceiptKey key{it->belief->receipt_sequence,
                           metadata_entry.belief_ordinal};
      const auto receiver = receiver_observations_.find(key);
      if (receiver == receiver_observations_.end() ||
          !receiver->second->diagnostic.terminal) {
        all_terminal = false;
        break;
      }
    }
    const bool expired =
        now - it->matched_wall_time_sec >= config_.association_timeout_sec;
    if (!all_terminal && !expired && !force_shutdown) {
      ++it;
      continue;
    }

    liorf::DcregBeliefShadowAnalysisArray output;
    output.header = it->belief->message.header;
    output.schema_version = kSchemaVersion;
    output.semantics_version = kSemanticsVersion;
    output.receiver_session_uuid = receiver_session_uuid_;
    output.sender_session_uuid = it->metadata->message.sender_session_uuid;
    output.belief_publication_sequence =
        it->metadata->message.belief_publication_sequence;
    output.source_metadata_schema_version =
        it->metadata->message.schema_version;
    output.source_metadata_semantics_version =
        it->metadata->message.semantics_version;
    output.entries.reserve(it->metadata->message.entries.size());
    const MetadataKey metadata_key{
        it->metadata->message.sender_session_uuid,
        it->metadata->message.belief_publication_sequence};
    const bool conflicting_metadata =
        conflicting_metadata_keys_.count(metadata_key) != 0u;

    for (const auto& metadata_entry : it->metadata->message.entries) {
      if (metadata_entry.belief_ordinal >=
          it->belief->message.beliefs.size()) {
        continue;
      }
      const ReceiptKey key{it->belief->receipt_sequence,
                           metadata_entry.belief_ordinal};
      const auto receiver = receiver_observations_.find(key);
      const ExternalBeliefMatchDiagnostic* diagnostic =
          receiver == receiver_observations_.end()
              ? nullptr
              : &receiver->second->diagnostic;
      const std::uint64_t residual_start_ns = steadyNowNs();
      auto entry = makeShadowEntry(
          it->belief->message.beliefs[metadata_entry.belief_ordinal],
          metadata_entry.belief_ordinal,
          &metadata_entry,
          diagnostic,
          it->metadata->message.sender_session_uuid,
          it->metadata->message.belief_publication_sequence,
          receiver_session_uuid_,
          next_shadow_sequence_.fetch_add(1u, std::memory_order_relaxed),
          config_);
      residual_time_ns_.fetch_add(steadyNowNs() - residual_start_ns,
                                  std::memory_order_relaxed);
      if (conflicting_metadata) {
        entry.belief_metadata_match_valid = false;
        entry.sender_lidar_health_state =
            liorf::DcregBeliefShadowAnalysis::HEALTH_INVALID;
        entry.invalid_reason_mask |= liorf::DcregBeliefShadowAnalysis::
            REASON_CONFLICTING_DUPLICATE;
      }
      if (!diagnostic && (expired || force_shutdown)) {
        entry.invalid_reason_mask |=
            liorf::DcregBeliefShadowAnalysis::REASON_EXPIRED;
      }
      entry.cumulative_dropped_shadow_array_count =
          dropped_shadow_arrays_.load(std::memory_order_relaxed);
      entry.cumulative_orphan_belief_count =
          orphan_beliefs_.load(std::memory_order_relaxed);
      entry.cumulative_orphan_metadata_count =
          orphan_metadata_.load(std::memory_order_relaxed);
      entry.cumulative_orphan_receiver_observation_count =
          orphan_receiver_observations_.load(std::memory_order_relaxed);
      entry.processing_latency_ms =
          1000.0 * (now - it->metadata->receipt_wall_time_sec);
      output.entries.push_back(std::move(entry));
      if (receiver != receiver_observations_.end()) {
        receiver_observations_.erase(receiver);
      }
    }
    output.cumulative_dropped_shadow_array_count =
        dropped_shadow_arrays_.load(std::memory_order_relaxed);
    output.cumulative_orphan_belief_count =
        orphan_beliefs_.load(std::memory_order_relaxed);
    output.cumulative_orphan_metadata_count =
        orphan_metadata_.load(std::memory_order_relaxed);
    output.cumulative_orphan_receiver_observation_count =
        orphan_receiver_observations_.load(std::memory_order_relaxed);
    queueOutput(std::move(output));
    metadata_fingerprints_.erase(metadata_key);
    conflicting_metadata_keys_.erase(metadata_key);
    it = pending_matched_arrays_.erase(it);
  }
}

void Analyzer::expireOrphans(bool force_shutdown) {
  const double now = wallNowSec();
  while (!pending_beliefs_.empty() &&
         (force_shutdown ||
          now - pending_beliefs_.front()->receipt_wall_time_sec >=
              config_.association_timeout_sec)) {
    const auto belief = pending_beliefs_.front();
    const auto reason = belief_association_reasons_.find(
        belief->receipt_sequence);
    const std::uint32_t reason_mask =
        reason == belief_association_reasons_.end() ? 0u : reason->second;
    queueBeliefWithoutMetadata(belief, reason_mask);
    belief_association_reasons_.erase(belief->receipt_sequence);
    pending_beliefs_.pop_front();
    orphan_beliefs_.fetch_add(1u, std::memory_order_relaxed);
  }
  while (!pending_metadata_.empty() &&
         (force_shutdown ||
          now - pending_metadata_.front()->receipt_wall_time_sec >=
              config_.association_timeout_sec)) {
    const MetadataKey key{
        pending_metadata_.front()->message.sender_session_uuid,
        pending_metadata_.front()->message.belief_publication_sequence};
    metadata_fingerprints_.erase(key);
    conflicting_metadata_keys_.erase(key);
    pending_metadata_.pop_front();
    orphan_metadata_.fetch_add(1u, std::memory_order_relaxed);
  }
  for (auto it = receiver_observations_.begin();
       it != receiver_observations_.end();) {
    if (force_shutdown ||
        now - it->second->receipt_wall_time_sec >=
            config_.association_timeout_sec) {
      it = receiver_observations_.erase(it);
      orphan_receiver_observations_.fetch_add(1u,
                                               std::memory_order_relaxed);
    } else {
      ++it;
    }
  }
}

void Analyzer::queueOutput(
    liorf::DcregBeliefShadowAnalysisArray message) {
  auto event = std::make_shared<OutputEvent>();
  event->message = std::move(message);
  if (!output_queue_->tryPush(std::move(event))) {
    dropped_shadow_arrays_.fetch_add(1u, std::memory_order_relaxed);
  }
}

void Analyzer::writeCsvHeader() {
  if (!csv_stream_ || !*csv_stream_) {
    return;
  }
  *csv_stream_
      << "schema_version,semantics_version,sender_session_uuid,"
         "receiver_session_uuid,publication_sequence,ordinal,from_index,"
         "to_index,from_stamp_sec,to_stamp_sec,metadata_match,receiver_match,"
         "receiver_match_status,active_factor_accepted,start_match_type,"
         "end_match_type,start_ambiguous_count,end_ambiguous_count,"
         "health_state,association_valid,association_complete,"
         "expected_samples,observed_samples,valid_hessian_samples,"
         "health_available_samples,metadata_reason_mask,reference_ready,"
         "reference_consistent,reference_source,relative_health_valid,"
         "rot_health_min,rot_health_median,trans_health_min,"
         "trans_health_median,rot_degraded_fraction,trans_degraded_fraction,"
         "rot_absolute_fraction,trans_absolute_fraction,worst_rot_condition,"
         "worst_trans_condition,clustered_rot_fraction,"
         "clustered_trans_fraction,low_rot_alignment_fraction,"
         "low_trans_alignment_fraction,registration_converged_fraction,"
         "linear_solve_success_fraction,source_point_mean,inlier_count_mean,"
         "inlier_fraction_mean,initial_cost_mean,final_cost_mean,"
         "cost_reduction_mean,rot_disagreement_rad,trans_disagreement_m,"
         "residual_r0,residual_r1,residual_r2,residual_t0,residual_t1,"
         "residual_t2,sender_r0,sender_r1,sender_r2,sender_t0,sender_t1,"
         "sender_t2,receiver_r0,receiver_r1,receiver_r2,receiver_t0,"
         "receiver_t1,receiver_t2,sender_covariance_trace,"
         "receiver_from_index,receiver_to_index,start_dt_sec,end_dt_sec,"
         "duration_error_sec,invalid_reason_mask,stable_edge_digest,"
         "payload_digest,"
         "processing_latency_ms\n";
}

void Analyzer::writeCsv(
    const liorf::DcregBeliefShadowAnalysisArray& message) {
  if (!csv_stream_ || !*csv_stream_) {
    return;
  }
  for (const auto& entry : message.entries) {
    const auto& health = entry.stage2a_metadata;
    *csv_stream_ << message.schema_version << ',' << message.semantics_version
                 << ',' << entry.sender_session_uuid << ','
                 << entry.receiver_session_uuid << ','
                 << entry.belief_publication_sequence << ','
                 << entry.belief_ordinal << ',' << entry.sender_from_pose_index
                 << ',' << entry.sender_to_pose_index << ','
                 << std::setprecision(17) << entry.sender_from_stamp_sec << ','
                 << entry.sender_to_stamp_sec << ','
                 << static_cast<unsigned>(entry.belief_metadata_match_valid)
                 << ','
                 << static_cast<unsigned>(entry.receiver_state_match_valid)
                 << ','
                 << static_cast<unsigned>(entry.receiver_match_status) << ','
                 << static_cast<unsigned>(entry.active_factor_accepted) << ','
                 << static_cast<unsigned>(entry.start_timestamp_match_type)
                 << ','
                 << static_cast<unsigned>(entry.end_timestamp_match_type)
                 << ',' << entry.start_ambiguous_match_count << ','
                 << entry.end_ambiguous_match_count << ','
                 << static_cast<unsigned>(entry.sender_lidar_health_state)
                 << ',' << static_cast<unsigned>(health.association_valid) << ','
                 << static_cast<unsigned>(health.association_complete) << ','
                 << health.expected_sample_count << ','
                 << health.observed_sample_count << ','
                 << health.valid_hessian_sample_count << ','
                 << health.health_available_sample_count << ','
                 << health.reason_mask << ','
                 << static_cast<unsigned>(health.reference_ready) << ','
                 << static_cast<unsigned>(health.reference_consistent) << ','
                 << static_cast<unsigned>(health.reference_source) << ','
                 << static_cast<unsigned>(health.relative_health_valid) << ','
                 << health.rotational_health_minimum << ','
                 << health.rotational_health_median << ','
                 << health.translational_health_minimum << ','
                 << health.translational_health_median << ','
                 << health.rotational_degraded_fraction << ','
                 << health.translational_degraded_fraction << ','
                 << health.rotational_absolute_degenerate_fraction << ','
                 << health.translational_absolute_degenerate_fraction << ','
                 << health.worst_rotational_condition_ratio << ','
                 << health.worst_translational_condition_ratio << ','
                 << health.clustered_rotation_fraction << ','
                 << health.clustered_translation_fraction << ','
                 << health.low_rotation_alignment_fraction << ','
                 << health.low_translation_alignment_fraction << ','
                 << health.registration_converged_fraction << ','
                 << health.linear_solve_success_fraction << ','
                 << health.aggregate_source_point_count.mean << ','
                 << health.aggregate_inlier_count.mean << ','
                 << health.aggregate_inlier_fraction.mean << ','
                 << health.aggregate_initial_cost.mean << ','
                 << health.aggregate_final_cost.mean << ','
                 << health.aggregate_cost_reduction.mean << ','
                 << entry.rotational_disagreement_norm_rad << ','
                 << entry.translational_disagreement_norm_m;
    for (const double value : entry.disagreement_residual) {
      *csv_stream_ << ',' << value;
    }
    for (const double value : entry.sender_relative_mu_base) {
      *csv_stream_ << ',' << value;
    }
    for (const double value : entry.receiver_relative_mu) {
      *csv_stream_ << ',' << value;
    }
    double sender_covariance_trace = std::numeric_limits<double>::quiet_NaN();
    if (entry.sender_covariance_available) {
      sender_covariance_trace = 0.0;
      for (std::size_t i = 0u; i < 6u; ++i) {
        sender_covariance_trace += entry.sender_covariance_base[i * 6u + i];
      }
    }
    *csv_stream_ << ',' << sender_covariance_trace << ','
                 << entry.receiver_from_pose_index << ','
                 << entry.receiver_to_pose_index << ','
                 << entry.start_timestamp_error_sec << ','
                 << entry.end_timestamp_error_sec << ','
                 << entry.interval_duration_error_sec << ','
                 << entry.invalid_reason_mask << ',';
    for (const auto byte : entry.stable_edge_digest) {
      *csv_stream_ << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<unsigned>(byte);
    }
    *csv_stream_ << ',';
    for (const auto byte : entry.belief_payload_digest) {
      *csv_stream_ << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<unsigned>(byte);
    }
    *csv_stream_ << std::dec << std::setfill(' ') << ','
                 << entry.processing_latency_ms << '\n';
  }
}

void Analyzer::publishOutputQueue() {
  OutputEventPtr output;
  while (output_queue_->tryPop(&output)) {
    try {
      output_publisher_.publish(output->message);
      published_bytes_.fetch_add(
          ros::serialization::serializationLength(output->message),
          std::memory_order_relaxed);
      writeCsv(output->message);
      shadow_arrays_published_.fetch_add(1u, std::memory_order_relaxed);
      shadow_entries_published_.fetch_add(output->message.entries.size(),
                                          std::memory_order_relaxed);
    } catch (...) {
      worker_failures_.fetch_add(1u, std::memory_order_relaxed);
    }
  }
}

void Analyzer::workerLoop() {
  try {
    if (!config_.structured_csv_path.empty()) {
      csv_stream_ = std::make_unique<std::ofstream>(
          config_.structured_csv_path, std::ios::out | std::ios::trunc);
      if (!*csv_stream_) {
        csv_stream_.reset();
        worker_failures_.fetch_add(1u, std::memory_order_relaxed);
      } else {
        writeCsvHeader();
      }
    }
    while (!stop_requested_.load(std::memory_order_acquire)) {
      const std::uint64_t loop_start_ns = steadyNowNs();
      drainInputQueues();
      matchBeliefsAndMetadata();
      finalizeReadyArrays(false);
      expireOrphans(false);
      publishOutputQueue();
      worker_time_ns_.fetch_add(steadyNowNs() - loop_start_ns,
                                std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    drainInputQueues();
    matchBeliefsAndMetadata();
    finalizeReadyArrays(true);
    expireOrphans(true);
    publishOutputQueue();
    if (csv_stream_) {
      csv_stream_->flush();
    }
  } catch (...) {
    worker_failures_.fetch_add(1u, std::memory_order_relaxed);
  }
}

Analyzer::Statistics Analyzer::statistics() const {
  Statistics output;
  output.belief_arrays_received = belief_arrays_received_.load();
  output.metadata_arrays_received = metadata_arrays_received_.load();
  output.receiver_observations_received =
      receiver_observations_received_.load();
  output.shadow_arrays_published = shadow_arrays_published_.load();
  output.shadow_entries_published = shadow_entries_published_.load();
  output.dropped_belief_arrays = dropped_belief_arrays_.load();
  output.dropped_metadata_arrays = dropped_metadata_arrays_.load();
  output.dropped_receiver_observations =
      dropped_receiver_observations_.load();
  output.dropped_shadow_arrays = dropped_shadow_arrays_.load();
  output.orphan_beliefs = orphan_beliefs_.load();
  output.orphan_metadata = orphan_metadata_.load();
  output.orphan_receiver_observations =
      orphan_receiver_observations_.load();
  output.digest_mismatches = digest_mismatches_.load();
  output.exact_duplicate_metadata = exact_duplicate_metadata_.load();
  output.conflicting_duplicate_metadata =
      conflicting_duplicate_metadata_.load();
  output.worker_failures = worker_failures_.load();
  output.maximum_belief_queue_depth = maximum_belief_queue_depth_.load();
  output.maximum_metadata_queue_depth = maximum_metadata_queue_depth_.load();
  output.maximum_receiver_queue_depth = maximum_receiver_queue_depth_.load();
  output.maximum_pending_beliefs = maximum_pending_beliefs_.load();
  output.maximum_pending_metadata = maximum_pending_metadata_.load();
  output.maximum_pending_receiver_observations =
      maximum_pending_receiver_observations_.load();
  output.producer_waits = 0u;
  output.belief_enqueue_time_ns = belief_enqueue_time_ns_.load();
  output.receiver_enqueue_time_ns = receiver_enqueue_time_ns_.load();
  output.maximum_belief_enqueue_time_ns =
      maximum_belief_enqueue_time_ns_.load();
  output.maximum_receiver_enqueue_time_ns =
      maximum_receiver_enqueue_time_ns_.load();
  output.worker_time_ns = worker_time_ns_.load();
  output.association_time_ns = association_time_ns_.load();
  output.residual_time_ns = residual_time_ns_.load();
  output.published_bytes = published_bytes_.load();
  return output;
}

void Analyzer::stop() {
  metadata_subscriber_.shutdown();
  if (metadata_spinner_) {
    metadata_spinner_->stop();
  }
  const bool already_stopped =
      stop_requested_.exchange(true, std::memory_order_acq_rel);
  if (!already_stopped && worker_.joinable()) {
    worker_.join();
  } else if (worker_.joinable()) {
    worker_.join();
  }
  metadata_spinner_.reset();
  metadata_node_.reset();
  output_publisher_.shutdown();
}

}  // namespace VIO::dcreg_shadow
