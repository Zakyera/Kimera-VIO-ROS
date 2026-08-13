#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <liorf/DcregBeliefShadowAnalysis.h>
#include <liorf/DcregEdgeHealthMetadata.h>
#include <liorf/DcregEdgeHealthMetadataArray.h>
#include <liorf/pose_odom_belief.h>
#include <liorf/pose_odom_belief_array.h>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera_vio_ros/DcregSha256.h"

namespace VIO::dcreg_shadow {

constexpr std::uint32_t kSchemaVersion = 1u;
constexpr std::uint32_t kSemanticsVersion = 1u;
constexpr std::uint32_t kSupportedStage2aSchemaVersion = 1u;
constexpr std::uint32_t kSupportedStage2aSemanticsVersion = 1u;

struct Config {
  bool enabled = false;
  std::string belief_topic = "/kimera/cbs/odom_belief_in";
  std::string metadata_topic = "/glim/cbs/dcreg_edge_health_metadata";
  std::string output_topic = "/kimera/cbs/dcreg_shadow_analysis";
  std::string structured_csv_path;
  std::size_t maximum_pending_beliefs = 512u;
  std::size_t maximum_pending_metadata = 512u;
  std::size_t maximum_receiver_observations = 4096u;
  double association_timeout_sec = 2.0;
  bool compute_raw_disagreement = true;
  bool compute_receiver_relative_covariance = false;
  bool compute_normalized_disagreement = false;
  double beta_receiver_covariance = 1.0;
  double model_floor_rotation = 0.0;
  double model_floor_translation = 0.0;
  double pseudoinverse_relative_threshold = 1.0e-10;
  double negative_eigenvalue_tolerance = 1.0e-10;
  std::size_t publication_queue_capacity = 256u;
  std::string overflow_policy = "drop_newest";
};

struct BeliefArrayObservation {
  std::uint64_t receipt_sequence = 0u;
  double receipt_wall_time_sec = 0.0;
  liorf::pose_odom_belief_array message;
};

struct DisagreementResult {
  bool valid = false;
  gtsam::Vector6 residual = gtsam::Vector6::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double rotation_norm = std::numeric_limits<double>::quiet_NaN();
  double translation_norm = std::numeric_limits<double>::quiet_NaN();
};

struct NormalizedDisagreementResult {
  bool valid = false;
  double value = std::numeric_limits<double>::quiet_NaN();
  int rank = 0;
  double condition = std::numeric_limits<double>::quiet_NaN();
  double regularization = 0.0;
};

struct GroundTruthEdgeEvaluation {
  bool valid = false;
  gtsam::Vector6 sender_error = gtsam::Vector6::Constant(
      std::numeric_limits<double>::quiet_NaN());
  gtsam::Vector6 receiver_error = gtsam::Vector6::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double sender_rotation_error = std::numeric_limits<double>::quiet_NaN();
  double sender_translation_error = std::numeric_limits<double>::quiet_NaN();
  double receiver_rotation_error = std::numeric_limits<double>::quiet_NaN();
  double receiver_translation_error = std::numeric_limits<double>::quiet_NaN();
};

sha256::Digest128 beliefPayloadDigest(
    const liorf::pose_odom_belief_array& belief_array,
    std::uint32_t ordinal,
    std::uint64_t publication_sequence);

sha256::Digest128 stableEdgeDigest(
    const liorf::pose_odom_belief& belief,
    const std::string& sender_session_uuid);

bool metadataMatchesBeliefArray(
    const liorf::pose_odom_belief_array& belief_array,
    const liorf::DcregEdgeHealthMetadataArray& metadata,
    std::uint32_t* reason_mask = nullptr);

DisagreementResult computeDisagreement(
    const gtsam::Pose3& sender_relative_base,
    const gtsam::Pose3& receiver_relative);

std::uint8_t timestampMatchType(double absolute_timestamp_error_sec);

NormalizedDisagreementResult computeNormalizedDisagreement(
    const gtsam::Vector6& residual,
    const gtsam::Matrix6& sender_covariance,
    const gtsam::Matrix6& receiver_relative_covariance,
    const Config& config);

std::uint8_t classifySenderHealth(
    const liorf::DcregEdgeHealthMetadata* metadata);

liorf::DcregBeliefShadowAnalysis makeShadowEntry(
    const liorf::pose_odom_belief& belief,
    std::uint32_t belief_ordinal,
    const liorf::DcregEdgeHealthMetadata* metadata,
    const ExternalBeliefMatchDiagnostic* receiver_observation,
    const std::string& sender_session_uuid,
    std::uint64_t publication_sequence,
    const std::string& receiver_session_uuid,
    std::uint64_t shadow_sequence,
    const Config& config);

GroundTruthEdgeEvaluation evaluateGroundTruthEdge(
    const gtsam::Pose3& sender_relative,
    const gtsam::Pose3& receiver_relative,
    const gtsam::Pose3& ground_truth_relative);

}  // namespace VIO::dcreg_shadow
