#include "kimera_vio_ros/DcregShadowAnalysis.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace VIO::dcreg_shadow {
namespace {

double nanValue() { return std::numeric_limits<double>::quiet_NaN(); }

template <typename T>
void appendBigEndian(std::vector<std::uint8_t>* bytes, T value) {
  static_assert(std::is_unsigned<T>::value, "unsigned integer required");
  for (int shift = static_cast<int>(sizeof(T) * 8u) - 8;
       shift >= 0;
       shift -= 8) {
    bytes->push_back(static_cast<std::uint8_t>(value >> shift));
  }
}

void appendDouble(std::vector<std::uint8_t>* bytes, double value) {
  static_assert(sizeof(double) == sizeof(std::uint64_t),
                "canonical encoding requires binary64");
  std::uint64_t bits = 0u;
  std::memcpy(&bits, &value, sizeof(bits));
  appendBigEndian(bytes, bits);
}

void appendString(std::vector<std::uint8_t>* bytes,
                  const std::string& value) {
  appendBigEndian(bytes, static_cast<std::uint32_t>(value.size()));
  bytes->insert(bytes->end(), value.begin(), value.end());
}

template <typename Array>
void appendDoubleArray(std::vector<std::uint8_t>* bytes,
                       const Array& values) {
  for (const double value : values) {
    appendDouble(bytes, value);
  }
}

template <typename A, typename B>
bool equalDigest(const A& a, const B& b) {
  return std::equal(a.begin(), a.end(), b.begin(), b.end());
}

template <typename Array>
void fillNan(Array* values) {
  std::fill(values->begin(), values->end(), nanValue());
}

gtsam::Vector6 vector6(const boost::array<double, 6>& values) {
  gtsam::Vector6 result;
  for (int i = 0; i < 6; ++i) {
    result(i) = values[static_cast<std::size_t>(i)];
  }
  return result;
}

void copyVector6(const gtsam::Vector6& source,
                 boost::array<double, 6>* destination) {
  for (int i = 0; i < 6; ++i) {
    (*destination)[static_cast<std::size_t>(i)] = source(i);
  }
}

void copyMatrix6(const gtsam::Matrix6& source,
                 boost::array<double, 36>* destination) {
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      (*destination)[static_cast<std::size_t>(row * 6 + col)] =
          source(row, col);
    }
  }
}

bool sameDoubleBits(double lhs, double rhs) {
  std::uint64_t left_bits = 0u;
  std::uint64_t right_bits = 0u;
  std::memcpy(&left_bits, &lhs, sizeof(left_bits));
  std::memcpy(&right_bits, &rhs, sizeof(right_bits));
  return left_bits == right_bits;
}

}  // namespace

sha256::Digest128 beliefPayloadDigest(
    const liorf::pose_odom_belief_array& belief_array,
    std::uint32_t ordinal,
    std::uint64_t publication_sequence) {
  if (ordinal >= belief_array.beliefs.size()) {
    throw std::out_of_range("belief ordinal outside array");
  }
  const auto& belief = belief_array.beliefs[ordinal];
  std::vector<std::uint8_t> bytes;
  const std::string prefix = "GLIM_CBS_BELIEF_PAYLOAD_V1";
  bytes.insert(bytes.end(), prefix.begin(), prefix.end());
  appendBigEndian(&bytes, kSupportedStage2aSchemaVersion);
  appendBigEndian(&bytes, belief_array.header.stamp.sec);
  appendBigEndian(&bytes, belief_array.header.stamp.nsec);
  appendString(&bytes, belief_array.header.frame_id);
  appendBigEndian(&bytes,
                  static_cast<std::uint32_t>(belief.source_agent));
  appendBigEndian(&bytes, belief.from_pose_index);
  appendBigEndian(&bytes, belief.to_pose_index);
  appendDouble(&bytes, belief.from_stamp_sec);
  appendDouble(&bytes, belief.to_stamp_sec);
  appendDoubleArray(&bytes, belief.relative_mu);
  appendDoubleArray(&bytes, belief.covariance);
  appendDouble(&bytes, belief.relax_factor);
  appendBigEndian(&bytes, ordinal);
  appendBigEndian(&bytes, publication_sequence);
  return sha256::prefix128(sha256::hash(bytes));
}

sha256::Digest128 stableEdgeDigest(
    const liorf::pose_odom_belief& belief,
    const std::string& sender_session_uuid) {
  std::vector<std::uint8_t> bytes;
  const std::string prefix = "GLIM_CBS_EDGE_KEY_V1";
  bytes.insert(bytes.end(), prefix.begin(), prefix.end());
  appendString(&bytes, sender_session_uuid);
  appendBigEndian(&bytes,
                  static_cast<std::uint32_t>(belief.source_agent));
  appendBigEndian(&bytes, belief.from_pose_index);
  appendBigEndian(&bytes, belief.to_pose_index);
  appendDouble(&bytes, belief.from_stamp_sec);
  appendDouble(&bytes, belief.to_stamp_sec);
  return sha256::prefix128(sha256::hash(bytes));
}

bool metadataMatchesBeliefArray(
    const liorf::pose_odom_belief_array& belief_array,
    const liorf::DcregEdgeHealthMetadataArray& metadata,
    std::uint32_t* reason_mask) {
  std::uint32_t reasons = liorf::DcregBeliefShadowAnalysis::REASON_NONE;
  if (metadata.schema_version != kSupportedStage2aSchemaVersion ||
      metadata.semantics_version != kSupportedStage2aSemanticsVersion) {
    reasons |= liorf::DcregBeliefShadowAnalysis::
        REASON_METADATA_SCHEMA_UNSUPPORTED;
  }
  if (belief_array.header.stamp != metadata.belief_array_header_stamp ||
      belief_array.header.frame_id != metadata.belief_array_frame_id ||
      belief_array.beliefs.size() != metadata.entries.size()) {
    reasons |= liorf::DcregBeliefShadowAnalysis::
        REASON_METADATA_IDENTITY_CONFLICT;
  }
  if (reasons == liorf::DcregBeliefShadowAnalysis::REASON_NONE) {
    for (const auto& entry : metadata.entries) {
      const std::uint32_t ordinal = entry.belief_ordinal;
      if (ordinal >= belief_array.beliefs.size()) {
        reasons |= liorf::DcregBeliefShadowAnalysis::
            REASON_METADATA_IDENTITY_CONFLICT;
        break;
      }
      const auto& belief = belief_array.beliefs[ordinal];
      const bool identity_matches =
          belief.source_agent == metadata.source_agent &&
          belief.from_pose_index == entry.from_pose_index &&
          belief.to_pose_index == entry.to_pose_index &&
          sameDoubleBits(belief.from_stamp_sec, entry.from_stamp_sec) &&
          sameDoubleBits(belief.to_stamp_sec, entry.to_stamp_sec);
      if (!identity_matches) {
        reasons |= liorf::DcregBeliefShadowAnalysis::
            REASON_METADATA_IDENTITY_CONFLICT;
        break;
      }
      const auto payload_digest = beliefPayloadDigest(
          belief_array, ordinal, metadata.belief_publication_sequence);
      const auto edge_digest = stableEdgeDigest(
          belief, metadata.sender_session_uuid);
      if (!equalDigest(payload_digest, entry.belief_payload_digest) ||
          !equalDigest(edge_digest, entry.stable_edge_digest)) {
        reasons |= liorf::DcregBeliefShadowAnalysis::
            REASON_PAYLOAD_DIGEST_MISMATCH;
        break;
      }
    }
  }
  if (reason_mask) {
    *reason_mask = reasons;
  }
  return reasons == liorf::DcregBeliefShadowAnalysis::REASON_NONE;
}

DisagreementResult computeDisagreement(
    const gtsam::Pose3& sender_relative_base,
    const gtsam::Pose3& receiver_relative) {
  DisagreementResult result;
  try {
    result.residual = gtsam::Pose3::Logmap(
        sender_relative_base.inverse() * receiver_relative);
    result.valid = result.residual.allFinite();
    if (result.valid) {
      result.rotation_norm = result.residual.head<3>().norm();
      result.translation_norm = result.residual.tail<3>().norm();
    }
  } catch (...) {
    result.valid = false;
  }
  return result;
}

std::uint8_t timestampMatchType(
    const double absolute_timestamp_error_sec) {
  if (!std::isfinite(absolute_timestamp_error_sec) ||
      absolute_timestamp_error_sec < 0.0) {
    return liorf::DcregBeliefShadowAnalysis::TIMESTAMP_MATCH_UNAVAILABLE;
  }
  return absolute_timestamp_error_sec <= 1.0e-12
             ? liorf::DcregBeliefShadowAnalysis::TIMESTAMP_MATCH_EXACT
             : liorf::DcregBeliefShadowAnalysis::TIMESTAMP_MATCH_NEAREST;
}

NormalizedDisagreementResult computeNormalizedDisagreement(
    const gtsam::Vector6& residual,
    const gtsam::Matrix6& sender_covariance,
    const gtsam::Matrix6& receiver_relative_covariance,
    const Config& config) {
  NormalizedDisagreementResult result;
  if (!residual.allFinite() || !sender_covariance.allFinite() ||
      !receiver_relative_covariance.allFinite()) {
    return result;
  }
  gtsam::Matrix6 diagnostic =
      sender_covariance +
      config.beta_receiver_covariance * receiver_relative_covariance;
  diagnostic.diagonal().head<3>().array() += config.model_floor_rotation;
  diagnostic.diagonal().tail<3>().array() += config.model_floor_translation;
  diagnostic = 0.5 * (diagnostic + diagnostic.transpose());

  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> solver(diagnostic);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return result;
  }
  const double maximum = solver.eigenvalues().maxCoeff();
  if (!(maximum > 0.0)) {
    return result;
  }
  const double negative_tolerance =
      config.negative_eigenvalue_tolerance * std::max(1.0, maximum);
  if (solver.eigenvalues().minCoeff() < -negative_tolerance) {
    return result;
  }
  const double threshold =
      config.pseudoinverse_relative_threshold * maximum;
  gtsam::Vector6 inverse_eigenvalues = gtsam::Vector6::Zero();
  double minimum_retained = maximum;
  for (int i = 0; i < 6; ++i) {
    const double value = std::max(0.0, solver.eigenvalues()(i));
    if (value > threshold) {
      inverse_eigenvalues(i) = 1.0 / value;
      minimum_retained = std::min(minimum_retained, value);
      ++result.rank;
    }
  }
  if (result.rank == 0) {
    return result;
  }
  const gtsam::Matrix6 pseudoinverse =
      solver.eigenvectors() * inverse_eigenvalues.asDiagonal() *
      solver.eigenvectors().transpose();
  result.value = residual.dot(pseudoinverse * residual);
  result.condition = maximum / minimum_retained;
  result.regularization =
      config.model_floor_rotation + config.model_floor_translation;
  result.valid = std::isfinite(result.value) && result.value >= -1.0e-12;
  if (result.valid && result.value < 0.0) {
    result.value = 0.0;
  }
  return result;
}

std::uint8_t classifySenderHealth(
    const liorf::DcregEdgeHealthMetadata* metadata) {
  using Message = liorf::DcregBeliefShadowAnalysis;
  if (!metadata) {
    return Message::HEALTH_METADATA_UNAVAILABLE;
  }
  if (!metadata->association_valid || !metadata->association_complete) {
    return Message::HEALTH_INVALID;
  }
  if (metadata->absolute_degeneracy_valid &&
      (metadata->rotational_absolute_degenerate_count > 0u ||
       metadata->translational_absolute_degenerate_count > 0u)) {
    return Message::HEALTH_ABSOLUTELY_DEGENERATE;
  }
  if (!metadata->relative_health_valid || !metadata->reference_ready ||
      !metadata->reference_consistent) {
    return Message::HEALTH_REFERENCE_UNAVAILABLE;
  }
  if (metadata->rotational_degraded_count > 0u ||
      metadata->translational_degraded_count > 0u) {
    return Message::HEALTH_DEGRADED;
  }
  return Message::HEALTH_HEALTHY;
}

liorf::DcregBeliefShadowAnalysis makeShadowEntry(
    const liorf::pose_odom_belief& belief,
    std::uint32_t belief_ordinal,
    const liorf::DcregEdgeHealthMetadata* metadata,
    const ExternalBeliefMatchDiagnostic* receiver_observation,
    const std::string& sender_session_uuid,
    std::uint64_t publication_sequence,
    const std::string& receiver_session_uuid,
    std::uint64_t shadow_sequence,
    const Config& config) {
  liorf::DcregBeliefShadowAnalysis output;
  output.sender_session_uuid = sender_session_uuid;
  output.belief_publication_sequence = publication_sequence;
  output.belief_ordinal = belief_ordinal;
  if (metadata) {
    output.stable_edge_digest = metadata->stable_edge_digest;
    output.belief_payload_digest = metadata->belief_payload_digest;
    output.stage2a_metadata_available = true;
    output.stage2a_metadata = *metadata;
    output.belief_metadata_match_valid = true;
  }
  output.receiver_session_uuid = receiver_session_uuid;
  output.sender_from_pose_index = belief.from_pose_index;
  output.sender_to_pose_index = belief.to_pose_index;
  output.sender_from_stamp_sec = belief.from_stamp_sec;
  output.sender_to_stamp_sec = belief.to_stamp_sec;
  output.sender_interval_duration_sec =
      belief.to_stamp_sec - belief.from_stamp_sec;
  output.sender_lidar_health_state = classifySenderHealth(metadata);
  output.shadow_sequence = shadow_sequence;
  output.creation_stamp = ros::Time::now();
  fillNan(&output.sender_relative_mu_base);
  fillNan(&output.receiver_relative_mu);
  fillNan(&output.disagreement_residual);
  fillNan(&output.sender_covariance_base);
  fillNan(&output.receiver_relative_covariance);
  output.rotational_disagreement_norm_rad = nanValue();
  output.translational_disagreement_norm_m = nanValue();
  output.receiver_from_stamp_sec = nanValue();
  output.receiver_to_stamp_sec = nanValue();
  output.start_timestamp_error_sec = nanValue();
  output.end_timestamp_error_sec = nanValue();
  output.receiver_interval_duration_sec = nanValue();
  output.interval_duration_error_sec = nanValue();
  output.normalized_disagreement = nanValue();
  output.diagnostic_covariance_condition = nanValue();

  if (!metadata) {
    output.invalid_reason_mask |=
        liorf::DcregBeliefShadowAnalysis::REASON_METADATA_MISSING;
  }
  if (!receiver_observation) {
    output.invalid_reason_mask |=
        liorf::DcregBeliefShadowAnalysis::REASON_RECEIVER_MATCH_MISSING;
    return output;
  }

  output.receiver_belief_receipt_sequence =
      receiver_observation->shadow_receipt_sequence;
  output.receiver_match_status =
      static_cast<std::uint8_t>(receiver_observation->status);
  output.invalid_reason_mask |=
      receiver_observation->shadow_invalid_reason_mask;
  output.active_factor_accepted =
      receiver_observation->status ==
      ExternalBeliefMatchDiagnostic::Status::Accepted;
  output.receiver_from_pose_index =
      receiver_observation->receiver_from_frame_id;
  output.receiver_to_pose_index = receiver_observation->receiver_to_frame_id;
  output.receiver_from_stamp_sec =
      receiver_observation->receiver_from_stamp_sec;
  output.receiver_to_stamp_sec = receiver_observation->receiver_to_stamp_sec;
  output.start_timestamp_error_sec =
      receiver_observation->start_abs_timestamp_error_sec;
  output.end_timestamp_error_sec =
      receiver_observation->end_abs_timestamp_error_sec;
  output.receiver_interval_duration_sec =
      output.receiver_to_stamp_sec - output.receiver_from_stamp_sec;
  output.interval_duration_error_sec =
      receiver_observation->interval_duration_error_sec;
  output.start_timestamp_match_type =
      timestampMatchType(output.start_timestamp_error_sec);
  output.end_timestamp_match_type =
      timestampMatchType(output.end_timestamp_error_sec);
  output.start_ambiguous_match_count =
      receiver_observation->start_ambiguous_match_count;
  output.end_ambiguous_match_count =
      receiver_observation->end_ambiguous_match_count;

  if (receiver_observation->status ==
          ExternalBeliefMatchDiagnostic::Status::Rejected ||
      receiver_observation->status ==
          ExternalBeliefMatchDiagnostic::Status::Superseded) {
    output.invalid_reason_mask |=
        liorf::DcregBeliefShadowAnalysis::REASON_RECEIVER_MATCH_REJECTED;
  }
  if (!receiver_observation->sender_frame_conversion_available) {
    output.invalid_reason_mask |= liorf::DcregBeliefShadowAnalysis::
        REASON_FRAME_CONVERSION_UNAVAILABLE;
    return output;
  }
  if (!receiver_observation->sender_relative_pose_available) {
    output.invalid_reason_mask |=
        liorf::DcregBeliefShadowAnalysis::REASON_INVALID_POSE;
    return output;
  }
  const gtsam::Pose3 sender_relative =
      receiver_observation->sender_relative_pose;
  try {
    copyVector6(gtsam::Pose3::Logmap(sender_relative),
                &output.sender_relative_mu_base);
  } catch (...) {
    output.invalid_reason_mask |=
        liorf::DcregBeliefShadowAnalysis::REASON_INVALID_POSE;
  }
  if (receiver_observation->sender_covariance_available) {
    output.sender_covariance_available = true;
    copyMatrix6(receiver_observation->sender_covariance,
                &output.sender_covariance_base);
  }

  output.receiver_state_match_valid =
      receiver_observation->receiver_pose_available;
  if (!output.receiver_state_match_valid) {
    output.invalid_reason_mask |=
        liorf::DcregBeliefShadowAnalysis::REASON_RECEIVER_POSE_UNAVAILABLE;
    return output;
  }

  gtsam::Pose3 receiver_relative;
  try {
    receiver_relative = receiver_observation->receiver_from_pose.between(
        receiver_observation->receiver_to_pose);
    copyVector6(gtsam::Pose3::Logmap(receiver_relative),
                &output.receiver_relative_mu);
  } catch (...) {
    output.invalid_reason_mask |=
        liorf::DcregBeliefShadowAnalysis::REASON_INVALID_POSE;
    return output;
  }
  if (config.compute_raw_disagreement) {
    const DisagreementResult disagreement =
        computeDisagreement(sender_relative, receiver_relative);
    output.raw_disagreement_available = disagreement.valid;
    if (disagreement.valid) {
      copyVector6(disagreement.residual, &output.disagreement_residual);
      output.rotational_disagreement_norm_rad = disagreement.rotation_norm;
      output.translational_disagreement_norm_m =
          disagreement.translation_norm;
    } else {
      output.invalid_reason_mask |=
          liorf::DcregBeliefShadowAnalysis::REASON_INVALID_POSE;
    }
  }

  // Exact receiver relative covariance is intentionally unavailable in
  // Stage 2B: obtaining it would require an added marginal query on Kimera's
  // optimization thread. Consequently no normalized diagnostic is emitted.
  if (config.compute_normalized_disagreement) {
    output.invalid_reason_mask |= liorf::DcregBeliefShadowAnalysis::
        REASON_NORMALIZED_DIAGNOSTIC_UNAVAILABLE;
  }
  return output;
}

GroundTruthEdgeEvaluation evaluateGroundTruthEdge(
    const gtsam::Pose3& sender_relative,
    const gtsam::Pose3& receiver_relative,
    const gtsam::Pose3& ground_truth_relative) {
  GroundTruthEdgeEvaluation result;
  try {
    result.sender_error = gtsam::Pose3::Logmap(
        ground_truth_relative.inverse() * sender_relative);
    result.receiver_error = gtsam::Pose3::Logmap(
        ground_truth_relative.inverse() * receiver_relative);
    result.valid = result.sender_error.allFinite() &&
                   result.receiver_error.allFinite();
    if (result.valid) {
      result.sender_rotation_error = result.sender_error.head<3>().norm();
      result.sender_translation_error = result.sender_error.tail<3>().norm();
      result.receiver_rotation_error = result.receiver_error.head<3>().norm();
      result.receiver_translation_error =
          result.receiver_error.tail<3>().norm();
    }
  } catch (...) {
    result.valid = false;
  }
  return result;
}

}  // namespace VIO::dcreg_shadow
