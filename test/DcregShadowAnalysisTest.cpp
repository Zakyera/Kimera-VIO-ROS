#include <gtest/gtest.h>
#include <glog/logging.h>

#include <gtsam/geometry/Rot3.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <thread>

#include <ros/serialization.h>

#include "kimera_vio_ros/CbsRelativeFrameConversion.h"
#include "kimera_vio_ros/DcregSha256.h"
#include "kimera_vio_ros/DcregShadowAnalysis.h"
#include "kimera_vio_ros/NonBlockingSpscQueue.h"

namespace VIO::dcreg_shadow {
namespace {

liorf::pose_odom_belief_array beliefArray() {
  liorf::pose_odom_belief_array array;
  array.header.stamp.sec = 10u;
  array.header.stamp.nsec = 20u;
  array.header.frame_id = "map";
  liorf::pose_odom_belief belief;
  belief.header = array.header;
  belief.source_agent = static_cast<std::uint8_t>('g');
  belief.from_pose_index = 1u;
  belief.to_pose_index = 2u;
  belief.from_stamp_sec = 1.0;
  belief.to_stamp_sec = 2.0;
  array.beliefs.push_back(belief);
  return array;
}

liorf::DcregEdgeHealthMetadataArray metadataArray(
    const liorf::pose_odom_belief_array& beliefs) {
  liorf::DcregEdgeHealthMetadataArray metadata;
  metadata.header = beliefs.header;
  metadata.schema_version = 1u;
  metadata.semantics_version = 1u;
  metadata.sender_session_uuid =
      "11111111-2222-4333-8444-555555555555";
  metadata.source_agent = static_cast<std::uint8_t>('g');
  metadata.belief_publication_sequence = 1u;
  metadata.belief_array_header_stamp = beliefs.header.stamp;
  metadata.belief_array_frame_id = beliefs.header.frame_id;
  for (std::uint32_t ordinal = 0u; ordinal < beliefs.beliefs.size();
       ++ordinal) {
    const auto& belief = beliefs.beliefs[ordinal];
    liorf::DcregEdgeHealthMetadata entry;
    entry.belief_ordinal = ordinal;
    entry.from_pose_index = belief.from_pose_index;
    entry.to_pose_index = belief.to_pose_index;
    entry.from_stamp_sec = belief.from_stamp_sec;
    entry.to_stamp_sec = belief.to_stamp_sec;
    const auto edge_digest =
        stableEdgeDigest(belief, metadata.sender_session_uuid);
    std::copy(edge_digest.begin(),
              edge_digest.end(),
              entry.stable_edge_digest.begin());
    const auto payload_digest = beliefPayloadDigest(
        beliefs, ordinal, metadata.belief_publication_sequence);
    std::copy(payload_digest.begin(),
              payload_digest.end(),
              entry.belief_payload_digest.begin());
    entry.association_valid = true;
    entry.association_complete = true;
    entry.reference_ready = true;
    entry.reference_consistent = true;
    entry.relative_health_valid = true;
    entry.rotational_health_minimum = 0.9;
    entry.rotational_health_median = 0.95;
    entry.translational_health_minimum = 0.8;
    entry.translational_health_median = 0.85;
    metadata.entries.push_back(entry);
  }
  return metadata;
}

ExternalBeliefMatchDiagnostic receiverObservation() {
  ExternalBeliefMatchDiagnostic observation;
  observation.shadow_identity_valid = true;
  observation.shadow_receipt_sequence = 7u;
  observation.shadow_belief_ordinal = 0u;
  observation.source_agent = static_cast<std::uint8_t>('g');
  observation.sender_from_pose_index = 1u;
  observation.sender_to_pose_index = 2u;
  observation.sender_from_stamp_sec = 1.0;
  observation.sender_to_stamp_sec = 2.0;
  observation.receiver_from_frame_id = 10u;
  observation.receiver_to_frame_id = 12u;
  observation.receiver_from_stamp_sec = 1.01;
  observation.receiver_to_stamp_sec = 2.01;
  observation.start_abs_timestamp_error_sec = 0.01;
  observation.end_abs_timestamp_error_sec = 0.01;
  observation.interval_duration_error_sec = 0.0;
  observation.receiver_pose_available = true;
  observation.receiver_from_pose = gtsam::Pose3();
  observation.receiver_to_pose =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.1, 0.0, 0.0));
  observation.sender_relative_pose =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
  observation.sender_relative_pose_available = true;
  observation.sender_covariance = 0.1 * gtsam::Matrix6::Identity();
  observation.sender_covariance_available = true;
  observation.status = ExternalBeliefMatchDiagnostic::Status::Accepted;
  observation.terminal = true;
  return observation;
}

TEST(Sha256, PublishedStandardVectors) {
  EXPECT_EQ(sha256::hex(sha256::hash("")),
            "e3b0c44298fc1c149afbf4c8996fb924"
            "27ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(sha256::hex(sha256::hash("abc")),
            "ba7816bf8f01cfea414140de5dae2223"
            "b00361a396177a9cb410ff61f20015ad");
}

TEST(Identity, IndependentlyReproducesLockedStage2aPayloadVector) {
  const auto beliefs = beliefArray();
  EXPECT_EQ(sha256::hex(beliefPayloadDigest(beliefs, 0u, 1u)),
            "b0703565d09b0df452313300c32e8308");
}

TEST(Identity, PublicationSequenceAndPayloadChangeDigest) {
  auto beliefs = beliefArray();
  const auto first = beliefPayloadDigest(beliefs, 0u, 1u);
  EXPECT_NE(first, beliefPayloadDigest(beliefs, 0u, 2u));
  beliefs.beliefs[0].relative_mu[0] = std::nextafter(0.0, 1.0);
  EXPECT_NE(first, beliefPayloadDigest(beliefs, 0u, 1u));
}

TEST(Identity, SessionRestartChangesStableEdgeOnly) {
  const auto beliefs = beliefArray();
  EXPECT_NE(stableEdgeDigest(beliefs.beliefs[0], "session-a"),
            stableEdgeDigest(beliefs.beliefs[0], "session-b"));
}

TEST(Association, ExactPayloadMatches) {
  const auto beliefs = beliefArray();
  const auto metadata = metadataArray(beliefs);
  std::uint32_t reason = 99u;
  EXPECT_TRUE(metadataMatchesBeliefArray(beliefs, metadata, &reason));
  EXPECT_EQ(reason, 0u);
}

TEST(Association, DigestMismatchIsRejected) {
  const auto beliefs = beliefArray();
  auto metadata = metadataArray(beliefs);
  metadata.entries[0].belief_payload_digest[0] ^= 1u;
  std::uint32_t reason = 0u;
  EXPECT_FALSE(metadataMatchesBeliefArray(beliefs, metadata, &reason));
  EXPECT_NE(reason & liorf::DcregBeliefShadowAnalysis::
                         REASON_PAYLOAD_DIGEST_MISMATCH,
            0u);
}

TEST(Association, UnsupportedSchemaIsRejected) {
  const auto beliefs = beliefArray();
  auto metadata = metadataArray(beliefs);
  metadata.schema_version = 999u;
  std::uint32_t reason = 0u;
  EXPECT_FALSE(metadataMatchesBeliefArray(beliefs, metadata, &reason));
  EXPECT_NE(reason & liorf::DcregBeliefShadowAnalysis::
                         REASON_METADATA_SCHEMA_UNSUPPORTED,
            0u);
}

TEST(Association, OrdinalOrEndpointConflictIsRejected) {
  const auto beliefs = beliefArray();
  auto metadata = metadataArray(beliefs);
  metadata.entries[0].to_pose_index = 9u;
  std::uint32_t reason = 0u;
  EXPECT_FALSE(metadataMatchesBeliefArray(beliefs, metadata, &reason));
  EXPECT_NE(reason & liorf::DcregBeliefShadowAnalysis::
                         REASON_METADATA_IDENTITY_CONFLICT,
            0u);
}

TEST(Association, ExactDuplicateMetadataRemainsAnExactMatch) {
  const auto beliefs = beliefArray();
  const auto first = metadataArray(beliefs);
  const auto duplicate = first;
  EXPECT_TRUE(metadataMatchesBeliefArray(beliefs, first));
  EXPECT_TRUE(metadataMatchesBeliefArray(beliefs, duplicate));
  EXPECT_EQ(first.entries[0].belief_payload_digest,
            duplicate.entries[0].belief_payload_digest);
}

TEST(Association, ConflictingDuplicatePayloadIsRejected) {
  const auto beliefs = beliefArray();
  auto conflicting = metadataArray(beliefs);
  conflicting.entries[0].belief_payload_digest[3] ^= 0x80u;
  std::uint32_t reason = 0u;
  EXPECT_FALSE(metadataMatchesBeliefArray(beliefs, conflicting, &reason));
  EXPECT_NE(reason & liorf::DcregBeliefShadowAnalysis::
                         REASON_PAYLOAD_DIGEST_MISMATCH,
            0u);
}

TEST(FrameConversion, IdentityExtrinsicIsExact) {
  const gtsam::Pose3 relative(
      gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3), gtsam::Point3(1.0, 2.0, 3.0));
  const gtsam::Matrix6 covariance = 0.2 * gtsam::Matrix6::Identity();
  const auto converted = cbs_frame_conversion::convertExternalToBase(
      relative, covariance, gtsam::Pose3(), gtsam::Pose3());
  EXPECT_TRUE(converted.relative_pose.equals(relative, 0.0));
  EXPECT_EQ((converted.covariance - covariance).norm(), 0.0);
}

TEST(FrameConversion, NontrivialConjugationMatchesProductionFormula) {
  const gtsam::Pose3 base_T_external(
      gtsam::Rot3::RzRyRx(0.3, -0.1, 0.2),
      gtsam::Point3(0.4, -0.2, 0.7));
  const gtsam::Pose3 relative(
      gtsam::Rot3::RzRyRx(-0.2, 0.1, 0.05),
      gtsam::Point3(1.0, 0.2, -0.3));
  const gtsam::Matrix6 covariance =
      gtsam::Matrix6::Identity().selfadjointView<Eigen::Upper>();
  const auto converted = cbs_frame_conversion::convertExternalToBase(
      relative, covariance, base_T_external, base_T_external.inverse());
  EXPECT_TRUE(converted.relative_pose.equals(
      base_T_external * relative * base_T_external.inverse(), 1.0e-14));
  const gtsam::Matrix6 adjoint = base_T_external.AdjointMap();
  EXPECT_LT((converted.covariance -
             adjoint * covariance * adjoint.transpose())
                .norm(),
            1.0e-14);
}

TEST(FrameConversion, RightPerturbationAdjointFiniteDifference) {
  const gtsam::Pose3 base_T_external(
      gtsam::Rot3::RzRyRx(0.2, 0.1, -0.3),
      gtsam::Point3(1.0, -0.5, 0.2));
  const gtsam::Pose3 external_relative(
      gtsam::Rot3::RzRyRx(-0.1, 0.05, 0.2),
      gtsam::Point3(0.3, 0.2, -0.1));
  const gtsam::Vector6 direction =
      (gtsam::Vector6() << 0.2, -0.1, 0.3, 0.4, 0.1, -0.2).finished();
  const double epsilon = 1.0e-7;
  const gtsam::Pose3 base_relative =
      base_T_external * external_relative * base_T_external.inverse();
  const gtsam::Pose3 perturbed =
      base_T_external *
      external_relative.retract(epsilon * direction) *
      base_T_external.inverse();
  const gtsam::Vector6 finite_difference =
      gtsam::Pose3::Logmap(base_relative.inverse() * perturbed) / epsilon;
  EXPECT_LT((finite_difference -
             base_T_external.AdjointMap() * direction)
                .norm(),
            1.0e-7);
}

TEST(FrameConversion, PureTranslationExtrinsicUsesFullAdjoint) {
  const gtsam::Pose3 base_T_external(
      gtsam::Rot3(), gtsam::Point3(1.0, -2.0, 0.5));
  const gtsam::Pose3 external_T_base = base_T_external.inverse();
  const gtsam::Pose3 external_relative(
      gtsam::Rot3::Rz(0.2), gtsam::Point3(0.0, 0.0, 0.0));
  gtsam::Matrix6 covariance = gtsam::Matrix6::Identity();
  const auto converted = cbs_frame_conversion::convertExternalToBase(
      external_relative, covariance, base_T_external, external_T_base);
  EXPECT_TRUE(converted.relative_pose.equals(
      base_T_external * external_relative * external_T_base, 1.0e-14));
  EXPECT_GT((converted.covariance.block<3, 3>(3, 0)).norm(), 0.0);
}

TEST(Disagreement, EqualMotionsProduceZero) {
  const gtsam::Pose3 motion(
      gtsam::Rot3::Ry(0.2), gtsam::Point3(1.0, 2.0, 3.0));
  const auto result = computeDisagreement(motion, motion);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.rotation_norm, 0.0, 1.0e-14);
  EXPECT_NEAR(result.translation_norm, 0.0, 1.0e-14);
}

TEST(Disagreement, PureRotationRemainsSeparateFromTranslation) {
  const auto result = computeDisagreement(
      gtsam::Pose3(),
      gtsam::Pose3(gtsam::Rot3::Rz(0.25), gtsam::Point3(0.0, 0.0, 0.0)));
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.rotation_norm, 0.25, 1.0e-12);
  EXPECT_NEAR(result.translation_norm, 0.0, 1.0e-12);
}

TEST(Disagreement, PureTranslationRemainsSeparateFromRotation) {
  const auto result = computeDisagreement(
      gtsam::Pose3(),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.3, -0.4, 0.0)));
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.rotation_norm, 0.0, 1.0e-12);
  EXPECT_NEAR(result.translation_norm, 0.5, 1.0e-12);
}

TEST(Disagreement, MixedPoseProducesFiniteSeparatedComponents) {
  const gtsam::Pose3 sender(
      gtsam::Rot3::Ry(-0.1), gtsam::Point3(0.5, -0.2, 0.1));
  const gtsam::Pose3 receiver(
      gtsam::Rot3::RzRyRx(0.05, -0.02, 0.08),
      gtsam::Point3(0.6, -0.1, 0.15));
  const auto result = computeDisagreement(sender, receiver);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.rotation_norm));
  EXPECT_TRUE(std::isfinite(result.translation_norm));
  EXPECT_GT(result.rotation_norm, 0.0);
  EXPECT_GT(result.translation_norm, 0.0);
}

TEST(CovarianceDiagnostic, BothValidCovariancesProduceControlledValue) {
  Config config;
  const gtsam::Vector6 residual = gtsam::Vector6::Ones();
  const auto result = computeNormalizedDisagreement(
      residual,
      2.0 * gtsam::Matrix6::Identity(),
      3.0 * gtsam::Matrix6::Identity(),
      config);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.value, 6.0 / 5.0, 1.0e-12);
  EXPECT_EQ(result.rank, 6);
}

TEST(CovarianceDiagnostic, SingularMatrixUsesControlledPseudoinverse) {
  Config config;
  gtsam::Matrix6 sender = gtsam::Matrix6::Zero();
  sender(0, 0) = 2.0;
  const auto result = computeNormalizedDisagreement(
      gtsam::Vector6::Ones(), sender, gtsam::Matrix6::Zero(), config);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.rank, 1);
  EXPECT_NEAR(result.value, 0.5, 1.0e-12);
}

TEST(CovarianceDiagnostic, TrulyIndefiniteMatrixIsInvalid) {
  Config config;
  gtsam::Matrix6 sender = gtsam::Matrix6::Identity();
  sender(0, 0) = -1.0;
  EXPECT_FALSE(computeNormalizedDisagreement(
                   gtsam::Vector6::Ones(),
                   sender,
                   gtsam::Matrix6::Zero(),
                   config)
                   .valid);
}

TEST(CovarianceDiagnostic, SmallNumericalNegativeModeIsTolerated) {
  Config config;
  config.negative_eigenvalue_tolerance = 1.0e-8;
  gtsam::Matrix6 sender = gtsam::Matrix6::Identity();
  sender(0, 0) = -1.0e-10;
  const auto result = computeNormalizedDisagreement(
      gtsam::Vector6::Ones(), sender, gtsam::Matrix6::Zero(), config);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.rank, 5);
}

TEST(CovarianceDiagnostic, NonFiniteInputIsUnavailable) {
  gtsam::Vector6 residual = gtsam::Vector6::Zero();
  residual(2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(computeNormalizedDisagreement(residual,
                                             gtsam::Matrix6::Identity(),
                                             gtsam::Matrix6::Identity(),
                                             Config())
                   .valid);
}

TEST(ReceiverMatching, ExactNearestAndUnavailableRemainDistinct) {
  using Message = liorf::DcregBeliefShadowAnalysis;
  EXPECT_EQ(timestampMatchType(0.0), Message::TIMESTAMP_MATCH_EXACT);
  EXPECT_EQ(timestampMatchType(1.0e-12), Message::TIMESTAMP_MATCH_EXACT);
  EXPECT_EQ(timestampMatchType(std::nextafter(1.0e-12, 1.0)),
            Message::TIMESTAMP_MATCH_NEAREST);
  EXPECT_EQ(timestampMatchType(std::numeric_limits<double>::quiet_NaN()),
            Message::TIMESTAMP_MATCH_UNAVAILABLE);
}

TEST(HealthLabels, MissingInvalidAndReferenceUnavailableStayDistinct) {
  using Message = liorf::DcregBeliefShadowAnalysis;
  EXPECT_EQ(classifySenderHealth(nullptr),
            Message::HEALTH_METADATA_UNAVAILABLE);
  liorf::DcregEdgeHealthMetadata metadata;
  EXPECT_EQ(classifySenderHealth(&metadata), Message::HEALTH_INVALID);
  metadata.association_valid = true;
  metadata.association_complete = true;
  EXPECT_EQ(classifySenderHealth(&metadata),
            Message::HEALTH_REFERENCE_UNAVAILABLE);
}

TEST(HealthLabels, AbsoluteDegradedAndHealthyStayDistinct) {
  using Message = liorf::DcregBeliefShadowAnalysis;
  liorf::DcregEdgeHealthMetadata metadata;
  metadata.association_valid = true;
  metadata.association_complete = true;
  metadata.absolute_degeneracy_valid = true;
  metadata.rotational_absolute_degenerate_count = 1u;
  EXPECT_EQ(classifySenderHealth(&metadata),
            Message::HEALTH_ABSOLUTELY_DEGENERATE);
  metadata.rotational_absolute_degenerate_count = 0u;
  metadata.relative_health_valid = true;
  metadata.reference_ready = true;
  metadata.reference_consistent = true;
  metadata.translational_degraded_count = 1u;
  EXPECT_EQ(classifySenderHealth(&metadata), Message::HEALTH_DEGRADED);
  metadata.translational_degraded_count = 0u;
  EXPECT_EQ(classifySenderHealth(&metadata), Message::HEALTH_HEALTHY);
}

TEST(ShadowEntry, ExactReceiverPairProducesRawResidualOnly) {
  const auto beliefs = beliefArray();
  const auto metadata = metadataArray(beliefs);
  const auto receiver = receiverObservation();
  Config config;
  const auto output = makeShadowEntry(beliefs.beliefs[0],
                                      0u,
                                      &metadata.entries[0],
                                      &receiver,
                                      metadata.sender_session_uuid,
                                      metadata.belief_publication_sequence,
                                      "receiver-session",
                                      1u,
                                      config);
  EXPECT_TRUE(output.belief_metadata_match_valid);
  EXPECT_TRUE(output.receiver_state_match_valid);
  EXPECT_TRUE(output.raw_disagreement_available);
  EXPECT_NEAR(output.rotational_disagreement_norm_rad, 0.0, 1.0e-12);
  EXPECT_NEAR(output.translational_disagreement_norm_m, 0.1, 1.0e-12);
  EXPECT_FALSE(output.receiver_relative_covariance_available);
  EXPECT_FALSE(output.normalized_disagreement_available);
}

TEST(ShadowEntry, MissingMetadataAndReceiverNeverLookHealthy) {
  const auto beliefs = beliefArray();
  Config config;
  const auto output = makeShadowEntry(beliefs.beliefs[0],
                                      0u,
                                      nullptr,
                                      nullptr,
                                      "",
                                      0u,
                                      "receiver-session",
                                      1u,
                                      config);
  EXPECT_EQ(output.sender_lidar_health_state,
            liorf::DcregBeliefShadowAnalysis::HEALTH_METADATA_UNAVAILABLE);
  EXPECT_FALSE(output.raw_disagreement_available);
  EXPECT_NE(output.invalid_reason_mask, 0u);
}

TEST(ShadowEntry, RejectedActiveMatchIsDescriptiveOnly) {
  const auto beliefs = beliefArray();
  const auto metadata = metadataArray(beliefs);
  auto receiver = receiverObservation();
  receiver.status = ExternalBeliefMatchDiagnostic::Status::Rejected;
  receiver.start_ambiguous_match_count = 2u;
  receiver.end_ambiguous_match_count = 1u;
  const auto output = makeShadowEntry(beliefs.beliefs[0],
                                      0u,
                                      &metadata.entries[0],
                                      &receiver,
                                      metadata.sender_session_uuid,
                                      metadata.belief_publication_sequence,
                                      "receiver-session",
                                      4u,
                                      Config());
  EXPECT_FALSE(output.active_factor_accepted);
  EXPECT_TRUE(output.raw_disagreement_available);
  EXPECT_EQ(output.start_ambiguous_match_count, 2u);
  EXPECT_EQ(output.end_ambiguous_match_count, 1u);
  EXPECT_NE(output.invalid_reason_mask &
                liorf::DcregBeliefShadowAnalysis::
                    REASON_RECEIVER_MATCH_REJECTED,
            0u);
}

TEST(ShadowEntry, ReceiveGateRejectionIsExplicitAndHasNoFakeMotion) {
  const auto beliefs = beliefArray();
  const auto metadata = metadataArray(beliefs).entries.front();
  auto receiver = receiverObservation();
  receiver.status = ExternalBeliefMatchDiagnostic::Status::Rejected;
  receiver.receiver_pose_available = false;
  receiver.sender_relative_pose_available = false;
  receiver.shadow_invalid_reason_mask =
      liorf::DcregBeliefShadowAnalysis::REASON_RECEIVE_GATE_REJECTED;
  const auto entry = makeShadowEntry(beliefs.beliefs.front(),
                                     0u,
                                     &metadata,
                                     &receiver,
                                     "sender",
                                     1u,
                                     "receiver",
                                     2u,
                                     Config());
  EXPECT_FALSE(entry.receiver_state_match_valid);
  EXPECT_FALSE(entry.raw_disagreement_available);
  EXPECT_TRUE(entry.invalid_reason_mask &
              liorf::DcregBeliefShadowAnalysis::
                  REASON_RECEIVE_GATE_REJECTED);
  EXPECT_TRUE(entry.invalid_reason_mask &
              liorf::DcregBeliefShadowAnalysis::
                  REASON_RECEIVER_MATCH_REJECTED);
  EXPECT_TRUE(entry.invalid_reason_mask &
              liorf::DcregBeliefShadowAnalysis::REASON_INVALID_POSE);
}

TEST(Passivity, ShadowEntryDoesNotMutateLegacyBeliefBytes) {
  auto beliefs = beliefArray();
  const auto metadata = metadataArray(beliefs);
  const auto receiver = receiverObservation();
  const std::uint32_t length =
      ros::serialization::serializationLength(beliefs);
  std::vector<std::uint8_t> before(length);
  ros::serialization::OStream before_stream(before.data(), before.size());
  ros::serialization::serialize(before_stream, beliefs);
  (void)makeShadowEntry(beliefs.beliefs[0],
                        0u,
                        &metadata.entries[0],
                        &receiver,
                        metadata.sender_session_uuid,
                        metadata.belief_publication_sequence,
                        "receiver-session",
                        5u,
                        Config());
  std::vector<std::uint8_t> after(length);
  ros::serialization::OStream after_stream(after.data(), after.size());
  ros::serialization::serialize(after_stream, beliefs);
  EXPECT_EQ(before, after);
}

TEST(GroundTruth, SenderAndReceiverErrorsUseSeparatePose3Residuals) {
  const gtsam::Pose3 ground_truth(
      gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
  const gtsam::Pose3 sender(
      gtsam::Rot3::Rz(0.1), gtsam::Point3(1.0, 0.0, 0.0));
  const gtsam::Pose3 receiver(
      gtsam::Rot3(), gtsam::Point3(1.2, 0.0, 0.0));
  const auto result = evaluateGroundTruthEdge(sender, receiver, ground_truth);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.sender_rotation_error, 0.1, 1.0e-12);
  EXPECT_NEAR(result.receiver_translation_error, 0.2, 1.0e-12);
}

TEST(Queue, OverflowDropsNewestWithoutChangingQueuedValues) {
  NonBlockingSpscQueue<int> queue(2u);
  EXPECT_TRUE(queue.tryPush(1));
  EXPECT_TRUE(queue.tryPush(2));
  EXPECT_FALSE(queue.tryPush(3));
  int value = 0;
  EXPECT_TRUE(queue.tryPop(&value));
  EXPECT_EQ(value, 1);
  EXPECT_TRUE(queue.tryPop(&value));
  EXPECT_EQ(value, 2);
  EXPECT_FALSE(queue.tryPop(&value));
}

TEST(Queue, SingleProducerConsumerPreservesImmutableOrdering) {
  constexpr int kCount = 2000;
  NonBlockingSpscQueue<int> queue(32u);
  std::atomic<bool> producer_done{false};
  std::thread producer([&]() {
    for (int value = 0; value < kCount;) {
      if (queue.tryPush(value)) {
        ++value;
      }
    }
    producer_done.store(true);
  });
  int expected = 0;
  while (!producer_done.load() || expected < kCount) {
    int value = -1;
    if (queue.tryPop(&value)) {
      EXPECT_EQ(value, expected);
      ++expected;
    }
  }
  producer.join();
  EXPECT_EQ(expected, kCount);
}

TEST(Configuration, PublicDefaultsArePassive) {
  const Config config;
  EXPECT_FALSE(config.enabled);
  EXPECT_TRUE(config.compute_raw_disagreement);
  EXPECT_FALSE(config.compute_receiver_relative_covariance);
  EXPECT_FALSE(config.compute_normalized_disagreement);
  EXPECT_EQ(config.overflow_policy, "drop_newest");
}

}  // namespace
}  // namespace VIO::dcreg_shadow

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  ros::Time::init();
  FLAGS_logtostderr = true;
  return RUN_ALL_TESTS();
}
