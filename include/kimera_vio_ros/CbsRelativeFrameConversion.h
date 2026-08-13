#pragma once

#include <gtsam/geometry/Pose3.h>

namespace VIO::cbs_frame_conversion {

struct ConvertedRelativeBelief {
  gtsam::Pose3 relative_pose;
  gtsam::Matrix6 covariance = gtsam::Matrix6::Zero();
};

// Converts a right-local relative-pose belief from an external body frame to
// Kimera's base frame. The transform arguments are exact inverses and are
// supplied separately to preserve the production receiver's operation order.
ConvertedRelativeBelief convertExternalToBase(
    const gtsam::Pose3& external_relative,
    const gtsam::Matrix6& external_covariance,
    const gtsam::Pose3& base_T_external,
    const gtsam::Pose3& external_T_base);

}  // namespace VIO::cbs_frame_conversion
