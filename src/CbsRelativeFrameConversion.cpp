#include "kimera_vio_ros/CbsRelativeFrameConversion.h"

namespace VIO::cbs_frame_conversion {

ConvertedRelativeBelief convertExternalToBase(
    const gtsam::Pose3& external_relative,
    const gtsam::Matrix6& external_covariance,
    const gtsam::Pose3& base_T_external,
    const gtsam::Pose3& external_T_base) {
  ConvertedRelativeBelief converted;
  converted.relative_pose =
      base_T_external * external_relative * external_T_base;
  const gtsam::Matrix6 adjoint_base_external =
      base_T_external.AdjointMap();
  converted.covariance =
      adjoint_base_external * external_covariance *
      adjoint_base_external.transpose();
  return converted;
}

}  // namespace VIO::cbs_frame_conversion
