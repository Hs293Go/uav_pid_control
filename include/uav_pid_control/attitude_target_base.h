// Copyright (c) 2023 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef UAV_PID_CONTROL_ATTITUDE_TARGET_BASE_H_
#define UAV_PID_CONTROL_ATTITUDE_TARGET_BASE_H_

#include <algorithm>
#include <cmath>

#include "Eigen/Dense"
#include "internal/traits.h"

namespace control {

template <typename Scalar>
struct AttitudeTarget {
  Eigen::Quaternion<Scalar> orientation;
  Scalar thrust;
};

template <typename Derived>
class AttitudeTargetBase {
 public:
  using Scalar = typename internal::traits<Derived>::Scalar;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using AttitudeTargetType = AttitudeTarget<Scalar>;

  [[nodiscard]] Derived& derived() { return static_cast<Derived&>(*this); }
  [[nodiscard]] const Derived& derived() const {
    return static_cast<const Derived&>(*this);
  }

  [[nodiscard]] AttitudeTargetType computeAttitudeTarget(
      const Vector3& thrust_sp, Scalar yaw_sp) {
    const Scalar thrust = thrust_sp.norm();
    AttitudeTargetType res;
    if (thrust < Eigen::NumTraits<Scalar>::epsilon()) {
      res.thrust = Scalar{0};
      res.orientation.setIdentity();
    } else {
      res.thrust = thrust;
      res.orientation = vectorToOrientation(thrust_sp / thrust, yaw_sp);
    }
    return res;
  }

 private:
  [[nodiscard]] static Quaternion vectorToOrientation(const Vector3& vector,
                                                      Scalar yaw) {
    using std::abs;
    using std::cos;
    using std::sin;

    Matrix3 r;
    r.col(2) = vector;

    // Check magnitude and sign of attitude_mat[2, 2] for
    // in case commanded thrust is fully lateral or upside down
    const Scalar r_22 = r.coeff(2, 2);
    if (abs(r_22) > Eigen::NumTraits<Scalar>::epsilon()) {
      // vector of desired yaw direction
      const Eigen::Vector3d heading_vector(-sin(yaw), cos(yaw), 0.0);
      // desired body_x axis, orthogonal to body_z
      r.col(0) =
          (r_22 < Scalar{0} ? -heading_vector : heading_vector).cross(r.col(2));
      r.col(0).normalize();

    } else {
      // Desired thrust is fully lateral, set x-axis down
      // yaw setpoint is discarded
      r.col(0) = Vector3::UnitZ();
    }

    // body-frame y-axis is simply tne x-axis cross z-axis, normalized
    r.col(1) = r.col(2).cross(r.col(0));
    r.col(1).normalize();

    // ROS uses quaternion internally, so convert the matrix to quaternion
    const Quaternion res(r);
    return res;
  }
};
}  // namespace control

#endif  // UAV_PID_CONTROL_ATTITUDE_TARGET_BASE_H_
