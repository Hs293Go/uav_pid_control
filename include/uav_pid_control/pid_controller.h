// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef UAV_PID_CONTROL_PID_CONTROLLER_H_
#define UAV_PID_CONTROL_PID_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <type_traits>

#include "Eigen/Dense"
#include "uav_pid_control/attitude_target_base.h"

#define XY_(vec) ((vec).template head<2>())

namespace control {

enum class AltitudeTrackingMode { kPosition, kVelocity };

template <typename ScalarType>
class PIDController : public AttitudeTargetBase<PIDController<ScalarType>> {
 public:
  using Base = AttitudeTargetBase<PIDController<ScalarType>>;
  using Base::computeAttitudeTarget;
  using ControlOutputType = typename Base::AttitudeTargetType;

  using Scalar = ScalarType;
  using Vector3 = typename Base::Vector3;
  using Matrix3 = typename Base::Matrix3;

  struct Params {
    Vector3 kp_pos{Vector3::Ones()};
    Vector3 kd_pos{Vector3::Ones()};
    Vector3 kp_vel{Vector3::Ones()};
    Vector3 ki_vel{Vector3::Zero()};
    Vector3 kd_vel{Vector3::Zero()};
    Scalar filt_coeff{0.5};
    Scalar hover_thrust{9.80665};
    Scalar min_thrust{0.0};
    Scalar max_thrust{20.0};
    Scalar max_tilt_angle;
    Vector3 max_vel_sp;
  };

  PIDController() = default;

  bool loadParams(const Params& params);

  /**
   * @brief
   *
   * @param dt
   * @return ControlOutputType
   */
  [[nodiscard]] ControlOutputType computeControlOutput(Scalar dt);

  AltitudeTrackingMode altitude_tracking_mode() {
    return altitude_tracking_mode_;
  }

  /**
   * @brief Gets/sets the absolute position of the UAV
   *
   * @details This is the position of the origin of the UAV body-fixed frame
   * relative that of the inertial frame, expressed in the inertial frame
   * (ENU/NED)
   *
   * @return Vector3& Absolute position of the UAV
   */
  [[nodiscard]] Vector3& position() { return position_; }

  /**
   * @brief Gets/sets the absolute velocity of the UAV
   *
   * @details This is the velocity of the UAV body-fixed frame relative to the
   * inertial frame, measured in the inertial frame (ENU/NED)
   *
   * @return Vector3& Absolute velocity of the UAV
   */
  [[nodiscard]] Vector3& velocity() { return velocity_; }

  /**
   * @brief Gets/sets the feedforward term
   *
   * @return Vector3&
   */
  [[nodiscard]] Vector3& acceleration_ff() { return acceleration_ff_; }

  /**
   * @brief
   *
   * @return Vector3&
   */
  [[nodiscard]] Vector3& velocity_ff() { return velocity_ff_; }

  [[nodiscard]] Vector3& position_sp() { return position_sp_; }

  [[nodiscard]] Scalar& yaw_sp() { return yaw_sp_; }

  [[nodiscard]] bool& pause_integration() { return pause_integration_; }

  [[nodiscard]] const Vector3& position_error() const {
    return position_error_;
  }
  [[nodiscard]] const Vector3& velocity_error() const {
    return velocity_error_;
  }

 private:
  // Position control loop [Input: current pos, desired pos; Output: desired
  // vel]
  Vector3 positionControlLoop(const Vector3& position_sp);

  // Velocity control loop [Input: current vel, desired vel; Output: desired
  // thrust]
  Vector3 velocityControlLoop(const Vector3& velocity_sp, Scalar dt);

  Vector3 computeVelocityErrorDerivative(const Vector3& vel_err, Scalar dt);

  void updateVelocityErrorIntegral(const Vector3& raw_thrust_sp,
                                   const Vector3& thrust_vec_sp,

                                   Scalar dt);

  [[nodiscard]] Vector3 thrustSetpointShaping(
      const Vector3& raw_thrust_sp) const;
  // Controller parameters
  Vector3 kp_pos_;
  Vector3 kd_pos_;
  Vector3 kp_vel_;
  Vector3 ki_vel_;
  Vector3 kd_vel_;
  Scalar filt_coeff_;

  // Bounds
  Scalar min_thrust_{0.0};
  Scalar max_thrust_{1000.0};
  Scalar max_thrust_sq_;
  Scalar max_tilt_angle_;
  Vector3 max_vel_sp_;
  bool params_set_{false};

  // States
  Vector3 acceleration_ff_{Vector3::Zero()};
  Vector3 velocity_ff_{Vector3::Zero()};
  Vector3 position_{Vector3::Zero()};
  Vector3 velocity_{Vector3::Zero()};
  Vector3 position_error_{Vector3::Zero()};
  Vector3 velocity_error_{Vector3::Zero()};

  // Inputs
  Vector3 position_sp_;
  Scalar yaw_sp_;
  bool pause_integration_{false};
  AltitudeTrackingMode altitude_tracking_mode_{AltitudeTrackingMode::kPosition};

  // Memory terms
  Vector3 iterm_{Vector3::Zero()};
  Vector3 error_vel_dot_last_{Vector3::Zero()};
  Vector3 error_vel_last_{Vector3::Zero()};
};

namespace internal {
template <typename ScalarType>
struct traits<PIDController<ScalarType>> {
  using Scalar = ScalarType;
};

}  // namespace internal

// Implementation of class members
template <typename ScalarType>
bool PIDController<ScalarType>::loadParams(const Params& params) {
  kp_pos_ = params.kp_pos;
  kd_pos_ = params.kd_pos;
  kp_vel_ = params.kp_vel;
  ki_vel_ = params.ki_vel;
  kd_vel_ = params.kd_vel;
  filt_coeff_ = params.filt_coeff;
  acceleration_ff_ = params.hover_thrust * Vector3::UnitZ();
  if (params.min_thrust > params.max_thrust) {
    return false;
  }
  min_thrust_ = params.min_thrust;
  max_thrust_ = params.max_thrust;
  max_thrust_sq_ = max_thrust_ * max_thrust_;
  max_tilt_angle_ = params.max_tilt_angle;
  max_vel_sp_ = params.max_vel_sp;
  params_set_ = true;
  return true;
}

template <typename ScalarType>
auto PIDController<ScalarType>::computeControlOutput(Scalar dt)
    -> ControlOutputType {
  using std::clamp;
  const Vector3 velocity_sp = positionControlLoop(position_sp_);
  const Vector3 thrust_vec_sp = velocityControlLoop(velocity_sp, dt);
  return computeAttitudeTarget(thrust_vec_sp, yaw_sp_);
}

template <typename ScalarType>
auto PIDController<ScalarType>::positionControlLoop(const Vector3& position_sp)
    -> Vector3 {
  Vector3 setpoint = velocity_ff_;
  position_error_ = position_sp - position_;
  switch (altitude_tracking_mode_) {
    case AltitudeTrackingMode::kPosition:
      setpoint += kp_pos_.cwiseProduct(position_error_) -
                  kd_pos_.cwiseProduct(velocity_);
      break;
    case AltitudeTrackingMode::kVelocity:
      XY_(setpoint) += XY_(kp_pos_).cwiseProduct(XY_(position_error_)) -
                       XY_(kd_pos_).cwiseProduct(XY_(velocity_));
      break;
    default:
      break;
  }
  return setpoint.cwiseMax(-max_vel_sp_).cwiseMin(max_vel_sp_);
}

template <typename ScalarType>
auto PIDController<ScalarType>::velocityControlLoop(const Vector3& velocity_sp,
                                                    Scalar dt) -> Vector3 {
  velocity_error_ = velocity_sp - velocity_;
  const Vector3 pterm = kp_vel_.cwiseProduct(velocity_error_);

  const Vector3 dterm =
      kd_vel_.cwiseProduct(computeVelocityErrorDerivative(velocity_error_, dt));

  // Consider thrust in Z-direction. [Here hover_throttle is incorporated into
  // acceleration_ff]
  const Vector3 raw_thrust_vec_sp = acceleration_ff_ + pterm + iterm_ + dterm;

  // Get maximum allowed thrust in XY based on tilt angle and excess thrust.
  Vector3 thrust_vec_sp = thrustSetpointShaping(raw_thrust_vec_sp);

  updateVelocityErrorIntegral(raw_thrust_vec_sp, thrust_vec_sp, dt);

  return thrust_vec_sp;
}

template <typename ScalarType>
auto PIDController<ScalarType>::computeVelocityErrorDerivative(
    const Vector3& vel_err, Scalar dt) -> Vector3 {
  if (dt < Eigen::NumTraits<Scalar>::epsilon()) {
    return Vector3::Zero();
  }
  constexpr Scalar kPi{3.14159265358979323846L};
  const Vector3 error_vel_dot_now = (vel_err - error_vel_last_) / dt;

  const Scalar b = kPi * filt_coeff_ * dt * Scalar{2};
  const Scalar a = b / (b + Scalar{1});

  Vector3 vel_err_deriv =
      a * error_vel_dot_now + (1.0 - a) * error_vel_dot_last_;

  error_vel_last_ = vel_err;
  error_vel_dot_last_ = vel_err_deriv;
  return vel_err_deriv;
}
template <typename ScalarType>
void PIDController<ScalarType>::updateVelocityErrorIntegral(
    const Vector3& raw_thrust_sp, const Vector3& thrust_vec_sp, Scalar dt) {
  using std::abs;
  using std::copysign;
  using std::min;
  if (pause_integration_) {
    return;
  }

  // Anti-Reset Windup for/ PID controllers, L.Rundqwist, 1990
  const Scalar arw_gain = Scalar{2} / kp_vel_.x();

  // Update integral
  XY_(iterm_) += XY_(ki_vel_).cwiseProduct(
                     XY_(velocity_error_) -
                     (XY_(raw_thrust_sp) - XY_(thrust_vec_sp)) * arw_gain) *
                 dt;

  // Apply Anti-Windup in Z-direction.
  const bool stop_integral_z =
      (raw_thrust_sp.z() > max_thrust_ && velocity_error_.z() >= Scalar{0}) ||
      (raw_thrust_sp.z() < min_thrust_ && velocity_error_.z() <= Scalar{0});
  if (!stop_integral_z) {
    iterm_.z() += ki_vel_.z() * velocity_error_.z() * dt;

    // limit thrust integral
    iterm_.z() = copysign(min(abs(iterm_.z()), max_thrust_), iterm_.z());
  }
}

template <typename ScalarType>
auto PIDController<ScalarType>::thrustSetpointShaping(
    const Vector3& raw_thrust_sp) const -> Vector3 {
  using std::abs;
  using std::copysign;
  using std::max;
  using std::min;
  using std::sqrt;
  Scalar max_lateral_thrust;

  // Check thrust setpoint against threshold of max thrust magnitude
  Scalar thrust_sp_z = raw_thrust_sp.z();
  if (thrust_sp_z < max_thrust_) {
    thrust_sp_z = max(thrust_sp_z, min_thrust_);
    // Lift does not saturate actuators, aim to deliver requested lift
    // while scaling back lateral thrust
    const Scalar max_thrust_sq = max_thrust_ * max_thrust_;
    max_lateral_thrust = sqrt(max_thrust_sq - thrust_sp_z * thrust_sp_z);
  } else {
    // Lift alone saturates actuators, deliver as much lift as possible and
    // no lateral thrust
    return Vector3::UnitZ() * max_thrust_;
  }

  // Check thrust setpoint against tilt angle limits (i.e. angular deviation
  // of the thrusting orientation from the vertical)
  if (max_tilt_angle_ > 0.0) {
    const Scalar max_tilt_lateral_thrust =
        abs(thrust_sp_z) * tan(max_tilt_angle_);
    max_lateral_thrust = min(max_lateral_thrust, max_tilt_lateral_thrust);
  }

  const Scalar lateral_thrust_sqnorm =
      raw_thrust_sp.template head<2>().squaredNorm();
  if (lateral_thrust_sqnorm > max_lateral_thrust * max_lateral_thrust) {
    Vector3 thrust_sp{raw_thrust_sp};
    thrust_sp.template head<2>() *=
        (max_lateral_thrust / sqrt(lateral_thrust_sqnorm));
    return thrust_sp;
  }
  return raw_thrust_sp;
}
}  // namespace control
#endif  // UAV_PID_CONTROL_PID_CONTROLLER_H_
