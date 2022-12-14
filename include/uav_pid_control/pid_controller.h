// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef UAV_PID_CONTROL_PID_CONTROLLER_H_
#define UAV_PID_CONTROL_PID_CONTROLLER_H_

#include <cmath>

#include <algorithm>
#include <iostream>

#include <type_traits>

#include "Eigen/Dense"

#define XY_(vec) ((vec).template head<2>())

namespace control {

enum class AltitudeTrackingMode { kPosition, kVelocity };

template <typename _Scalar = double>
class PIDController {
 public:
  using Scalar = _Scalar;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Quaternion = Eigen::Quaternion<Scalar>;

  struct Params {
    Vector3 kp_pos{Vector3::Ones()};
    Vector3 kd_pos{Vector3::Ones()};
    Vector3 kp_vel{Vector3::Ones()};
    Vector3 ki_vel{Vector3::Zero()};
    Vector3 kd_vel{Vector3::Zero()};
    Scalar filt_coeff{0.5};
    Scalar motor_slope{0.05};
    Scalar motor_intercept{0.0};
    Scalar hover_throttle{9.80665};
    Scalar min_thrust{0.0};
    Scalar max_thrust{20.0};
    Scalar max_thrust_sq;
    Scalar max_tilt_angle;
    Scalar max_tilt_ratio;
    Vector3 max_vel_sp;
  };

  struct ActuatorModel {
    Scalar intercept{0.0};
    Scalar slope{0.01};
    Scalar hover_throttle{0.5};

    Scalar operator()(Scalar thrust) const {
      return slope * thrust + intercept;
    }
  };

  PIDController() = default;

  bool loadParams(const Params& params) {
    using std::tan;
    kp_pos_ = params.kp_pos;
    kd_pos_ = params.kd_pos;
    kp_vel_ = params.kp_vel;
    ki_vel_ = params.ki_vel;
    kd_vel_ = params.kd_vel;
    filt_coeff_ = params.filt_coeff;
    motor_mdl_.hover_throttle = params.hover_throttle;
    motor_mdl_.intercept = params.motor_intercept;
    motor_mdl_.slope = params.motor_slope;
    if (params.min_thrust > params.max_thrust) {
      return false;
    }
    min_thrust_ = params.min_thrust;
    max_thrust_ = params.max_thrust;
    max_thrust_sq_ = params.max_thrust_sq;
    max_tilt_angle_ = params.max_tilt_angle;
    max_tilt_ratio_ = tan(max_tilt_angle_);
    max_vel_sp_ = params.max_vel_sp;
    params_set_ = true;
    return true;
  }

  // Position control main function
  // [Input: Current state, Reference state, _Reference_State.Sub_mode, dt;
  // Output: AttitudeReference;]
  void run(Scalar dt) {
    const Vector3 velocity_sp = positionControlLoop(position_sp_);
    const Vector3 thrust_sp = velocityControlLoop(velocity_sp, dt);
    computeAttitudeTarget(thrust_sp, yaw_sp_);
  }

  // Position control loop [Input: current pos, desired pos; Output: desired
  // vel]
  Vector3 positionControlLoop(const Vector3& position_sp) {
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
    }
    return setpoint.cwiseMax(-max_vel_sp_).cwiseMin(max_vel_sp_);
  }

  // Velocity control loop [Input: current vel, desired vel; Output: desired
  // thrust]
  Vector3 velocityControlLoop(const Vector3& velocity_sp, Scalar dt) {
    velocity_error_ = velocity_sp - velocity_;
    const Vector3 pterm = kp_vel_.cwiseProduct(velocity_error_);

    const Vector3 dterm = kd_vel_.cwiseProduct(
        computeVelocityErrorDerivative(velocity_error_, dt));

    // Consider thrust in Z-direction. [Here Hover_throttle is added]
    const Vector3 raw_thrust_sp = acceleration_ff_ + pterm + iterm_ + dterm +
                                  motor_mdl_.hover_throttle * Vector3::UnitZ();
    Vector3 thrust_sp = raw_thrust_sp;

    // Get maximum allowed thrust in XY based on tilt angle and excess thrust.
    thrustSetpointShaping(thrust_sp);

    if (!pause_integration_) {
      updateIntegral(raw_thrust_sp, thrust_sp, velocity_error_, dt);
    }

    return thrust_sp;
  }

  AltitudeTrackingMode altitude_tracking_mode() {
    return altitude_tracking_mode_;
  }
  Vector3& position() { return position_; }
  Vector3& velocity() { return velocity_; }
  Vector3& acceleration_ff() { return acceleration_ff_; }
  Vector3& velocity_ff() { return velocity_ff_; }
  Vector3& position_sp() { return position_sp_; }
  Scalar& yaw_sp() { return yaw_sp_; }
  bool& pause_integration() { return pause_integration_; }

  const Vector3& position() const { return position_; }
  const Vector3& velocity() const { return velocity_; }
  const Vector3& acceleration_ff() const { return acceleration_ff_; }
  const Vector3& velocity_ff() const { return velocity_ff_; }
  const Vector3& position_sp() const { return position_sp_; }
  const Vector3& position_error() const { return position_error_; }
  const Vector3& velocity_error() const { return velocity_error_; }
  Scalar yaw_sp() const { return yaw_sp_; }
  bool pause_integration() const { return pause_integration_; }

  const Quaternion& orientation_sp() const { return orientation_sp_; }
  Scalar throttle_sp() const { return throttle_sp_; }

 private:
  Vector3 computeVelocityErrorDerivative(const Vector3& vel_err, Scalar dt) {
    if (dt < Eigen::NumTraits<Scalar>::epsilon()) {
      return Vector3::Zero();
    }
    constexpr Scalar kPi = 3.14159265358979323846L;
    const Vector3 error_vel_dot_now = (vel_err - error_vel_last_) / dt;

    const Scalar b = 2.0 * kPi * filt_coeff_ * dt;
    const Scalar a = b / (1.0 + b);

    const Vector3 vel_err_deriv =
        a * error_vel_dot_now + (1.0 - a) * error_vel_dot_last_;

    error_vel_last_ = vel_err;
    error_vel_dot_last_ = vel_err_deriv;
    return vel_err_deriv;
  }

  void thrustSetpointShaping(Eigen::Vector3d& thrust_sp) const {
    using std::abs;
    using std::copysign;
    using std::max;
    using std::min;
    using std::sqrt;
    Scalar max_lateral_thrust;
    if (thrust_sp.z() < max_thrust_) {
      thrust_sp.z() = max(thrust_sp.z(), min_thrust_);
      // Lift does not saturate actuators, aim to deliver requested lift
      // exactly while scaling back lateral thrust
      max_lateral_thrust = sqrt(max_thrust_sq_ - thrust_sp.z() * thrust_sp.z());
    } else {
      // Lift alone saturates actuators, deliver as much lift as possible and
      // no lateral thrust
      thrust_sp << 0.0, 0.0, max_thrust_;
      return;
    }
    if (max_tilt_angle_ > 0.0) {
      const Scalar max_tilt_lateral_thrust =
          abs(thrust_sp.z()) * max_tilt_ratio_;
      max_lateral_thrust = min(max_lateral_thrust, max_tilt_lateral_thrust);
    }

    const Scalar lateral_thrust_sqnorm = XY_(thrust_sp).squaredNorm();
    if (lateral_thrust_sqnorm > max_lateral_thrust * max_lateral_thrust) {
      XY_(thrust_sp) *= (max_lateral_thrust / sqrt(lateral_thrust_sqnorm));
    }
  }

  void updateIntegral(const Vector3& raw_thrust_sp, const Vector3& thrust_sp,
                      const Vector3& error_vel, Scalar dt) {
    using std::abs;
    using std::min;
    // Anti-Reset Windup for/ PID controllers, L.Rundqwist, 1990
    Scalar arw_gain = Scalar(2) / kp_vel_.x();

    auto scaled_thrust_delta = (XY_(raw_thrust_sp) - XY_(thrust_sp)) * arw_gain;

    // Update integral
    XY_(iterm_) +=
        XY_(ki_vel_).cwiseProduct(XY_(error_vel) - scaled_thrust_delta) * dt;

    // Apply Anti-Windup in Z-direction.
    bool stop_integral_Z =
        (raw_thrust_sp.z() > max_thrust_ && error_vel.z() >= Scalar(0)) ||
        (raw_thrust_sp.z() < min_thrust_ && error_vel.z() <= Scalar(0));
    if (!stop_integral_Z) {
      iterm_.z() += ki_vel_.z() * error_vel.z() * dt;

      // limit thrust integral
      iterm_.z() = copysign(min(abs(iterm_.z()), max_thrust_), iterm_.z());
    }
  }

  void computeAttitudeTarget(const Vector3& thrust_sp, Scalar yaw_sp) {
    using std::clamp;
    const Scalar raw_thrust = thrust_sp.norm();
    if (raw_thrust < Eigen::NumTraits<Scalar>::epsilon()) {
      throttle_sp_ = 0.0;
      orientation_sp_.setIdentity();
    } else {
      throttle_sp_ = clamp(motor_mdl_(raw_thrust), Scalar(0), Scalar(1));
      orientation_sp_ = vectorToOrientation(thrust_sp / raw_thrust, yaw_sp);
    }
  }

  static Eigen::Quaterniond vectorToOrientation(const Eigen::Vector3d& vector,
                                                Scalar yaw) {
    using std::abs;
    using std::cos;
    using std::sin;

    Matrix3 R;
    R.col(2) = vector;

    // Check magnitude and sign of attitude_mat[2, 2] for
    // in case commanded thrust is fully lateral or upside down
    const Scalar R_22 = R.coeff(2, 2);
    if (abs(R_22) > Eigen::NumTraits<Scalar>::epsilon()) {
      // vector of desired yaw direction
      const Eigen::Vector3d heading_vector(-sin(yaw), cos(yaw), 0.0);
      // desired body_x axis, orthogonal to body_z
      R.col(0) =
          (R_22 < 0.0 ? -heading_vector : heading_vector).cross(R.col(2));
      R.col(0).normalize();

    } else {
      // Desired thrust is fully lateral, set x-axis down
      // yaw setpoint is discarded
      R.col(0) = Eigen::Vector3d::UnitZ();
    }

    // body-frame y-axis is simply tne x-axis cross z-axis, normalized
    R.col(1) = R.col(2).cross(R.col(0));
    R.col(1).normalize();

    // ROS uses quaternion internally, so convert the matrix to quaternion
    Quaternion res(R);
    return res;
  }

  // Controller parameters
  Vector3 kp_pos_;
  Vector3 kd_pos_;
  Vector3 kp_vel_;
  Vector3 ki_vel_;
  Vector3 kd_vel_;
  Scalar filt_coeff_;
  ActuatorModel motor_mdl_;

  // Bounds
  Scalar min_thrust_;
  Scalar max_thrust_;
  Scalar max_thrust_sq_;
  Scalar max_tilt_angle_;
  Scalar max_tilt_ratio_;
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

  // outputs
  Quaternion orientation_sp_{Quaternion::Identity()};
  Scalar throttle_sp_;

  // Memory terms
  Vector3 iterm_{Vector3::Zero()};
  Vector3 error_vel_dot_last_{Vector3::Zero()};
  Vector3 error_vel_last_{Vector3::Zero()};
};

}  // namespace control
#endif  // UAV_PID_CONTROL_PID_CONTROLLER_H_
