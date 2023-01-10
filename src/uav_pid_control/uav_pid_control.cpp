// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "uav_pid_control/uav_pid_control.h"

#include <string>

#include "mavros_msgs/AttitudeTarget.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "tf2_eigen/tf2_eigen.h"
#include "uav_pid_control/PIDControlStatus.h"
#include "uav_pid_control/PositionTarget.h"
#include "uav_pid_control/pid_controller.h"

PIDControlRos::PIDControlRos() {
  loadParams();
  setupPubSub();
}

int PIDControlRos::spin() {
  const ros::NodeHandle nh;
  timer_ = nh.createTimer(loop_interval_, &PIDControlRos::mainLoop, this);
  ros::spin();
  return 0;
}

void PIDControlRos::loadParams() {
  const ros::NodeHandle pnh("~");
  control::PIDController<double>::Params p;
  p.kp_pos.head<2>().setConstant(pnh.param("kp_pos/xy", 1.0));
  p.kp_pos.z() = pnh.param("kp_pos/z", 1.0);
  p.kd_pos.head<2>().setConstant(pnh.param("kd_pos/xy", 1.0));
  p.kd_pos.z() = pnh.param("kd_pos/z", 1.0);
  p.kp_vel.head<2>().setConstant(pnh.param("kp_vel/xy", 0.5));
  p.kp_vel.z() = pnh.param("kp_vel/z", 0.5);
  p.ki_vel.head<2>().setConstant(pnh.param("ki_vel/xy", 0.02));
  p.ki_vel.z() = pnh.param("ki_vel/z", 0.02);
  p.kd_vel.head<2>().setConstant(pnh.param("kd_vel/xy", 0.01));
  p.kd_vel.z() = pnh.param("kd_vel/z", 0.01);
  p.filt_coeff = pnh.param("filt_coeff", 5.0);
  p.hover_thrust = pnh.param("thrust/hover", 9.81);
  throttle_intercept_ = pnh.param("thrust/offset", 0.0);
  throttle_slope_ = pnh.param("thrust/scaling", 0.03397);
  p.max_vel_sp.head<2>().setConstant(
      pnh.param("constraints/max_vel_sp/xy", 1.0));
  p.max_vel_sp.z() = pnh.param("constraints/max_vel_sp/z", 2.0);
  p.min_thrust = pnh.param("constraints/min_thrust", 0.0);
  p.max_thrust = pnh.param("constraints/max_thrust", 15.0);
  p.max_tilt_angle = pnh.param("constraints/max_tilt_angle", 5.0);

  loop_interval_ = ros::Duration(pnh.param("controller/looptime", 0.01));
  ctl_.loadParams(p);
}

void PIDControlRos::setupPubSub() {
  using std::string_literals::operator""s;
  ros::NodeHandle nh;
  pos_sub_ = nh.subscribe(
      "/mavros/local_position/pose"s, 1, &PIDControlRos::poseCb, this);
  vel_sub_ = nh.subscribe("/mavros/local_position/velocity_local"s,
                          1,
                          &PIDControlRos::twistCb,
                          this);

  pos_sp_sub_ = nh.subscribe("/uav_pid_control/setpoints/position",
                             1,
                             &PIDControlRos::positionTargetCb,
                             this);
  att_tgt_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude"s, 1);
  ctl_status_pub_ = nh.advertise<uav_pid_control::PIDControlStatus>(
      "/uav_pid_control/controller_internal/status", 1);
}

void PIDControlRos::stateCb(const mavros_msgs::StateConstPtr& msg) {
  using std::string_literals::operator""s;
  ctl_.pause_integration() =
      !static_cast<bool>(msg->armed) || msg->mode != "OFFBOARD"s;
}

void PIDControlRos::poseCb(const geometry_msgs::PoseStampedConstPtr& msg) {
  tf2::fromMsg(msg->pose.position, ctl_.position());
}

void PIDControlRos::twistCb(const geometry_msgs::TwistStampedConstPtr& msg) {
  tf2::fromMsg(msg->twist.linear, ctl_.velocity());
}

void PIDControlRos::positionTargetCb(
    const uav_pid_control::PositionTargetConstPtr& msg) {
  tf2::fromMsg(msg->position, ctl_.position_sp());
  ctl_.yaw_sp() = msg->heading;
}

double PIDControlRos::throttleCurve(double x) const {
  return throttle_intercept_ + throttle_slope_ * x;
}

void PIDControlRos::mainLoop(const ros::TimerEvent& event) {
  const ros::Time now = event.current_real;
  const double dt = (now - event.last_expired).toSec();
  const auto& [orientation_sp, thrust_sp] = ctl_.computeControlOutput(dt);
  mavros_msgs::AttitudeTarget pld;
  pld.header.stamp = now;
  pld.orientation = tf2::toMsg(orientation_sp);
  pld.thrust = std::clamp(throttleCurve(thrust_sp), 0.0, 1.0);
  att_tgt_pub_.publish(pld);

  uav_pid_control::PIDControlStatus status;
  status.header.stamp = now;
  tf2::toMsg(ctl_.position_error(), status.position_error);
  tf2::toMsg(ctl_.velocity_error(), status.velocity_error);
  ctl_status_pub_.publish(status);
}
