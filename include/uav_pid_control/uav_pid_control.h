// Copyright (c) 2022 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef UAV_PID_CONTROL_UAV_PID_CONTROL_H_
#define UAV_PID_CONTROL_UAV_PID_CONTROL_H_
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/State.h"
#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/timer.h"
#include "uav_pid_control/PositionTarget.h"
#include "uav_pid_control/pid_controller.h"

class PIDControlRos {
 public:
  PIDControlRos();

  int spin();

 private:
  void loadParams();
  void setupPubSub();
  void stateCb(const mavros_msgs::StateConstPtr& msg);
  void poseCb(const geometry_msgs::PoseStampedConstPtr& msg);
  void twistCb(const geometry_msgs::TwistStampedConstPtr& msg);
  void positionTargetCb(const uav_pid_control::PositionTargetConstPtr& msg);
  void mainLoop(const ros::TimerEvent& event);

  ros::Subscriber state_sub_;
  ros::Subscriber pos_sub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber pos_sp_sub_;
  ros::Publisher att_tgt_pub_;
  ros::Publisher ctl_status_pub_;
  ros::Timer timer_;
  ros::Duration loop_interval_;
  control::PIDController<> ctl_;
};
#endif  // UAV_PID_CONTROL_UAV_PID_CONTROL_H_
