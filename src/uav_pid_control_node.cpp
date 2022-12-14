
#include "ros/init.h"
#include "uav_pid_control/uav_pid_control.h"

int main(int argc, char** argv) {
  using namespace std::string_literals;
  ros::init(argc, argv, "uav_pid_control_node");

  PIDControlRos node;

  node.spin();

  return 0;
}
