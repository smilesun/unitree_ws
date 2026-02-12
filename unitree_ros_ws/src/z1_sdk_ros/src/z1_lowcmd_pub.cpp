#include <ros/ros.h>
#include <array>
#include "unitree_z1_msgs/LowCmd.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "z1_lowcmd_pub");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string topic;
  pnh.param<std::string>("topic", topic, std::string("/z1/lowcmd"));
  double rate_hz;
  pnh.param<double>("rate_hz", rate_hz, 200.0);

  std::array<double, 7> q{{0}};
  std::array<double, 7> qd{{0}};
  std::array<double, 7> tau{{0}};
  std::array<double, 7> kp{{0}};
  std::array<double, 7> kd{{0}};

  // Default gains (can be overridden by params)
  std::vector<double> kp_param, kd_param;
  if (pnh.getParam("kp", kp_param) && kp_param.size() == 7) {
    for (int i = 0; i < 7; ++i) kp[i] = kp_param[i];
  }
  if (pnh.getParam("kd", kd_param) && kd_param.size() == 7) {
    for (int i = 0; i < 7; ++i) kd[i] = kd_param[i];
  }

  ros::Publisher pub = nh.advertise<unitree_z1_msgs::LowCmd>(topic, 1);
  ros::Rate rate(rate_hz);

  while (ros::ok()) {
    unitree_z1_msgs::LowCmd msg;
    for (int i = 0; i < 7; ++i) {
      msg.q[i] = q[i];
      msg.qd[i] = qd[i];
      msg.tau[i] = tau[i];
      msg.kp[i] = kp[i];
      msg.kd[i] = kd[i];
    }
    pub.publish(msg);
    rate.sleep();
  }
  return 0;
}
