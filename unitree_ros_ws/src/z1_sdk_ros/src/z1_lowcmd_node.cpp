#include <ros/ros.h>
#include <array>
#include <mutex>
#include <vector>

#include <sensor_msgs/JointState.h>
#include "unitree_z1_msgs/LowCmd.h"
#include "unitree_arm_sdk/control/unitreeArm.h"

using UNITREE_ARM::unitreeArm;
using UNITREE_ARM::Vec6;
using UNITREE_ARM::ArmFSMState;

namespace {
Vec6 toVec6(const double *data) {
  Vec6 v;
  for (int i = 0; i < 6; ++i) {
    v[i] = data[i];
  }
  return v;
}
}

class Z1LowcmdNode {
 public:
  Z1LowcmdNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : has_gripper_(getHasGripper(pnh)),
        arm_(has_gripper_) {
    std::string topic;
    pnh.param<std::string>("topic", topic, std::string("/z1/lowcmd"));
    pnh.param<double>("cmd_timeout", cmd_timeout_, 0.2);
    pnh.param<bool>("set_lowcmd_fsm", set_lowcmd_fsm_, true);
    pnh.param<double>("rate_hz", rate_hz_, 0.0);
    pnh.param<std::string>("state_topic", state_topic_, std::string("/z1/joint_states"));

    sub_ = nh.subscribe(topic, 10, &Z1LowcmdNode::onCmd, this);
    state_pub_ = nh.advertise<sensor_msgs::JointState>(state_topic_, 10);

    std::vector<std::string> joint_names;
    if (pnh.getParam("joint_names", joint_names) && joint_names.size() == 7) {
      joint_names_ = joint_names;
    } else {
      joint_names_ = {
          "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};
    }

    arm_.sendRecvThread->start();
    ros::Duration(0.2).sleep();

    if (set_lowcmd_fsm_) {
      arm_.setFsm(ArmFSMState::PASSIVE);
      arm_.setFsm(ArmFSMState::LOWCMD);
    }

    // Initialize gains from SDK defaults.
    kp_ = arm_._ctrlComp->lowcmd->kp;
    kd_ = arm_._ctrlComp->lowcmd->kd;

    // Stop background thread; we drive send/recv in the loop.
    arm_.sendRecvThread->shutdown();

    // Initialize commands to current state to avoid jumps.
    arm_.sendRecv();
    Vec6 q0 = arm_.lowstate->getQ();
    for (int i = 0; i < 6; ++i) {
      last_q_[i] = q0[i];
    }
    last_gripper_q_ = arm_.lowstate->getGripperQ();
    last_cmd_time_ = ros::Time::now();
  }

  void spin() {
    double dt = arm_._ctrlComp->dt;
    if (rate_hz_ <= 0.0) {
      if (dt <= 0.0) {
        dt = 0.002;
      }
      rate_hz_ = 1.0 / dt;
    }
    ros::Rate rate(rate_hz_);

    while (ros::ok()) {
      ros::spinOnce();
      sendCommand();
      publishState();
      rate.sleep();
    }
  }

 private:
  static bool getHasGripper(ros::NodeHandle& pnh) {
    bool has_gripper = true;
    pnh.param<bool>("has_gripper", has_gripper, true);
    return has_gripper;
  }

  void onCmd(const unitree_z1_msgs::LowCmd::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
    has_cmd_ = true;
  }

  void sendCommand() {
    unitree_z1_msgs::LowCmd cmd;
    bool has_cmd = false;
    ros::Time last_time;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = last_cmd_;
      has_cmd = has_cmd_;
      last_time = last_cmd_time_;
    }

    if (has_cmd && cmd_timeout_ > 0.0 && (ros::Time::now() - last_time).toSec() > cmd_timeout_) {
      has_cmd = false;
    }

    if (has_cmd) {
      for (int i = 0; i < 7; ++i) {
        last_q_[i] = cmd.q[i];
        last_qd_[i] = cmd.qd[i];
        last_tau_[i] = cmd.tau[i];
        kp_[i] = cmd.kp[i];
        kd_[i] = cmd.kd[i];
      }
    }

    // Apply gains (first 6 joints). Gripper gain uses last element if enabled.
    std::vector<double> kp6(6), kd6(6);
    for (int i = 0; i < 6; ++i) {
      kp6[i] = kp_[i];
      kd6[i] = kd_[i];
    }
    arm_._ctrlComp->lowcmd->setControlGain(kp6, kd6);

    if (has_gripper_) {
      arm_._ctrlComp->lowcmd->setGripperGain(kp_[6], kd_[6]);
    }

    Vec6 q = toVec6(last_q_.data());
    Vec6 qd = toVec6(last_qd_.data());
    Vec6 tau = toVec6(last_tau_.data());

    arm_.setArmCmd(q, qd, tau);
    if (has_gripper_) {
      arm_.setGripperCmd(last_q_[6], last_qd_[6], last_tau_[6]);
    }

    arm_.sendRecv();
  }

  void publishState() {
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.name = joint_names_;
    js.position.resize(7);
    js.velocity.resize(7);
    js.effort.resize(7);

    Vec6 q = arm_.lowstate->getQ();
    Vec6 qd = arm_.lowstate->getQd();
    Vec6 tau = arm_.lowstate->getTau();

    for (int i = 0; i < 6; ++i) {
      js.position[i] = q[i];
      js.velocity[i] = qd[i];
      js.effort[i] = tau[i];
    }

    js.position[6] = arm_.lowstate->getGripperQ();
    js.velocity[6] = arm_.lowstate->getGripperQd();
    js.effort[6] = arm_.lowstate->getGripperTau();

    state_pub_.publish(js);
  }

  unitreeArm arm_;
  bool has_gripper_{true};
  bool set_lowcmd_fsm_{true};
  double cmd_timeout_{0.2};
  double rate_hz_{0.0};

  ros::Subscriber sub_;
  ros::Publisher state_pub_;
  std::mutex mutex_;
  unitree_z1_msgs::LowCmd last_cmd_;
  bool has_cmd_{false};
  ros::Time last_cmd_time_;

  std::array<double, 7> last_q_{{0}};
  std::array<double, 7> last_qd_{{0}};
  std::array<double, 7> last_tau_{{0}};
  std::array<double, 7> kp_{{0}};
  std::array<double, 7> kd_{{0}};

  std::string state_topic_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "z1_lowcmd_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Z1LowcmdNode node(nh, pnh);
  node.spin();
  return 0;
}
