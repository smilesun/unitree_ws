#include <ros/ros.h>
#include <array>
#include <vector>

#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "unitree_z1_msgs/LowCmd.h"
#include "unitree_arm_sdk/model/ArmModel.h"

using UNITREE_ARM::Vec6;
using UNITREE_ARM::HomoMat;
using UNITREE_ARM::Z1Model;

namespace {
Vec6 toVec6(const std::array<double, 6>& a) {
  Vec6 v;
  for (int i = 0; i < 6; ++i) {
    v[i] = a[i];
  }
  return v;
}

std::array<double, 6> fromVec6(const Vec6& v) {
  std::array<double, 6> a;
  for (int i = 0; i < 6; ++i) {
    a[i] = v[i];
  }
  return a;
}

Vec6 clampVec6(const Vec6& v, double low, double high) {
  Vec6 out;
  for (int i = 0; i < 6; ++i) {
    double x = v[i];
    if (x < low) x = low;
    if (x > high) x = high;
    out[i] = x;
  }
  return out;
}
}

class Z1PandaXYController {
 public:
  Z1PandaXYController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : tf_listener_(tf_buffer_) {
    pnh.param<std::string>("world_frame", world_frame_, std::string("world"));
    pnh.param<std::string>("panda_ee_frame", panda_ee_frame_, std::string("panda_link8"));
    pnh.param<std::string>("z1_base_frame", z1_base_frame_, std::string("z1_base"));
    pnh.param<double>("z_offset", z_offset_, 0.0);
    pnh.param<double>("rate_hz", rate_hz_, 200.0);
    pnh.param<double>("tau_limit", tau_limit_, 30.0);
    pnh.param<bool>("enable_gripper", enable_gripper_, true);
    pnh.param<std::string>("state_topic", state_topic_, std::string("/z1/joint_states"));
    pnh.param<bool>("use_state", use_state_, true);

    std::vector<double> kp_param, kd_param;
    if (pnh.getParam("kp", kp_param) && kp_param.size() == 7) {
      for (int i = 0; i < 7; ++i) kp_[i] = kp_param[i];
    } else {
      kp_ = {{20, 30, 30, 20, 15, 10, 20}};
    }

    if (pnh.getParam("kd", kd_param) && kd_param.size() == 7) {
      for (int i = 0; i < 7; ++i) kd_[i] = kd_param[i];
    } else {
      kd_ = {{2000, 2000, 2000, 2000, 2000, 2000, 2000}};
    }

    std::vector<double> q_init;
    if (pnh.getParam("q_init", q_init) && q_init.size() == 6) {
      for (int i = 0; i < 6; ++i) last_q_[i] = q_init[i];
    }

    pub_ = nh.advertise<unitree_z1_msgs::LowCmd>("/z1/lowcmd", 1);
    state_sub_ = nh.subscribe(state_topic_, 10, &Z1PandaXYController::onState, this);
  }

  void spin() {
    ros::Rate rate(rate_hz_);
    while (ros::ok()) {
      step();
      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  void step() {
    geometry_msgs::TransformStamped tf_panda;
    geometry_msgs::TransformStamped tf_z1;
    try {
      tf_panda = tf_buffer_.lookupTransform(world_frame_, panda_ee_frame_, ros::Time(0));
      tf_z1 = tf_buffer_.lookupTransform(world_frame_, z1_base_frame_, ros::Time(0));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
      return;
    }

    tf2::Transform T_world_z1;
    tf2::fromMsg(tf_z1.transform, T_world_z1);

    tf2::Vector3 p_panda_world(tf_panda.transform.translation.x,
                               tf_panda.transform.translation.y,
                               tf_panda.transform.translation.z);

    tf2::Vector3 p_z1_world = T_world_z1.getOrigin();

    // Track panda XY in world, keep Z offset relative to Z1 base.
    tf2::Vector3 p_des_world(p_panda_world.x(), p_panda_world.y(),
                             p_z1_world.z() + z_offset_);

    // Transform desired position into Z1 base frame.
    tf2::Vector3 p_des_base = T_world_z1.inverse() * p_des_world;

    // Build desired pose: keep current orientation, change position.
    Z1Model model;
    Vec6 q_past = toVec6(getSeedQ());
    HomoMat T_curr = model.forwardKinematics(q_past);
    T_curr(0, 3) = p_des_base.x();
    T_curr(1, 3) = p_des_base.y();
    T_curr(2, 3) = p_des_base.z();

    Vec6 q_result;
    bool ok = model.inverseKinematics(T_curr, q_past, q_result, true);
    if (!ok) {
      ROS_WARN_THROTTLE(1.0, "IK failed for desired XY target");
      return;
    }

    double dt = 1.0 / rate_hz_;
    Vec6 q_cmd = q_result;
    Vec6 qd_cmd = (q_cmd - q_past) / dt;
    Vec6 tau_cmd = model.inverseDynamics(q_cmd, qd_cmd, Vec6::Zero(), Vec6::Zero());
    tau_cmd = clampVec6(tau_cmd, -tau_limit_, tau_limit_);

    last_q_ = fromVec6(q_cmd);

    unitree_z1_msgs::LowCmd msg;
    for (int i = 0; i < 6; ++i) {
      msg.q[i] = q_cmd[i];
      msg.qd[i] = qd_cmd[i];
      msg.tau[i] = tau_cmd[i];
      msg.kp[i] = kp_[i];
      msg.kd[i] = kd_[i];
    }

    // Gripper: hold at zero unless disabled.
    if (enable_gripper_) {
      msg.q[6] = 0.0;
      msg.qd[6] = 0.0;
      msg.tau[6] = 0.0;
      msg.kp[6] = kp_[6];
      msg.kd[6] = kd_[6];
    } else {
      msg.q[6] = 0.0;
      msg.qd[6] = 0.0;
      msg.tau[6] = 0.0;
      msg.kp[6] = 0.0;
      msg.kd[6] = 0.0;
    }

    pub_.publish(msg);
  }

  void onState(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->position.size() < 6) {
      return;
    }
    for (int i = 0; i < 6; ++i) {
      last_state_q_[i] = msg->position[i];
    }
    has_state_ = true;
  }

  std::array<double, 6> getSeedQ() const {
    if (use_state_ && has_state_) {
      return last_state_q_;
    }
    return last_q_;
  }

  ros::Publisher pub_;
  ros::Subscriber state_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string world_frame_;
  std::string panda_ee_frame_;
  std::string z1_base_frame_;
  double z_offset_{0.0};
  double rate_hz_{200.0};
  double tau_limit_{30.0};
  bool enable_gripper_{true};
  bool use_state_{true};

  std::array<double, 6> last_q_{{0}};
  std::array<double, 6> last_state_q_{{0}};
  bool has_state_{false};
  std::array<double, 7> kp_{{0}};
  std::array<double, 7> kd_{{0}};

  std::string state_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "z1_panda_xy_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Z1PandaXYController node(nh, pnh);
  node.spin();
  return 0;
}
