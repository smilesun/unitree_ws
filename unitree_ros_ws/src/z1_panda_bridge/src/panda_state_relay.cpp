#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class PandaStateRelay {
 public:
  PandaStateRelay(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string in_topic;
    std::string out_topic;

    pnh.param<std::string>("in_topic", in_topic, "/panda/joint_states");
    pnh.param<std::string>("out_topic", out_topic, "/z1/panda_joint_states");
    pnh.param<bool>("latch", latch_, false);

    pub_ = nh.advertise<sensor_msgs::JointState>(out_topic, 10, latch_);
    sub_ = nh.subscribe(in_topic, 10, &PandaStateRelay::onJointState, this);

    ROS_INFO_STREAM("Relay listening on " << in_topic << " -> publishing on " << out_topic);
  }

 private:
  void onJointState(const sensor_msgs::JointState::ConstPtr& msg) {
    pub_.publish(*msg);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  bool latch_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "panda_state_relay");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PandaStateRelay relay(nh, pnh);
  ros::spin();
  return 0;
}
