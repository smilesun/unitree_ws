#!/usr/bin/env python3
import threading

import rospy
from sensor_msgs.msg import JointState
from unitree_z1_msgs.msg import LowCmd


class DualArmMPCNode:
    def __init__(self):
        self._lock = threading.Lock()
        self._panda = None
        self._z1 = None

        self.panda_state_topic = rospy.get_param("~panda_state_topic", "/panda/joint_states")
        self.z1_state_topic = rospy.get_param("~z1_state_topic", "/z1/joint_states")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/z1/lowcmd")
        self.rate_hz = float(rospy.get_param("~rate_hz", 100.0))
        self.kp = float(rospy.get_param("~kp", 20.0))
        self.kd = float(rospy.get_param("~kd", 1.5))
        self.max_step = float(rospy.get_param("~max_step", 0.02))

        self.pub = rospy.Publisher(self.cmd_topic, LowCmd, queue_size=1)
        rospy.Subscriber(self.panda_state_topic, JointState, self._on_panda, queue_size=10)
        rospy.Subscriber(self.z1_state_topic, JointState, self._on_z1, queue_size=10)

    def _on_panda(self, msg):
        with self._lock:
            self._panda = msg

    def _on_z1(self, msg):
        with self._lock:
            self._z1 = msg

    def _compute_target_q(self, panda, z1):
        target = list(z1.position[:6])
        panda_q = list(panda.position[:6])
        z1_q = list(z1.position[:6])
        for i in range(6):
            err = panda_q[i] - z1_q[i]
            step = max(-self.max_step, min(self.max_step, 0.2 * err))
            target[i] = z1_q[i] + step
        return target

    def _build_lowcmd(self, z1_msg, target_q6):
        cmd = LowCmd()
        for i in range(7):
            q = z1_msg.position[i] if i < len(z1_msg.position) else 0.0
            qd = z1_msg.velocity[i] if i < len(z1_msg.velocity) else 0.0
            cmd.q[i] = q
            cmd.qd[i] = 0.0
            cmd.tau[i] = 0.0
            cmd.kp[i] = self.kp
            cmd.kd[i] = self.kd

            if i < 6:
                cmd.q[i] = target_q6[i]
            else:
                cmd.q[i] = q
                cmd.qd[i] = qd
        return cmd

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        warned = False
        while not rospy.is_shutdown():
            with self._lock:
                panda = self._panda
                z1 = self._z1

            if panda is None or z1 is None:
                if not warned:
                    rospy.logwarn(
                        "Waiting for states on %s and %s",
                        self.panda_state_topic,
                        self.z1_state_topic,
                    )
                    warned = True
                rate.sleep()
                continue

            if len(panda.position) < 6 or len(z1.position) < 7:
                rospy.logwarn_throttle(
                    1.0,
                    "Insufficient joint lengths: panda=%d z1=%d",
                    len(panda.position),
                    len(z1.position),
                )
                rate.sleep()
                continue

            target_q6 = self._compute_target_q(panda, z1)
            cmd = self._build_lowcmd(z1, target_q6)
            self.pub.publish(cmd)
            rate.sleep()


def main():
    rospy.init_node("dual_arm_mpc_node")
    node = DualArmMPCNode()
    node.spin()


if __name__ == "__main__":
    main()
