# Z1 low-level command. Units follow SDK conventions.
# 6 arm joints + 1 gripper (last element for gripper).

float64[7] q    # joint position command (rad), gripper in q[6]
float64[7] qd   # joint velocity command (rad/s), gripper in qd[6]
float64[7] tau  # joint torque feedforward (Nm), gripper in tau[6]
float64[7] kp   # joint position gains
float64[7] kd   # joint velocity gains
