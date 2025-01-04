std_msgs/Header header
string robot_name
std_msgs/Float64MultiArray joint_positions      # 关节位置矩阵
std_msgs/Float64MultiArray joint_velocities     # 关节速度矩阵

std_msgs/Float64MultiArray leg_pos_hip_in_body
std_msgs/Float64MultiArray leg_pos_hip_in_world
std_msgs/Float64MultiArray leg_pos_base_in_body
std_msgs/Float64MultiArray leg_pos_base_in_world

std_msgs/Float64MultiArray leg_velo_in_body
std_msgs/Float64MultiArray leg_velo_in_world

std_msgs/Float64MultiArray leg_force_in_body
std_msgs/Float64MultiArray leg_force_in_world


# geometry_msgs/Pose end_effector_pose            # 末端位姿
# bool is_enabled