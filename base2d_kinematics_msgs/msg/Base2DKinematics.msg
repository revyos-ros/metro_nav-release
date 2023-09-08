# Velocities
# X & Y in meters / second
float64 min_vel_x
float64 max_vel_x
float64 min_vel_y
float64 max_vel_y

# theta in radians / second
# The minimum rotational velocity is assumed to be -max_vel_theta
# i.e. can turn both ways equally
float64 max_vel_theta

# Acceleration
# X & Y in meters / second^2
float64 acc_lim_x
float64 decel_lim_x
float64 acc_lim_y
float64 decel_lim_y

# Theta in radians / second^2
float64 acc_lim_theta
float64 decel_lim_theta

# Speeds
# Limits for combined xy / translational velocity in meters / second
float64 min_speed_xy    # Previously called min_trans_vel
float64 max_speed_xy    # Previously called max_trans_vel

# The absolute value of the minimum rotational velocity in radians / second
float64 min_speed_theta # Previously called min_rot_vel
