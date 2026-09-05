# Command creation stamp; frame_id="base_link".
std_msgs/Header header

# Target road-wheel steering angle in radians.
float32 steering

# Target longitudinal acceleration in m/s^2; negative requests braking.
float32 target_accel

# Simulator API turnSignal code, forwarded without remapping.
uint8 turn_signal
