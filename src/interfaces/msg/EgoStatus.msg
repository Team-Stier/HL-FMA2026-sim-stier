# Bridge receive stamp inherited by Tracker; frame_id="map".
std_msgs/Header header

# Rear axle reference position in map metres.
float32 x

float32 y

float32 z

# Radians; Rz(heading) * Ry(pitch) * Rx(roll).
float32 heading

float32 pitch

float32 roll

# Planar m/s: sqrt((x-previous_x)^2 + (y-previous_y)^2) / dt.
# dt is the difference between consecutive source Header stamps in seconds.
# First sample, nonpositive dt or speed above configured threshold: output 0.
float32 speed
