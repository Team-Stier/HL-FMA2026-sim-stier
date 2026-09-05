# Same packet receive stamp as EgoPose; frame_id="map".
std_msgs/Header header

# Number of populated entries: 0..30. Use only indices [0, length).
int32 length

# Simulator object ID. Entries at the same index describe the same object.
uint32[30] id

# Map metres after Bridge conversion: XY box center and world box minimum Z.
float32[30] x

float32[30] y

float32[30] z

# Object orientation heading in radians; not necessarily velocity direction.
float32[30] heading

# Planar speed magnitude in m/s, not a velocity vector.
float32[30] speed

# Full object dimensions in metres: API length/width/height respectively.
# The Bridge applies the reference offset; consumers must not apply it again.
float32[30] size_x

float32[30] size_y

float32[30] size_z
