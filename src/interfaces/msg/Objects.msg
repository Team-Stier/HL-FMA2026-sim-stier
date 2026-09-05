# Same packet receive stamp as EgoPose; frame_id="map".
std_msgs/Header header

# Number of populated entries: 0..30. Use only indices [0, length).
int32 length

# Simulator object ID. Entries at the same index describe the same object.
uint32[30] id

# Object box-center X/Y in map metres, offset applied by Sim Bridge.
# Bridge selects object-local reference -> CENTER offset by simulator object ID.
# center = API reference point + Rz(heading) * offset[id].
# No object pitch/roll in API; yaw-only box approximation.
# No shared fallback. Unknown IDs or changed dimensions reject the whole packet.
float32[30] x

float32[30] y

# Object bottom (min Z) in map metres: ref_z + offset_z - size_z / 2.
# For an RViz CUBE, use pose.position.z = z + size_z / 2.
float32[30] z

# Object orientation heading in radians; not necessarily velocity direction.
float32[30] heading

# Planar speed magnitude in m/s, not a velocity vector.
float32[30] speed

# Full object dimensions in metres: API length/width/height respectively.
# Dimensions are unchanged; consumers must not apply the reference offset again.
float32[30] size_x

float32[30] size_y

float32[30] size_z
