# Inherits the input Header stamp; frame_id="map".
std_msgs/Header header

# Indexed by offline cell ID. Array length is the number of cells.
# m/s; minimum of lanelet static limit and dynamic restrictions.
float32[] speed_cap_mps

# Cell-major index: cell_id * 13 + bin.
# Bin 0=current; bins 1..12=((bin-1)*0.5, bin*0.5] seconds from header.stamp.
# Any intersection within an interval occupies that interval.
# Values: 0=free, 1=occupied, -1=unknown, otherwise probability in [0,1].
float32[] occupancy_probability
