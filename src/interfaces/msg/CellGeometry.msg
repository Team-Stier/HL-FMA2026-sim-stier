# Fixed polygon vertices shared by cell color displays. Cell ID is its array index.
# offsets has cell_count + 1 entries, starts at zero, and ends at points length.
# Publish once per loaded geometry with reliable, transient-local QoS.
std_msgs/Header header
string geometry_id
uint32[] offsets
geometry_msgs/Point[] points
