# Same packet receive stamp as EgoPose; frame_id="".
std_msgs/Header header

# Selected controller ID from the simulator API, not a physical lamp ID.
int32 id

# 0=unknown, 1=red, 2=yellow, 3=green, 4=left, 5=green+left, 6=flashing.
uint8 state
