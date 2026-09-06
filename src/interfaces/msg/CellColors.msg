# Full color snapshot in CellGeometry cell order; an empty array clears the display.
# Keep the original data timestamp. Only matching geometry_id may be rendered.
std_msgs/Header header
string geometry_id
std_msgs/ColorRGBA[] colors
