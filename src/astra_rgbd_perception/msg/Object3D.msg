# One detected object with a 2D bounding box and a 3D position.
Header header

string class_name
uint32 class_id
float32 confidence

# 2D bounding box in the color image, inclusive pixel coordinates.
uint32 xmin
uint32 ymin
uint32 xmax
uint32 ymax

# Depth statistics for the ROI used by the localizer.
float32 depth_m
uint32 valid_depth_pixels
float32 valid_depth_ratio

# 3D position in the camera optical frame (X right, Y down, Z forward).
geometry_msgs/Point position

# False when there were too few valid depth pixels; position must be ignored.
bool depth_valid