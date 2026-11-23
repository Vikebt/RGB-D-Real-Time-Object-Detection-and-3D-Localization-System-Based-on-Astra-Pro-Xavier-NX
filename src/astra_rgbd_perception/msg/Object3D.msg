# One detected object with a 2D bounding box and a 3D position.
# 中文：单个目标：2D 框 + 三维位置 + 深度有效性。
Header header

string class_name
uint32 class_id
float32 confidence

# 2D bounding box in the color image, inclusive pixel coordinates.
# 中文：彩色图像中的 2D 框（闭区间像素坐标）。
uint32 xmin
uint32 ymin
uint32 xmax
uint32 ymax

# Depth statistics for the ROI used by the localizer.
# 中文：ROI 深度统计。depth_m 为有效深度中位数（米）。
float32 depth_m
uint32 valid_depth_pixels
float32 valid_depth_ratio

# 3D position in the camera optical frame (X right, Y down, Z forward).
# 中文：相机光学坐标系下的三维位置（X 右、Y 下、Z 前），单位米。
geometry_msgs/Point position

# False when there were too few valid depth pixels; position must be ignored.
# 中文：有效深度不足时为 false，此时 position 不可用。
bool depth_valid