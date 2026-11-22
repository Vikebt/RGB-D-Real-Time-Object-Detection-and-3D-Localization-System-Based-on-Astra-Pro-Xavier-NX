# Synchronization / point cloud statistics for one RGB-D pair.
Header header

# color_stamp - depth_stamp in milliseconds (approximate-time pairing).
float64 time_diff_ms

# Exponential moving average rates derived from image header stamps.
float64 color_rate_hz
float64 depth_rate_hz

# Fraction of sampled depth pixels inside [min_range_m, max_range_m].
float64 valid_depth_ratio

# Number of points in the most recently generated XYZRGB cloud.
uint32 point_count

# Wall time spent building the most recent clouds, milliseconds.
float64 process_ms

# Number of synchronized callbacks separated by more than gap_threshold_ms.
uint64 gap_count

# True when the node had to use fallback intrinsics because CameraInfo K is invalid.
bool using_fallback_intrinsics