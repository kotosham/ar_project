# Low-rate map->odom correction from edge SLAM (NOT a TF stream).
# Applied / rebroadcast locally on the Pi by map_odom_relay.
std_msgs/Header header                       # stamp = time the correction refers to
geometry_msgs/TransformStamped map_to_odom   # the map->odom correction itself
float64[36] covariance                       # correction covariance (uncertainty gating)
float64 fitness                              # SLAM fit quality (0..1)
uint32 seq                                   # monotonic counter for stale/reorder rejection
bool relocalized                             # true on a jump (relocalization) -> jump gating
