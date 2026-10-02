# v2 flight tracking (catch_tracker): stereo detection + ballistic least squares (known g) of one cube.
std_msgs/Header header                 # stamp = render time (sim) of the newest camera frame used; frame_id "world"
string color
bool in_flight                         # a ballistic segment is being tracked (false: not acquired / ended)
int32 n_frames                         # frames in the current fit
builtin_interfaces/Time t_first        # stamp of the first frame of the flight segment (~ release + first visible)
geometry_msgs/Point position           # fitted cube centre at header.stamp
geometry_msgs/Vector3 velocity         # fitted velocity at header.stamp
float64[36] covariance                 # of (position, velocity) at header.stamp, row-major [m, m/s]
float64 rms                            # fit residual rms [m]
geometry_msgs/Point measured           # this frame's triangulated, bias-corrected position
builtin_interfaces/Time t_received     # node clock when the second image of the pair arrived
builtin_interfaces/Time t_published    # node clock at publication: latency = t_published - header.stamp
geometry_msgs/Point intercept          # predicted position closest to the reference point (B's ready TCP) on the
builtin_interfaces/Time intercept_time #   descending branch, and when
float64[9] intercept_covariance
