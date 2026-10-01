# Published by robot A's throw node when a throw is committed (before execution starts).
# All quantities in the world frame; times are sim time.
std_msgs/Header header
string cube_id                       # e.g. "cube_red" (cross-check only: B must determine the color itself)
geometry_msgs/Point release_position  # cube centre at the predicted separation instant
geometry_msgs/Vector3 release_velocity
geometry_msgs/Vector3 release_spin    # cube angular velocity at release (rad/s)
builtin_interfaces/Time release_time  # predicted separation time
builtin_interfaces/Time stroke_start_time
float64 flight_time                   # nominal release -> catch point
geometry_msgs/Point catch_point       # nominal intercept used for the geometry design (B may choose another)
