#!/bin/bash
# Background sim control for scripted tests.
#   tools/sim_ctl.sh start [launch-pkg launch-file [args...]]   (default: catch_sim sim.launch.py headless:=true)
#   tools/sim_ctl.sh wait  [n_controllers=5] [timeout_s=90]     (blocks until controllers are active)
#   tools/sim_ctl.sh stop
# Logs go to log/sim_latest.log
REPO="$(cd "$(dirname "$0")/.." && pwd)"
LOG="$REPO/log/sim_latest.log"
cmd=$1; shift
kill_sim() {
  pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch catch_" 2>/dev/null
  for i in $(seq 1 20); do pgrep -f "mujoco_ros2_control/ros2_control_node" >/dev/null || break; sleep 0.5; done
  pkill -9 -f "mujoco_ros2_control/ros2_control_node" 2>/dev/null
  pkill -9 -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch catch_" 2>/dev/null
  pkill -f "$REPO/install/catch_" 2>/dev/null
  pkill -f "moveit_ros_move_group/move_group" 2>/dev/null
  pkill -f "robot_state_publisher/robot_state_publisher" 2>/dev/null   # outlived the launch (v2.3)
  pkill -f "rviz2/rviz2" 2>/dev/null
  true
}
case "$cmd" in
  start)
    kill_sim
    # FastDDS shared-memory segments of killed processes make discovery fail for new ones
    # ("open_and_lock_file failed", action servers never seen); remove the ones no live process holds
    (source /opt/ros/jazzy/setup.bash; fastdds shm clean > /dev/null 2>&1)
    mkdir -p "$REPO/log"
    source "$REPO/tools/nv_env.sh"; source /opt/ros/jazzy/setup.bash; source "$REPO/install/setup.bash"
    if [ $# -eq 0 ]; then set -- catch_sim sim.launch.py headless:=true; fi
    setsid nohup ros2 launch "$@" > "$LOG" 2>&1 < /dev/null &
    echo "started: ros2 launch $* (log: $LOG)";;
  wait)
    n=${1:-5}; t=${2:-90}
    for i in $(seq 1 $((t*2))); do
      c=$(grep -c "Configured and activated" "$LOG" 2>/dev/null)
      [ "${c:-0}" -ge "$n" ] && { echo "ready ($c controllers active after $((i/2)) s)"; exit 0; }
      grep -q "process has died.*ros2_control_node" "$LOG" 2>/dev/null && { echo "sim died"; tail -20 "$LOG"; exit 1; }
      sleep 0.5
    done
    echo "timeout waiting for controllers"; tail -30 "$LOG"; exit 1;;
  stop) kill_sim; echo stopped;;
  *) echo "usage: $0 start|wait|stop"; exit 2;;
esac
