#!/bin/bash
# v2 full runs (one fresh sim each, default ROS domain), recorded for the ground-truth check and the demo video.
#   tools/v2_live/run_demo_v2.sh SEED...      e.g. tools/v2_live/run_demo_v2.sh 1 2 3   (NOISE=medium default)
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
mkdir -p log/v2demo docs/v2/phase2/demo
NOISE=${NOISE:-medium}
for seed in "$@"; do
  tools/sim_ctl.sh start catch_bringup bringup.launch.py headless:=true rviz:=false
  tools/sim_ctl.sh wait 6 120 || exit 1
  sleep 6
  rm -rf log/v2demo/bag_seed$seed
  ros2 bag record -s mcap -o log/v2demo/bag_seed$seed /joint_states /free_joint_state_publisher/free_joint_states \
    /b_palm_ft_broadcaster/wrench > log/v2demo/bag_seed$seed.log 2>&1 &
  RPID=$!
  sleep 3
  timeout 1500 ros2 run catch_tasks run_demo --seed $seed --noise $NOISE --out docs/v2/phase2/demo/run_seed$seed.json \
      > log/v2demo/run_seed$seed.log 2>&1
  echo "seed $seed exit $?"
  sleep 1
  kill -TERM $RPID; wait $RPID       # SIGINT is ignored by background jobs of a non-interactive shell
  grep -E "RTF" log/sim_latest.log | awk '{print $NF}' | sort | uniq -c > log/v2demo/rtf_seed$seed.txt
  cp log/sim_latest.log log/v2demo/sim_seed$seed.log
  tools/sim_ctl.sh stop
  ros2 run catch_tasks bag_to_npz log/v2demo/bag_seed$seed log/v2demo/run_seed$seed.npz
done
echo ALL DONE
