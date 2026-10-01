#!/bin/bash
# M6: full runs (one fresh sim each), recorded with rosbag2 (C++) for the ground-truth check and the demo video.
#   tools/run_m6.sh SEED...              e.g. tools/run_m6.sh 1 2 3
#   FAULT="--fault-throw 1" tools/run_m6.sh 4     (recovery-path run with an injected late grip loosen)
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
mkdir -p log/m6 docs/m6
for seed in "$@"; do
  tools/sim_ctl.sh start catch_bringup bringup.launch.py headless:=true rviz:=false
  tools/sim_ctl.sh wait 6 120 || exit 1
  sleep 5
  rm -rf log/m6/bag_seed$seed
  ros2 bag record -s mcap -o log/m6/bag_seed$seed /joint_states /free_joint_state_publisher/free_joint_states \
    /b_palm_ft_broadcaster/wrench > log/m6/bag_seed$seed.log 2>&1 &
  BPID=$!
  sleep 3
  timeout 900 ros2 run catch_tasks run_demo --seed $seed $FAULT --out docs/m6/run_seed$seed.json > log/m6/run_seed$seed.log 2>&1
  echo "seed $seed exit $?"
  sleep 1
  kill -TERM $BPID; wait $BPID       # SIGINT is ignored by background jobs of a non-interactive shell
  grep -E "RTF" log/sim_latest.log | awk '{print $NF}' | sort | uniq -c > log/m6/rtf_seed$seed.txt
  cp log/sim_latest.log log/m6/sim_seed$seed.log
  tools/sim_ctl.sh stop
  ros2 run catch_tasks bag_to_npz log/m6/bag_seed$seed log/m6/run_seed$seed.npz
done
echo ALL DONE
