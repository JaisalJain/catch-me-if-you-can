#!/bin/bash
# M5 evaluation batch. Fresh sim per run.
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
run() {  # name, args...
  name=$1; shift
  tools/sim_ctl.sh start catch_bringup bringup.launch.py headless:=true rviz:=false
  tools/sim_ctl.sh wait 6 120 || exit 1
  sleep 5
  ros2 run catch_tasks catch_eval "$@" --rec-dir log/m5/$name > log/m5_$name.log 2>&1
  grep -E "RTF" log/sim_latest.log | awk '{print $NF}' | sort | uniq -c > log/m5_${name}_rtf.txt
  cp log/sim_latest.log log/m5_${name}_sim.log
  tools/sim_ctl.sh stop
}
case "${1:-main}" in
  main)
    run force --throws 30 --out docs/m5/catch_30.json
    run time --throws 30 --trigger time --no-place --out docs/m5/catch_30_time_trigger.json
    run nocorr --throws 15 --no-correction --out docs/m5/place_15_no_correction.json;;
  place)   # everything that involves B's place (after changing the place code)
    run force --throws 30 --out docs/m5/catch_30.json
    run nocorr --throws 15 --no-correction --out docs/m5/place_15_no_correction.json;;
  cup)   # off-centre catches (experiment knob) -> cube rotated in the hand: place with vs without correction
    run cup_p15 --throws 6 --cup-offset 0.015 --out docs/m5/cup_p15_corrected.json
    run cup_p15_nc --throws 6 --cup-offset 0.015 --no-correction --out docs/m5/cup_p15_uncorrected.json
    run cup_m10 --throws 6 --cup-offset -0.010 --out docs/m5/cup_m10_corrected.json
    run cup_m10_nc --throws 6 --cup-offset -0.010 --no-correction --out docs/m5/cup_m10_uncorrected.json;;
esac
echo ALL DONE
