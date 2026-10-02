#!/bin/bash
# v2 step 6 live benchmark: three ROS domains in parallel, five cells each (60 throws per cell).
#   tools/v2_live/bench.sh            (detached: setsid nohup tools/v2_live/bench.sh > log/p2/bench.log 2>&1 &)
# Cells skip themselves if docs/v2/phase2/bench/LABEL.json already holds THROWS trials (resume).
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
N=${N:-60}
run() {   # domain label mode source noise cam profile latency
  local f="$REPO/docs/v2/phase2/bench/$2.json"
  if [ -f "$f" ] && python3 -c "import json,sys; sys.exit(0 if len(json.load(open('$f'))['trials']) >= $N else 1)"; then
    echo "skip $2"; return; fi
  "$REPO/tools/v2_live/bench_cell.sh" "$1" "$2" "$N" "$3" "$4" "$5" "$6" "$7" "$8"
}
(
  run 41 vision_none    pinch vision none   150 realistic default
  run 41 vision_low     pinch vision low    150 realistic default
  run 41 vision_medium  pinch vision medium 150 realistic default
  run 41 vision_high    pinch vision high   150 realistic default
  run 41 cup_medium     cup   vision medium 150 realistic default
) &
(
  run 42 plan_none      pinch plan   none   150 realistic default
  run 42 plan_low       pinch plan   low    150 realistic default
  run 42 plan_medium    pinch plan   medium 150 realistic default
  run 42 plan_high      pinch plan   high   150 realistic default
  run 42 cam60_medium   pinch vision medium 70  realistic default
) &
(
  run 43 latency0_medium      pinch vision medium 150 realistic 0.0
  run 43 latency20_medium     pinch vision medium 150 realistic 0.020
  run 43 latency30_medium     pinch vision medium 150 realistic 0.030
  run 43 fastclose_medium     pinch vision medium 150 fast_close default
  run 43 fastidealized_medium pinch vision medium 150 fast_idealized default
) &
wait
echo BENCH_DONE
