#!/bin/bash
# v2.2 experiment: the reactive catcher with the gripper delay probe (reactive.yaml delay_probe) on the cells where
# the gripper's true latency differs from the assumed 10 ms, plus the default as a regression check. One sim at a
# time, like bench_seq.sh. Output: docs/v2/phase2/bench_v2.2_delayprobe/<label>.json
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
export OUT_DIR=docs/v2/phase2/bench_v2.2_delayprobe
N=${N:-60}
run() {
  local f="$REPO/$OUT_DIR/$1.json"
  if [ -f "$f" ] && python3 -c "import json,sys; sys.exit(0 if len(json.load(open('$f'))['trials']) >= $N else 1)"; then
    echo "skip $1"; return; fi
  "$REPO/tools/v2_live/bench_cell.sh" 41 "$1" "$N" "$2" "$3" "$4" "$5" "$6" "$7"
}
run latency20_medium_dp pinch vision medium 150 realistic 0.020
run latency30_medium_dp pinch vision medium 150 realistic 0.030
run vision_medium_dp    pinch vision medium 150 realistic default
run latency0_medium_dp  pinch vision medium 150 realistic 0.0
echo BENCH_DP_DONE
