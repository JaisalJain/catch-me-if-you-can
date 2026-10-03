#!/bin/bash
# v2 step 6 live benchmark, the clean version (v2.1): every cell ALONE, one sim at a time, sequentially (nothing else
# heavy may run: reactive_eval records the load average and real-time factor per trial). Resumable: a cell is skipped
# if docs/v2/phase2/bench/LABEL.json already holds N trials.
#   setsid nohup tools/v2_live/bench_seq.sh > log/p2/bench_seq.log 2>&1 &
# (The v2.0 numbers — three cells in parallel, plus three re-runs alone — are in docs/v2/phase2/bench_v2.0/.)
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
N=${N:-60}
run() {   # label mode source noise cam profile latency
  local f="$REPO/docs/v2/phase2/bench/$1.json"
  if [ -f "$f" ] && python3 -c "import json,sys; sys.exit(0 if len(json.load(open('$f'))['trials']) >= $N else 1)"; then
    echo "skip $1"; return; fi
  "$REPO/tools/v2_live/bench_cell.sh" 41 "$1" "$N" "$2" "$3" "$4" "$5" "$6" "$7"
}
run vision_medium        pinch vision medium 150 realistic default
run vision_none          pinch vision none   150 realistic default
run vision_low           pinch vision low    150 realistic default
run vision_high          pinch vision high   150 realistic default
run plan_none            pinch plan   none   150 realistic default
run plan_low             pinch plan   low    150 realistic default
run plan_medium          pinch plan   medium 150 realistic default
run plan_high            pinch plan   high   150 realistic default
run cup_medium           cup   vision medium 150 realistic default
run cam60_medium         pinch vision medium 70  realistic default
run latency0_medium      pinch vision medium 150 realistic 0.0
run latency20_medium     pinch vision medium 150 realistic 0.020
run latency30_medium     pinch vision medium 150 realistic 0.030
run fastclose_medium     pinch vision medium 150 fast_close default
run fastidealized_medium pinch vision medium 150 fast_idealized default
echo BENCH_SEQ_DONE
