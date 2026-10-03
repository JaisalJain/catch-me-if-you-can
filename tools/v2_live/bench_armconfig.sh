#!/bin/bash
# Arm-configuration review (docs/v2/arm_config/): B's Phase 1 ready pose with a free IK branch (= the v2.2 default)
# vs the home-style elbow-up ready poses (approach 60 / 55 deg) with the IK branch fixed; camera-only pinch, medium noise, v2.2 defaults,
# 60 throws, each cell ALONE (one sim at a time, like bench_seq.sh). Resumable. All trials are recorded
# (log/p2/bench/<label>/) for the IK-branch analysis (tools/v2_live/ik_branch_analysis.py).
#   setsid nohup tools/v2_live/bench_armconfig.sh > log/arm_config/bench.log 2>&1 &
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
export OUT_DIR=docs/v2/arm_config/bench
N=${N:-60}
run() {   # label ready_pose ik_branch
  local f="$REPO/$OUT_DIR/$1.json"
  if [ -f "$f" ] && python3 -c "import json,sys; sys.exit(0 if len(json.load(open('$f'))['trials']) >= $N else 1)"; then
    echo "skip $1"; return; fi
  CATCH_READY_POSE=$2 CATCH_IK_BRANCH=$3 "$REPO/tools/v2_live/bench_cell.sh" 41 "$1" "$N" pinch vision medium 150 realistic default
}
run armcfg_phase1_free   phase1        free
run armcfg_home_fixed    home_style    fixed
run armcfg_home55_fixed  home_style_55 fixed
echo BENCH_ARMCONFIG_DONE
