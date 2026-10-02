#!/bin/bash
# One live benchmark cell in its own ROS domain (parallel cells must use different domains).
#   tools/v2_live/bench_cell.sh DOMAIN LABEL THROWS MODE SOURCE NOISE CAMERA_SETTING PROFILE LATENCY [SEED]
#   e.g. tools/v2_live/bench_cell.sh 41 vision_medium 60 pinch vision medium 150 realistic default
# CAMERA_SETTING: plugin rate setting (150 -> 128.7 Hz measured, 70 -> 63.9 Hz). PROFILE: realistic | fast_close |
# fast_idealized (MJCF variant from tools/v2_live/mjcf_variant.py + gripper command model). LATENCY: true gripper
# command latency in seconds or "default" (profile: 10 ms); B always assumes 10 ms.
# Output: docs/v2/phase2/bench/LABEL.json (+ failure recordings in log/p2/bench/LABEL, sim log + RTF samples).
set -u
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
DOM=$1; LABEL=$2; N=$3; MODE=$4; SRC=$5; NOISE=$6; CAM=$7; PROF=$8; LAT=$9; SEED=${10:-1}
cd "$REPO"
source /opt/ros/jazzy/setup.bash; source install/setup.bash; source tools/nv_env.sh
export ROS_DOMAIN_ID=$DOM
export CATCH_GRIPPER_PROFILE=$PROF
[ "$LAT" != "default" ] && export CATCH_GRIPPER_LATENCY=$LAT || unset CATCH_GRIPPER_LATENCY
export CATCH_CAMERA_RATE=$CAM
MJ=""; [ "$PROF" = "fast_close" ] || [ "$PROF" = "fast_idealized" ] && MJ="$REPO/log/mjcf/$PROF/scene.xml"
export CATCH_MJCF=${MJ:-default}
PLAN=false; [ "$SRC" = "plan" ] && PLAN=true
D="$REPO/log/p2/bench"; mkdir -p "$D" docs/v2/phase2/bench
SIMLOG="$D/$LABEL.sim.log"
(fastdds shm clean > /dev/null 2>&1)
setsid ros2 launch catch_bringup bringup.launch.py headless:=true rviz:=false camera_rate:=$CAM plan_catcher:=$PLAN \
    ${MJ:+mujoco_model:=$MJ} > "$SIMLOG" 2>&1 < /dev/null &
LPID=$!
for i in $(seq 1 240); do
  [ "$(grep -c 'Configured and activated' "$SIMLOG" 2>/dev/null)" -ge 6 ] && break; sleep 0.5
done
sleep 8
# RTF samples during the cell
( timeout $((N * 60 + 300)) ros2 topic echo /sim/rtf --field data > "$D/$LABEL.rtf.log" 2>/dev/null & )
ros2 run catch_tasks reactive_eval --throws "$N" --mode "$MODE" --source "$SRC" --noise "$NOISE" --seed "$SEED" \
    --label "$LABEL" --out "docs/v2/phase2/bench/$LABEL.json" --rec-dir "$D/$LABEL" --rec-fail-only > "$D/$LABEL.eval.log" 2>&1
grep SUMMARY "$D/$LABEL.eval.log" | cut -c1-400
kill -INT -- -$LPID 2>/dev/null; sleep 8; kill -9 -- -$LPID 2>/dev/null
echo "CELL_DONE $LABEL"
