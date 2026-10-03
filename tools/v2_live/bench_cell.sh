#!/bin/bash
# One live benchmark cell in its own ROS domain (parallel cells must use different domains).
#   tools/v2_live/bench_cell.sh DOMAIN LABEL THROWS MODE SOURCE NOISE CAMERA_SETTING PROFILE LATENCY [SEED]
#   e.g. tools/v2_live/bench_cell.sh 41 vision_medium 60 pinch vision medium 150 realistic default
# CAMERA_SETTING: plugin rate setting (150 -> 128.7 Hz measured, 70 -> 63.9 Hz). PROFILE: realistic | fast_close |
# fast_idealized (B only: MJCF variant from tools/v2_live/mjcf_variant.py PROFILE --arms b + gripper command model). LATENCY: true gripper
# command latency in seconds or "default" (profile: 10 ms); B always assumes 10 ms.
# Output: OUT_DIR/LABEL.json (default docs/v2/phase2/bench; + failure recordings in log/p2/bench/LABEL, sim log).

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
DOM=$1; LABEL=$2; N=$3; MODE=$4; SRC=$5; NOISE=$6; CAM=$7; PROF=$8; LAT=$9; SEED=${10:-1}
cd "$REPO"
source /opt/ros/jazzy/setup.bash; source install/setup.bash; source tools/nv_env.sh
export ROS_DOMAIN_ID=$DOM
export CATCH_GRIPPER_PROFILE=$PROF
export CATCH_GRIPPER_ARMS=b          # only B's (catching) gripper varies; A throws with the default one
[ "$LAT" != "default" ] && export CATCH_GRIPPER_LATENCY=$LAT || unset CATCH_GRIPPER_LATENCY
export CATCH_CAMERA_RATE=$CAM
MJ=""; [ "$PROF" = "fast_close" ] || [ "$PROF" = "fast_idealized" ] && MJ="$REPO/log/mjcf/${PROF}_b/scene.xml"
export CATCH_MJCF=${MJ:-default}
PLAN=false; [ "$SRC" = "plan" ] && PLAN=true
D="$REPO/log/p2/bench"; OUT_DIR=${OUT_DIR:-docs/v2/phase2/bench}; mkdir -p "$D" "$OUT_DIR"
SIMLOG="$D/$LABEL.sim.log"
(fastdds shm clean > /dev/null 2>&1)
setsid ros2 launch catch_bringup bringup.launch.py headless:=true rviz:=false camera_rate:=$CAM plan_catcher:=$PLAN \
    ${MJ:+mujoco_model:=$MJ} > "$SIMLOG" 2>&1 < /dev/null &
LPID=$!
for i in $(seq 1 240); do
  [ "$(grep -c 'Configured and activated' "$SIMLOG" 2>/dev/null)" -ge 6 ] && break; sleep 0.5
done
sleep 8
# real-time factor and load average are recorded per trial by reactive_eval (summary.rtf, summary.loadavg_1min)
# recordings: failures only, except the fast-gripper cells (all trials: offline-vs-live comparison per throw,
# tools/v2_live/fastclose_diag.py); recording runs on every trial anyway, only the saving differs
REC="--rec-fail-only"; case "$LABEL" in fastclose_medium|fastidealized_medium|armcfg_*) REC="";; esac
ros2 run catch_tasks reactive_eval --throws "$N" --mode "$MODE" --source "$SRC" --noise "$NOISE" --seed "$SEED" \
    --label "$LABEL" --out "$OUT_DIR/$LABEL.json" --rec-dir "$D/$LABEL" $REC > "$D/$LABEL.eval.log" 2>&1
grep SUMMARY "$D/$LABEL.eval.log" | cut -c1-400
kill -INT -- -$LPID 2>/dev/null; sleep 8; kill -9 -- -$LPID 2>/dev/null
echo "CELL_DONE $LABEL"
