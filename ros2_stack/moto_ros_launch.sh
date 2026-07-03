#!/usr/bin/env bash
# moto_ros_launch.sh — single entrypoint the dashboard's Robotics tab spawns.
#
# Sources ROS 2 + the colcon workspace, then execs one stack component so the
# dashboard can start/stop each piece with a button. One component per invocation.
#
# Usage:  moto_ros_launch.sh <component> [arg1] [arg2]
#   sim                 → PyChrono physics + WS telemetry (this repo)
#   rosbridge           → rosbridge_server websocket (ws://localhost:9090) for the dashboard
#   bridge              → moto_ros ws_bridge  (/moto/odom, /tf, /moto/imu)
#   world [kind] [hl]   → moto_gazebo city world + sensor bike (kind=city|straight, hl=true|false)
#   slam                → slam_toolbox 2D mapping
#   multi [n] [hl]      → N-bike convoy
#
# Env:
#   ROS_SETUP  (default /opt/ros/humble/setup.bash)
#   MOTO_WS    (default $HOME/ws_chassis)  — colcon workspace with moto_ros + moto_gazebo
set -eo pipefail
set +u   # ROS 2 setup.bash references unbound vars; clear nounset even if inherited (SHELLOPTS)

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
MOTO_WS="${MOTO_WS:-$HOME/ws_chassis}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # .../moto_sim

if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
else
  echo "[moto_ros_launch] ERROR: ROS setup not found at $ROS_SETUP (set ROS_SETUP)" >&2
  exit 3
fi
if [ -f "$MOTO_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$MOTO_WS/install/setup.bash"
else
  echo "[moto_ros_launch] WARN: workspace not built at $MOTO_WS/install (set MOTO_WS / colcon build)" >&2
fi

# The motorcycle sim runs under CONDA python (has pychrono); ROS nodes must run
# under SYSTEM python 3.10 (rclpy's C-extension is built for 3.10, and conda's
# python3 shadows it → "_rclpy_pybind11 ... isn't present" crash).
#
# Find a pychrono-capable python for the sim. Electron may be launched WITHOUT
# conda active (CONDA_PREFIX unset), so search common conda locations too.
find_sim_python() {
  for cand in \
      ${CONDA_PREFIX:+"$CONDA_PREFIX/bin/python3"} \
      "$HOME/miniconda3/bin/python3" "$HOME/anaconda3/bin/python3" \
      "$HOME/miniconda3/envs/"*/bin/python3 \
      python3; do
    [ -n "$cand" ] || continue
    if "$cand" -c "import pychrono" >/dev/null 2>&1; then echo "$cand"; return 0; fi
  done
  echo "python3"   # fallback; will error clearly if pychrono missing
}
ros_env() {
  # Remove conda's bin so `env python3` resolves to system python 3.10 (rclpy C-ext).
  # Keep PYTHONPATH — ROS setup.bash points it at /opt/ros/.../site-packages (needed
  # for ros2cli etc.); conda uses its binary's own site, not PYTHONPATH, so this is safe.
  if [ -n "${CONDA_PREFIX:-}" ]; then
    PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vF "${CONDA_PREFIX}/bin" | paste -sd: || true)"
  fi
  export PATH="/usr/bin:$PATH"
}

COMP="${1:-}"; shift || true
echo "[moto_ros_launch] component=$COMP  ROS_DISTRO=${ROS_DISTRO:-?}  WS=$MOTO_WS"

# Make every Start idempotent: kill any prior instance of THIS component before
# launching, so leftover processes from earlier runs/sessions never accumulate
# (duplicate gzservers/bridges = invisible bike, NaN odom, blank sensors).
kill_pat() { for pid in $(pgrep -f "$1" 2>/dev/null); do
               [ "$pid" != "$$" ] && kill -9 "$pid" 2>/dev/null || true; done; }
kill_prev() {
  case "$1" in
    sim)        kill_pat "moto_sim_main.py" ;;
    rosbridge)  kill_pat "rosbridge_websocket"; kill_pat "rosapi" ;;
    bridge)     kill_pat "lib/moto_ros/ws_bridge" ;;
    world|fixedcity|multi|oval)
                kill_pat "gzserver"; kill_pat "gzclient"; kill_pat "spawn_entity"
                kill_pat "lib/moto_gazebo/pose_driver"; kill_pat "lib/moto_gazebo/city_streamer"
                kill_pat "lib/moto_gazebo/roam_controller"; kill_pat "lib/moto_gazebo/convoy_ghost"
                kill_pat "lib/moto_gazebo/path_driver" ;;
    slam)       kill_pat "async_slam_toolbox_node"; kill_pat "static_transform_publisher" ;;
    nav2)       for n in controller_server planner_server behavior_server bt_navigator \
                         velocity_smoother lifecycle_manager_navigation \
                         lib/moto_gazebo/cmd_vel_to_moto lib/moto_gazebo/roam_goals; do
                  kill_pat "$n"; done ;;
    ai)         kill_pat "ai/yolo_node.py"; kill_pat "ai/qwen_decision.py" ;;
  esac
  sleep 1
}
kill_prev "$COMP"

case "$COMP" in
  sim)       cd "$REPO_DIR"; SIM_PY="$(find_sim_python)"; echo "[moto_ros_launch] sim python = $SIM_PY"; exec "$SIM_PY" moto_sim_main.py ;;
  rosbridge) ros_env; exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml ;;
  bridge)    ros_env; exec ros2 launch moto_ros bridge.launch.py ;;
  world)     ros_env; exec ros2 launch moto_gazebo endless.launch.py roam:="${1:-true}" headless:="${2:-false}" light:="${3:-false}" ;;
  oval)      ros_env; exec ros2 launch moto_gazebo oval.launch.py headless:="${1:-false}" light:="${2:-true}" ;;
  fixedcity) ros_env; exec ros2 launch moto_gazebo city.launch.py  kind:="${1:-city}" headless:="${2:-false}" ;;
  slam)      ros_env; exec ros2 launch moto_gazebo slam.launch.py ;;
  nav2)      ros_env; exec ros2 launch moto_gazebo nav2.launch.py ;;
  multi)     ros_env; exec ros2 launch moto_gazebo multi_bike.launch.py n:="${1:-3}" kind:=city headless:="${2:-false}" ;;
  ai)        # AI sidecars run in CONDA python (ultralytics/cv2), talk over rosbridge.
             AI_PY="$(find_sim_python)"; AID="$REPO_DIR/ros2_stack/ai"
             echo "[moto_ros_launch] AI python = $AI_PY (YOLO on GPU + Qwen via Ollama/CPU)"
             "$AI_PY" "$AID/yolo_node.py" & YPID=$!
             "$AI_PY" "$AID/qwen_decision.py" & QPID=$!
             trap 'kill $YPID $QPID 2>/dev/null' INT TERM
             wait ;;
  *) echo "[moto_ros_launch] unknown component '$COMP'" >&2; exit 2 ;;
esac
