#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-bringup}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$mode" == -h || "$mode" == --help ]]; then
  echo 'Usage: run_demo.sh bringup|sim|joint|cartesian [gui:=true] [sim_backend:=tutorial|external|existing]'
  echo 'sim starts only the standalone PyBullet node. external attaches control to an already-running simulator.'
  echo 'existing requires LBR_TUTORIAL_LEGACY_WS pointing to an optional pybullet_ros2_sim workspace.'
  exit 0
fi
if [[ "$mode" != bringup && "$mode" != sim && "$mode" != joint && "$mode" != cartesian ]]; then
  echo 'Usage: run_demo.sh bringup|sim|joint|cartesian [gui:=true] [sim_backend:=tutorial|external|existing]' >&2
  exit 2
fi
backend=tutorial
for argument in "$@"; do
  if [[ "$argument" == sim_backend:=* ]]; then
    backend="${argument#sim_backend:=}"
  fi
done
source /opt/ros/humble/setup.bash
if [[ ! -f "$project_dir/install/local_setup.bash" ]]; then
  echo 'Run ./scripts/setup.sh first.' >&2
  exit 1
fi
# An optional legacy backend is loaded only when explicitly selected. The
# default simulator and external-node mode never source a neighboring project.
if [[ "$mode" != sim && "$backend" == existing ]]; then
  if [[ -n "${LBR_TUTORIAL_LEGACY_WS:-}" ]]; then
    legacy_ws="$LBR_TUTORIAL_LEGACY_WS"
    if [[ ! -f "$legacy_ws/install/local_setup.bash" ]]; then
      echo "Legacy workspace is not built: $legacy_ws/install/local_setup.bash" >&2
      exit 1
    fi
    source "$legacy_ws/install/local_setup.bash"
  elif ! ros2 pkg prefix pybullet_ros2_sim >/dev/null 2>&1; then
    echo 'For sim_backend:=existing, set LBR_TUTORIAL_LEGACY_WS to the built pybullet_ros2_sim workspace.' >&2
    exit 1
  fi
fi
source "$project_dir/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
if [[ "$mode" == bringup ]]; then
  exec ros2 launch lbr_pybullet_tutorial tutorial.launch.py "$@"
fi
if [[ "$mode" == sim ]]; then
  exec ros2 launch lbr_pybullet_tutorial sim.launch.py "$@"
fi
mkdir -p "$project_dir/artifacts"
launch_pid=''
cleanup() {
  if [[ -n "$launch_pid" ]]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# Start launch in its own foreground process so SIGINT reaches launch's handlers.
/usr/bin/python3 "$project_dir/scripts/launch_process.py" "$@" >"$project_dir/artifacts/${mode}_launch.log" 2>&1 &
launch_pid=$!
ros2 run lbr_pybullet_tutorial demo --mode "$mode" \
  --report "$project_dir/artifacts/${mode}_report.json"
