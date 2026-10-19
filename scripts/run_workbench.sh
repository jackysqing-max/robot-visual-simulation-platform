#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$project_dir/install/local_setup.bash" ]]; then
  echo 'Run ./scripts/setup.sh first.' >&2
  exit 1
fi
if ! /usr/bin/python3 -c 'import PyQt5' >/dev/null 2>&1; then
  echo 'Install the UI dependency: sudo apt-get install python3-pyqt5' >&2
  exit 1
fi
source "$project_dir/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
exec ros2 run lbr_pybullet_tutorial workbench "$@"
