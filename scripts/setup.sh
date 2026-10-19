#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$project_dir"
if ! /usr/bin/python3 -c 'import pybullet, rclpy, numpy' >/dev/null 2>&1; then
  echo 'Install ROS-compatible PyBullet: /usr/bin/python3 -m pip install --user -r requirements.txt' >&2
  exit 1
fi
/usr/bin/python3 -m colcon build --symlink-install --base-paths src \
  --packages-select lbr_pybullet_control lbr_pybullet_tutorial \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
printf 'Build complete. Source %s/install/local_setup.bash\n' "$project_dir"
