#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$project_dir/install/local_setup.bash"
cd "$project_dir"
/usr/bin/python3 -m pytest tests -q
smoke_flags=()
check_existing=false
for flag in "$@"; do
  case "$flag" in
    --existing) check_existing=true ;;
    --upstream) smoke_flags+=(--upstream) ;;
    *) echo "Unknown option: $flag" >&2; exit 2 ;;
  esac
done
/usr/bin/python3 scripts/smoke_test.py --output artifacts/smoke_tutorial "${smoke_flags[@]}"
if [[ "$check_existing" == true ]]; then
  if [[ -n "${LBR_TUTORIAL_LEGACY_WS:-}" ]]; then
    source "$LBR_TUTORIAL_LEGACY_WS/install/local_setup.bash"
  elif ! ros2 pkg prefix pybullet_ros2_sim >/dev/null 2>&1; then
    echo 'For --existing, set LBR_TUTORIAL_LEGACY_WS to the built pybullet_ros2_sim workspace.' >&2
    exit 1
  fi
  /usr/bin/python3 scripts/smoke_test.py --backend existing --domain 90 --output artifacts/smoke_existing
fi
