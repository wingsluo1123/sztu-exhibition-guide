#!/usr/bin/env bash
# Source this file. Trial-local dependencies; no changes to /opt/ros or ~/.bashrc.
TRIAL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source /home/sztu/aimdk/install/setup.bash
if [[ -f "$TRIAL_ROOT/install/setup.bash" ]]; then
  source "$TRIAL_ROOT/install/setup.bash"
fi
export AMENT_PREFIX_PATH="$TRIAL_ROOT/vendor/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$TRIAL_ROOT/vendor/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$TRIAL_ROOT/vendor/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"

