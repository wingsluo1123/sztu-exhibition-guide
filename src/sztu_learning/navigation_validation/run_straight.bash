#!/usr/bin/env bash
set -e
source "$HOME/.aima/env/bashrc" >/dev/null 2>&1
validation_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export PYTHONPATH="$validation_dir/runtime_deps:/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}"
exec python3 "$validation_dir/straight_trial.py" "$@"
