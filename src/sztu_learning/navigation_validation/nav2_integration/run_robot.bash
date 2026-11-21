#!/usr/bin/env bash
set -e
source "$HOME/.aima/env/bashrc" >/dev/null 2>&1
source "$HOME/sztu_learning/online_fastlio_ws/install/setup.bash"
nav2_learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 MPLBACKEND=Agg
export LEARNING_LOCALIZATION_ROOT="$HOME/sztu_learning/live_release01"
export PYTHONPATH="$nav2_learning_dir/../runtime_deps:/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}"
case "$1" in
  bringup|mission) nav2_entry="$1"; shift ;;
  *) echo 'Use bringup or mission'; exit 2 ;;
esac
exec python3 "$nav2_learning_dir/$nav2_entry.py" "$@"
