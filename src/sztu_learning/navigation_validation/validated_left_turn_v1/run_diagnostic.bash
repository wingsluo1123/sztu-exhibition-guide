#!/usr/bin/env bash
set -e
source "$HOME/.aima/env/bashrc" >/dev/null 2>&1
source "$HOME/sztu_learning/online_fastlio_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 MPLBACKEND=Agg
export LEARNING_LOCALIZATION_ROOT="$HOME/sztu_learning/live_release01"
validation_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$validation_dir/diagnostic_session.py" "$@"
