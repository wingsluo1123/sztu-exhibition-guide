#!/usr/bin/env bash
set -e
source "$HOME/.aima/env/bashrc" >/dev/null 2>&1
source "$HOME/sztu_learning/online_fastlio_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
export MPLBACKEND=Agg
learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$learning_dir/online_robot_session.py" "$@"
