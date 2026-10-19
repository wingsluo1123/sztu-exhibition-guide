#!/usr/bin/env bash
# Operator-started only. No execution when called without --execute.
set -eo pipefail
if [ "$#" -ne 2 ] || { [ "$1" != "--preview" ] && [ "$1" != "--execute" ]; }; then
    echo "Usage: bash tools/run_restrained_handoff.sh --preview|--execute NEW_OUTPUT_PREFIX" >&2
    echo "Execute requires fixed fall protection, operator at independent damping stop, and one authorized attempt." >&2
    exit 2
fi
task_trial_mode="$1"
task_trial_prefix="$2"
task_trial_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$(dirname "$task_trial_prefix")"
if [ -e "$task_trial_prefix.json" ] || [ -e "$task_trial_prefix.log" ]; then
    echo "Existing evidence: choose an unused output prefix; do not repeat a physical trial." >&2
    exit 2
fi
# Persistent reservation prevents accidental reruns even if the process crashes.
mkdir "$task_trial_prefix.lock"
task_trial_arguments=(--require-mc-state --speed 0.10 --duration 1.0 --start-delay 5
                      --output "$task_trial_prefix.json")
if [ "$task_trial_mode" = "--execute" ]; then
    source /opt/ros/humble/setup.bash
    source "$task_trial_root/install/setup.bash"
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
    export PYTHONPATH="/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}"
    export FASTDDS_DEFAULT_PROFILES_FILE="$task_trial_root/src/sztu_slam/config/fastdds_large_cloud.xml"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_DEFAULT_PROFILES_FILE"
    task_trial_arguments+=(--execute --restrained --operator-stop-ready)
    echo "PREFLIGHT: one restrained pulse only; wait for PULSE_STARTED. No automatic retry or standing recovery."
fi
python3 "$task_trial_root/tools/restrained_drive_trial.py" "${task_trial_arguments[@]}" 2>&1 |
    tee "$task_trial_prefix.log" |
    python3 -u -c 'import sys
for line in sys.stdin:
    if line.startswith(("PREFLIGHT", "PULSE_", "dry_run:", "{", "Traceback", "Error")):
        print(line, end="", flush=True)'
echo "RECORDING_COMPLETE: inspect $task_trial_prefix.json and .log; keep current mode."
