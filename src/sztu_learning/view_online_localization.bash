#!/usr/bin/env bash
set -e
learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$learning_dir/previous_learning/view_online_localization.bash" "$@"
