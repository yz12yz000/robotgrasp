#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source_setup() {
    local setup_file="$1"
    if [[ ! -f "$setup_file" ]]; then
        echo "错误：找不到环境文件 $setup_file" >&2
        exit 1
    fi
    set +u
    # ROS 2 setup files may reference optional unset variables.
    source "$setup_file"
    set -u
}

source_setup /opt/ros/humble/setup.bash
source_setup "$WORKSPACE/install/setup.bash"

exec ros2 launch rim_locator rim_locator.launch.py "$@"
