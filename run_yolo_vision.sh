#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${YOLO_VENV:-/home/rob/yolovision_ws/.venv}"

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

if [[ -f "$VENV/bin/activate" ]]; then
    source_setup "$VENV/bin/activate"
    # ROS cv_bridge is compiled against NumPy 1.x. Do not let the user's
    # ~/.local NumPy 2.x override the virtualenv version.
    export PYTHONNOUSERSITE=1
    # Ultralytics is installed in the user's site on this machine. Add that
    # path explicitly after the virtualenv path so NumPy still resolves to
    # the compatible 1.26.x wheel.
    export PYTHONPATH="$VENV/lib/python3.10/site-packages:/home/rob/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
source_setup /opt/ros/humble/setup.bash
source_setup "$WORKSPACE/install/setup.bash"

exec ros2 launch yolo_vision yolo_vision.launch.py "$@"
