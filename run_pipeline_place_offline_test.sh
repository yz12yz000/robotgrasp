#!/usr/bin/env bash
# Isolated simulation only: never starts a camera or hardware driver.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
cd "$ROBOT_WORKSPACE"
unset ROBOT_STARTUP_ROS_TEST
colcon build --symlink-install --packages-select yolo_vision rim_locator grasp_executor
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
export PYTHONPATH="$ROBOT_WORKSPACE/src/grasp_executor:$ROBOT_WORKSPACE/src/rim_locator:$ROBOT_WORKSPACE/src/yolo_vision:${PYTHONPATH:-}"
"$ROBOT_PYTHON" -m pytest -q src/grasp_executor/test src/rim_locator/test src/yolo_vision/test \
    tools/test_place_runner.py tools/test_pipeline_runner.py tools/test_robot_control.py tools/test_joint_stationarity.py
# Chooses an unoccupied ROS domain, restricts discovery to localhost, and
# starts only MoveIt + robot_state_publisher + synthetic feedback/actions.
exec "$ROBOT_PYTHON" tools/test_moveit_grasp.py
