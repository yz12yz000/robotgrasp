#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_place_execute.sh [--vision-config 配置.json] [--timeout 900]
启动新的多碗抓取、放置、释放执行流程；每个碗仅一个 Pose，按近到远执行。
会启动位置控制器并真实移动机械臂、开合夹爪；不会覆盖旧 run_pipeline_execute.sh。
执行前必须确认固定放置点和接收区域已经通过 plan-only 验收。
EOF
    exit 0
fi
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
probe absent grasp_executor rim_locator yolo_vision
probe wait-camera
probe planning-ready
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/test_live_place_pipeline.py" --execute "$@"
