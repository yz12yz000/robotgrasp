#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_place_execute.sh [--vision-config 配置.json] [--timeout 900]
观察当前视野全部碗：0 个结束且不启动抓取，1 个抓放 1 个，N 个从近到远逐个抓放。
不再选择单／多物体模式；默认全视野，显式配置 ROI 时只处理该区域内的碗。
默认继承 place_config.json，--vision-config 只覆盖显式字段，不改写源配置。
有有效定位目标后才启动位置控制器并真实移动机械臂、开合夹爪。
执行前必须确认固定放置点和接收区域已经通过 plan-only 验收。
EOF
    exit 0
fi
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
# Do not start a controller until observation has produced graspable targets.
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/test_live_place_pipeline.py" --execute "$@"
