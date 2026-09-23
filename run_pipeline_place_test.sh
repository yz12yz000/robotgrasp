#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_place_test.sh [--vision-config 配置.json] [--timeout 360]
观察当前视野全部碗：0 个结束，1 个规划 1 个，N 个从近到远逐个规划完整抓放。
不再选择单／多物体模式；默认全视野，显式配置 ROI 时只处理该区域内的碗。
默认继承 place_config.json，--vision-config 只覆盖显式字段，不改写源配置。
日志目录保存三份 effective_*_config.json、检测数量、有效数量及拒绝原因。
只做 plan-only，不切换控制器、不移动机械臂、不驱动夹爪；不会覆盖旧 run_pipeline_test.sh。
前提：run_robot_prepare.sh 已完成，相机、TF、MoveIt 和工作空间已就绪。
EOF
    exit 0
fi
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
# Vision validates one actual RGB/depth observation using the effective config.
# A camera timeout is distinct from a successful observation with zero targets.
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/test_live_place_pipeline.py" "$@"
