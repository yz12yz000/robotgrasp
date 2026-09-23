#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_grasp_executor.sh [--plan-only|--execute] [config_path:=路径]
默认仅规划并发布 RViz 预览，不驱动机械臂或夹爪。
--execute 执行抓取，先运行 run_grasp_position_control.sh。
使用 MoveIt OMPL + Cartesian 路径，无需 Pilz。静止目标有效期默认 600 秒。
不加载桌面模型和额外夹爪保护盒；规划前自动移除旧场景中的这两个对象。
保留机器人 URDF 自碰撞、关节限位和轨迹校验。
EOF
    exit 0
fi
plan_only=true
case "${1:-}" in
    --execute) plan_only=false; shift ;;
    --plan-only) shift ;;
esac
for argument in "$@"; do
    case "$argument" in
        plan_only:=*|--*) echo '请使用 --plan-only 或 --execute 选择模式。' >&2; exit 2 ;;
    esac
done
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
echo "grasp_executor: plan_only=$plan_only"
exec ros2 launch grasp_executor grasp_executor.launch.py plan_only:="$plan_only" "$@"
