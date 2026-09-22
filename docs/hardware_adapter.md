# MoveIt 抓取后端

当前入口已从 FZI 位置 Topic 改为现有 MoveIt + UR 轨迹控制器，不需要安装 Pilz。
本机已具备 `moveit_msgs`、OMPL、KDL、UR 控制器和 MoveIt 控制器映射。
本次没有修改外部工作区的 URDF、SRDF、控制器 YAML 或 MoveIt 安装包。
旧 `cartesian_ros.py` 和 `docs/switch_arm_control_mode.sh` 保留作历史参考，当前启动流程不使用它们。

操作步骤见 [startup_scripts.md](0921/startup_scripts.md)，此处说明实现和验证边界。

## 接口与动作

- 输入仍是 `/rim_locator/rim_point`，`PointStamped`，`base_link` 坐标系。
- 抓取坐标是 `grasp_center`，规划组 `ur_manipulator` 的末端是 `tool0`。
- 读取当前关节状态和真实 TF，以当前关节状态为种子调用 `/compute_ik`，选定接近点关节解。
- `/plan_kinematic_path` 用现有 OMPL 规划到接近点；`/compute_cartesian_path` 规划竖直下降、抬升。
- 三段必须先全部规划通过。Cartesian 完成率必须为 100%，开启 `avoid_collisions`，拒绝关节跳变。
- 对轨迹的关节限位、时间、导数、采样状态碰撞、工具 FK、直线偏差和最终到位误差进行验证。
- 在 `/display_planned_path` 发布三段完整预览；`planned` 表示规划成功，不代表执行过。
- 执行模式通过 `/execute_trajectory` 交给 MoveIt，再由其现有映射发送到
  `/scaled_joint_trajectory_controller/follow_joint_trajectory`。
- 顺序：接近 → 打开夹爪 → 竖直下降 → 关闭夹爪 → 竖直抬升到接近点。

工具转换使用完整刚体变换（包含平移和旋转）：

```text
T_base_tool0 = T_base_grasp_center × inverse(T_tool0_grasp_center)
```

启动时检查工具链是固定关节链，并核对 MoveIt FK 与实时 TF。
`grasp_offset` 是视觉点到期望夹持中心的修正，不能重复填写工具长度补偿。

## 参数

配置在 `src/grasp_executor/config/config.json`。可用
`./run_grasp_executor.sh config_path:=/绝对路径/现场配置.json` 指定现场参数。
脚本/launch 的模式开关优先于 JSON 的 `plan_only`；真实执行显式加 `--execute`。

| 参数 | 默认/含义 |
|---|---|
| `max_target_age`、`input_timeout` | 600 秒；目标年龄与等待目标超时分别检查 |
| `fixed_orientation`、`grasp_offset` | 沿用原配置，须按现场夹爪姿态/夹持点核实 |
| `approach_height` | 0.15 m |
| `planning_link`、`tool_frame` | `tool0`、`grasp_center` |
| `workspace_min/max` | 限制工具 XYZ；现在也实际检查 Z 下限，默认 -0.10 m |
| `velocity_scaling`、`acceleration_scaling` | OMPL 请求均为 0.1 |
| `approach_speed`、`descend_speed` | TCP 采样限速 0.03、0.01 m/s；抬升也用 descend_speed |
| `angular_speed` | TCP 采样角速度限值 0.15 rad/s |
| `max_joint_velocity/acceleration` | 额外限值 0.3 rad/s、0.3 rad/s² |
| `cartesian_step`、`max_joint_step` | 5 mm 插值步长，0.25 rad 相邻轨迹点跳变阈值 |
| `planning_timeout`、`motion_timeout` | 每段规划及检查 60 秒；每段执行 90 秒 |
| `position_tolerance`、`orientation_tolerance` | 到位 2 mm、0.02 rad；静止保持 0.25 秒 |
| `table_enabled` | false；不自动加入未经标定的桌面 |
| `require_collision_world_for_execution` | false；无世界障碍模型也允许通过此项检查 |
| `gripper_settle_time` | Open/Close 命令退出后等待 1 秒 |

本机 MoveIt 的 Cartesian 服务返回带时间轨迹，但不接收 OMPL 的速度缩放参数。
适配器检查控制器使用的五次插值采样，统一延长整段时间并同步缩放速度/加速度，
保持轨迹几何形状；过长而无法在 motion_timeout 内执行的轨迹直接拒绝。
这些是基于模型的离散检查，不构成连续路径或真机跟踪误差的数学保证。

保留但当前 MoveIt 后端不使用的旧 FZI 参数为 `controller_namespace`、
`controller_tip_frame`、`command_rate`、`max_tracking_error`。

## 失败与停止

规划失败、目标过期、工具链错误、工作空间越界、碰撞、Cartesian 不完整均不发送运动目标。
每段执行前检查控制器、External Control、当前位置，并对缓存路径重新检查当前场景。
Action 成功、实际 TF 到位及新鲜静止反馈同时满足，才继续夹爪/下一段动作。

执行前将目标原时间戳和坐标写入 `execution_journal`，失败也保留记录，防止节点重启后重复执行。
重试应重新识别定位。纯预览不写执行记录。
Ctrl+C、执行错误和超时会取消 Action；未确认取消时尝试停用本任务轨迹控制器。
停止还需观察新的持续静止反馈；仅停用控制器但未确认 Action 结束时会报告 `stop_error`。
模式锁仅约束本项目入口，无法阻止其他外部程序直接发控制命令。

## 目前未验证的现场事项

没有实测桌面，不添加桌面碰撞模型；保留自碰撞和已有障碍物检查。
模型不能拒绝与未建模桌面发生的碰撞。启用 `table_enabled` 前必须填写实测
`table_center` 和 `table_size`，当前数值只是禁用状态下的示例。

相机外参、`grasp_center` 的物理 TCP、固定抓取姿态和夹持偏移仍需现场核实。
假定对象、相机和底盘相对位置不变；延长到 10 分钟不会重新测量目标。
夹爪驱动暂无可用的到位/夹持成功 Topic，本次只检查命令退出并等待，成功状态仍标明
`grasp_verified=false`；不会声称已经抓住物体。抬升规划未将被抓物体附着到场景。
真实 UR 运动、速度缩放、保护停止和实际夹持效果需要现场验收。

## 可复现测试

加载 `startup_scripts.md` 中的 ROS 环境后：

```bash
colcon build --symlink-install --packages-select grasp_executor
source install/setup.bash
/usr/bin/python3 -m pytest -q src/grasp_executor/test
ROBOT_STARTUP_ROS_TEST=1 /usr/bin/python3 -B tools/test_robot_control.py
/usr/bin/python3 -B tools/test_moveit_grasp.py
```

最后两项选择随机空闲的 localhost DDS domain；若发现已被使用会退出，可重新运行。
`test_moveit_grasp.py` 启动本机真实 MoveIt、项目 UR5e/夹爪模型与合成关节反馈，
验证三段规划、工具变换、不可达目标拒绝、模拟障碍碰撞拒绝，以及真实 MoveIt Action
到模拟 FollowJointTrajectory 控制器的三段执行/夹爪命令顺序。
不会启动 UR 驱动、ros2_control 硬件接口或真实夹爪命令。
测试日志保存在打印的 `/tmp/moveit-grasp-test-*` 目录。
