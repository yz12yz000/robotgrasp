# MoveIt 抓取修改与测试记录

日期：2026-09-21。环境：本机 ROS 2 Humble、现有 MoveIt/UR 工作区。
本次完成源码修改、构建、自动测试与隔离模型验证，没有连接机械臂进行真实运动。

## 完成的行为

抓取后端改为以当前关节状态求 IK，OMPL 规划接近，Cartesian 规划下降/抬升，
经 MoveIt ExecuteTrajectory 映射到 scaled_joint_trajectory_controller。
不用 Pilz，不需要在本机安装新运行依赖。正式执行前验证三段完整路径；默认只发布预览。
工具转换包含旋转和平移，目标过期、不可达、碰撞、不完整路径和执行失败均有拒绝/停止处理。
静止目标有效期与等待时间改为 600 秒，保留真实原始时间戳与重复执行保护。
按用户要求不添加未标定桌面，自碰撞和已有场景检查保留。

## 本次新增文件

| 文件 | 内容 |
|---|---|
| [moveit_ros.py](../src/grasp_executor/grasp_executor/moveit_ros.py) | MoveIt 规划、工具转换、场景检查、Action 执行和停止后端 |
| [trajectory.py](../src/grasp_executor/grasp_executor/trajectory.py) | 时间/关节轨迹验证、五次插值采样、统一减速 |
| [test_grasp.py](../src/grasp_executor/test/test_grasp.py) | 38 项抓取逻辑、轨迹、Action 失败/停止测试 |
| [test_moveit_grasp.py](../tools/test_moveit_grasp.py) | 真实 MoveIt/模型与模拟控制器的隔离集成测试 |
| [本报告](moveit_grasp_test_report.md) | 修改清单、测试结果和现场边界 |

## 本次修改文件

| 文件 | 内容 |
|---|---|
| [config.py](../src/grasp_executor/grasp_executor/config.py)、[config.json](../src/grasp_executor/config/config.json) | 默认预览、600 秒、MoveIt 参数及校验 |
| [grasp.py](../src/grasp_executor/grasp_executor/grasp.py) | 先规划全程再执行、抬升用直线、实际检查 Z 下限 |
| [interfaces.py](../src/grasp_executor/grasp_executor/interfaces.py) | 选择新后端；预览跳过夹爪；夹爪命令后等待 |
| [main.py](../src/grasp_executor/grasp_executor/main.py) | 模式参数、planned 状态、执行结果标注未验证夹持 |
| [grasp_executor.launch.py](../src/grasp_executor/launch/grasp_executor.launch.py)、[pipeline.launch.py](../src/grasp_executor/launch/pipeline.launch.py) | 暴露 plan_only；组合启动默认禁用抓取 |
| [package.xml](../src/grasp_executor/package.xml)、[setup.py](../src/grasp_executor/setup.py) | 声明 ROS 消息依赖和 pytest，使 colcon test 真正运行用例 |
| [run_grasp_executor.sh](../run_grasp_executor.sh) | 加载全部环境；默认预览；--execute 执行 |
| [run_grasp_position_control.sh](../run_grasp_position_control.sh) | 更新与新抓取后端的衔接说明 |
| [robot_control.py](../tools/robot_control.py)、[test_robot_control.py](../tools/test_robot_control.py) | 就绪检查覆盖 IK/FK/Cartesian/场景服务；更新模拟接口 |
| [startup_scripts.md](startup_scripts.md)、[hardware_adapter.md](hardware_adapter.md)、[run.md](../run.md) | 启动顺序、模式关系、参数与验证边界 |

已有 `run_robot_prepare.sh`、`run_joystick_control.sh` 继续使用。
第一、第二模块的算法和 `run_yolo_vision.sh`、`run_rim_locator.sh` 没有改动。
旧 FZI 文件保留，但新启动流程不会调用它们。

## 测试结果

| 测试 | 结果 |
|---|---|
| colcon build --symlink-install --packages-select grasp_executor | 成功 |
| colcon test --packages-select grasp_executor | 38 passed，0 failed，0 skipped |
| colcon test-result --test-result-base build/grasp_executor --verbose | 38 tests，0 errors，0 failures |
| ROBOT_STARTUP_ROS_TEST=1 /usr/bin/python3 -B tools/test_robot_control.py | 14 项通过，包含隔离 ROS 图与进程/锁检查 |
| /usr/bin/python3 -B tools/test_moveit_grasp.py | 5 项通过，详见下面 |
| 四个相关入口脚本和共享 helper 的 bash -n | 通过 |
| 抓取包、launch、相关 Python 工具 compileall | 通过 |
| 抓取/位置控制脚本 --help | 正常，帮助不会启动节点 |

38 项测试涵盖：包含旋转的工具变换、10 分钟过期边界、规划期间过期、工作空间下限、
预览不执行/不写目标记录、全部规划先于执行、失败保留目标记录、轨迹格式和时间缩放、
Cartesian 部分完成拒绝、IK 种子/失败、Action 拒绝/失败/迟到接受时取消、到位条件、
超时、起点改变拒绝，以及取消未确认时停用控制器并明确报错。

5 项集成检查使用本机真实 MoveIt、项目 UR5e+夹爪模型：

1. OMPL 接近 + Cartesian 下降/抬升完整规划，并收到三段 DisplayTrajectory；预览没有执行命令。
2. 真正的 ExecuteTrajectory Action 经现有 MoveIt 映射到模拟 FollowJointTrajectory 服务端，
   三段执行完成，模拟夹爪收到 Open、Close。
3. 非零 grasp_center 工具偏移被正确转换，实时 TF 与 MoveIt FK 相符。
4. base_link 下 (1.4, 1.4, 1.4) 不可达目标被拒绝：`approach_ik_failed:-31`。
5. 在隔离场景添加与夹爪重叠的测试箱体，规划被拒绝：
   `collision_or_invalid_state:test_obstacle/gripper_base_link`。

最后一次集成测试使用 localhost ROS_DOMAIN_ID=213，未启动 UR 驱动、硬件控制接口或夹爪命令。
测试进程已退出。日志：`/tmp/grasp-integration-final.log`，详细 MoveIt 日志及结果：
`/tmp/moveit-grasp-test-h_0p017n/`；colcon 报告：`build/grasp_executor/pytest.xml`。
规划出的测试轨迹时长约为接近 2.52 秒、下降 5.49 秒、抬升 5.49 秒；
模拟控制器只验证消息/状态链路，并不模拟真实机械臂动力学或跟踪误差。
测试订阅者会报告与 MoveIt 自带 volatile 预览发布者的 QoS 不匹配；本后端的 retained
三段预览已经被测试订阅者收到，该提示不代表本后端预览失败。

## 仍需现场解决/验证

- 没有桌面实测标定，未建模桌面不会被碰撞检查发现；也没有被抓物体的附着碰撞模型。
- 相机外参、物理 TCP、固定抓取姿态、夹持偏移和现场工作空间需核实。
- 夹爪仅有命令完成与等待，没有闭环到位/夹持成功 Topic；success 仍带 grasp_verified=false。
- 真机 UR 的轨迹跟踪、速度缩放、保护停止，以及实际抓取和抬升效果尚未验收。

使用步骤以 [startup_scripts.md](startup_scripts.md) 为准：
硬件/控制器就绪 → 抓取和定位等待 → YOLO 生成新目标。
`./run_grasp_executor.sh` 只预览，`./run_grasp_executor.sh --execute` 才执行。
