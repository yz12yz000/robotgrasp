# 2026-09-27 实机验证与运行器监督修复

本次按用户授权连接现场相机、UR 机械臂、夹爪和 MoveIt，用户在场并确认沿用原固定放置点、通道及接收区清空。原始记录位于 [live_demo_20260927_gjgip1](../debug_output/live_demo_20260927_gjgip1/)。

## 实际检查

- 只读预检确认驱动、External Control、静止关节、TF、MoveIt 与任务互斥通过；未重复启动现有设备。
- 初始同步 RGB/深度识别到白碗、绿碗，保存原始时间戳、TF、点云及图像。初始回放每碗有 8 个候选；候选数随新帧有效几何变化。
- 固定放置 TCP 在图像中约为 `(460, 433)`，同样处于全相机视野。取物区域据实际图像设置为 `[610, 385, 875, 555]`，包括现场两只待抓碗并排除左侧放置区。
- 两只碗的 plan-only 六段路径均通过，未发送动作。实机执行仍重新采集目标、重新规划，不复用此前快照。

现场 ROI 保存为 [live_pick_roi_20260927.json](../src/yolo_vision/config/live_pick_roi_20260927.json)，是当前相机位置和工作台布置对应的配置，不是通用标定。默认全视野配置没有被静默覆盖。

## 白碗及中途异常

白碗在现场完成接近、闭爪、抬升、转移到固定放置点及释放。相机 [lift_01.png](../debug_output/live_demo_20260927_gjgip1/lift_01.png) 与 [after_stop.png](../debug_output/live_demo_20260927_gjgip1/after_stop.png) 分别记录了带起和放下的状态。

首次执行终端会话异常结束，退出码为 143；运行器进程消失、未生成汇总结果，但独立执行节点仍在执行本轮轨迹。发现后对本次所有节点所在进程组发出 SIGINT，通过原停止流程停止机械臂。执行器报告 `failed_step=retreating_from_place, reason=cancelled`，并有 `controller_deactivated_but_action_cancel_unconfirmed`；随后再次只读预检确认机械臂静止、任务互斥及控制接口正常。

因此，**首次执行不能记为完整流程成功**。恢复诊断另存 [recovered_execution_diagnostic.json](../debug_output/live_demo_20260927_gjgip1/execute_01/recovered_execution_diagnostic.json)，没有伪造缺失的 runner result。终端会话具体被谁终止仍未确定；已确认并修复的是执行器未主动检测运行器消失这一监督缺口。

## 修复及测试

- 新增 `src/grasp_executor/grasp_executor/process_guard.py`：检查 Linux PID 和进程启动时钟，拒绝僵尸/已退出进程，并防止 PID 复用被误认为原运行器。
- `src/grasp_executor/grasp_executor/main.py` 增加 `supervisor_pid`、`supervisor_start_ticks` 参数。50 ms 稳态定时器检测运行器消失时，触发现有取消/停止流程；等待输入时直接失败，不发送动作，不自动释放或重抓。
- `grasp_executor.launch.py`、`pipeline.launch.py` 透传参数；旧独立入口默认 0/0 保持兼容。
- `tools/test_live_place_pipeline.py` 每轮记录并传递真实运行器身份；关闭 rclpy 默认信号处理，显式处理 SIGINT、SIGTERM、SIGHUP，让正常终止请求进入所属进程组清理和结果落盘。
- 后续实机任务使用直接文件日志和独立进程会话，保留 PID、启动身份和全局截止时间，避免依赖交互终端/tee 管道寿命。执行器仍监督该运行器；这不改变超时、停止和运动限制。
- 新增 `src/grasp_executor/test/test_process_guard.py`、`tools/test_place_signal_cleanup.py`；扩展 `tools/test_place_runner.py`、`tools/test_place_launch_contract.py`；离线入口接入新测试。

修复后验证：

| 检查 | 结果 |
|---|---|
| 最终构建 | 3 个包成功，现场 ROI 配置已安装 |
| 相关执行器与运行器回归 | 194 passed |
| 全量离线回归 | 503 passed，7 个可选隔离启动测试在下一项单独运行 |
| 隔离假 ROS 启动回归 | 7 passed，24.23 秒 |
| 真实 launch 参数/监督检查，独立空闲 DDS 域 | 32 项通过，含杀死假运行器后执行节点无动作失败 |
| 实际 runner 信号清理，独立空闲 DDS 域 | SIGTERM、SIGHUP 两项通过，结果落盘且所属节点全部清理 |
| 现场新观测与重新规划 | 只检测到剩余绿碗，完整六段路径通过 |

上述隔离测试没有连接现场驱动、相机或控制器。运动期间运行器消失后的取消/停止、禁止后续释放和禁止继续下一段，使用假接口回归验证，未故意对运动中的实机注入进程崩溃。

## 修复后的实机执行结果

绿碗的新观测产生 5 个有效抓取候选，candidate 0 的完整六段路径通过。随后实机完成接近、抓取、抬升、转移、下降、释放及退回，执行器报告 `success`。运行器在退回结束后自动启动第二轮重新观测，取物区检测到 0 个目标，正常结束。没有把第一轮剩余候选当成其他物体继续抓取，也没有复用旧坐标。

原始汇总为 [execute_02_after_fix/result.json](../debug_output/live_demo_20260927_gjgip1/execute_02_after_fix/result.json)：

| 字段 | 实际结果 |
|---|---|
| `passed / outcome` | `true / completed` |
| `completed_count / observation_count` | `1 / 2` |
| `candidate_attempt_count` | `1` |
| 第一轮原始采集时间 `source_stamp_ns` | `1790484404915154944` |
| 退回后第二轮采集下界 `source_not_before_ns` | `1790484622065387506` |
| 第二轮原始采集时间 `source_stamp_ns` | `1790484626303715072`，严格晚于新一轮下界 |
| 两轮所属节点清理 | 无 `cleanup_errors` |
| 最终只读预检 | 静止关节、驱动、External Control、TF、MoveIt 和任务互斥通过 |

最后的 [final_scene.png](../debug_output/live_demo_20260927_gjgip1/final_scene.png) 记录了取物台面为空；[final_preflight.log](../debug_output/live_demo_20260927_gjgip1/final_preflight.log) 记录静止检查。单张空台面图不能单独证明抓取和放置成功，动作过程须结合逐段执行记录与现场观察判断。程序没有增加夹持反馈，仍保留未由传感器确认的标志。

两次运行的简要对照及采集时间校验保存为 [live_verification_summary.json](../debug_output/live_demo_20260927_gjgip1/live_verification_summary.json)。最终构建、隔离启动回归日志分别为 `final_build.log`、`final_fake_startup_tests.log`，全部日志位于本次记录目录；`git diff --check` 与离线入口 Bash 语法检查通过。

本次完整成功计数是修复后 **1 次**，不能将首次白碗退回被取消的运行计入完整成功次数。绿碗在新观测中从原来的物体编号 1 变为 0，流程正常，符合“不依赖跨帧 YOLO 编号”的设计。

## 本次修改文件

以下为本次实机调试增加或修改的文件；此前同碗多候选实现的完整文件清单见 [2026-09-25 实施记录](demo_implementation_20260925.md)。

| 文件 | 用途 |
|---|---|
| `src/grasp_executor/grasp_executor/process_guard.py` | 新增运行器进程身份检查 |
| `src/grasp_executor/grasp_executor/main.py` | 接收、校验和监测运行器身份，丢失时进入取消流程 |
| `src/grasp_executor/launch/grasp_executor.launch.py` | 传递监督参数 |
| `src/grasp_executor/launch/pipeline.launch.py` | 完整入口传递监督参数 |
| `tools/test_live_place_pipeline.py` | 记录监督身份，处理终止信号并清理本任务节点 |
| `src/grasp_executor/test/test_process_guard.py` | 进程身份、运行器丢失及取消回归 |
| `tools/test_place_runner.py` | 更新运行器参数和初始化契约测试 |
| `tools/test_place_launch_contract.py` | 真正启动节点验证监督参数和丢失处理 |
| `tools/test_place_signal_cleanup.py` | 新增真实运行器 SIGTERM/SIGHUP 清理测试 |
| `run_pipeline_place_offline_test.sh` | 接入信号清理测试 |
| `src/yolo_vision/config/live_pick_roi_20260927.json` | 保存本次实际取物区域配置 |
| `New_run/README.md`、`docs/README.md`、本文 | 使用说明与实机验证记录 |

## 当前场景复现入口

保持设备准备环境运行，在工作区根目录先预览，再执行：

```bash
./run_pipeline_place_test.sh --vision-config src/yolo_vision/config/live_pick_roi_20260927.json
./run_pipeline_place_execute.sh --vision-config src/yolo_vision/config/live_pick_roi_20260927.json
```

在 `New_run` 目录调用时，配置路径使用 `../src/yolo_vision/config/live_pick_roi_20260927.json`。移动相机或改变取物区域后需重新核对 ROI；固定接收区域需要现场保持可放置。

本次现场可行候选均从 candidate 0 通过，不能声称实机已经覆盖“首候选不可达再换候选”或“所有候选失败再重观测”。这些分支保留此前隔离 MoveIt 与当前单元测试证据。本次也不能证明接近 100% 的总体抓取成功率。没有夹持/释放传感反馈，程序仍如实记录 `grasp_verified=false`、`placement_verified=false`。
