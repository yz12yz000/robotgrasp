# 2026-09-23 现场检测、抓取点与规划调试

本轮针对 `approach_planning_failed:99999` 及碗壁点不足开展复现。用户最终重新摆放白碗、绿碗，并授权在规划与预检通过后执行一次真实抓放。工作区原有未提交修改保留；下列仅列本轮增量。

## 已查明的问题

1. **原日志的 99999 不能单独说明原因。** 使用原目标、原 IK 处理方式复现时，肩关节目标绕行约 7.07 rad、腕关节约 6.11 rad；返回的 233 点轨迹中有 5 点出现 `forearm_link/gripper_base_link` 自碰撞，复现响应为 `INVALID_MOTION_PLAN (-2)`。这证明旧方法存在分支/绕圈问题，不声称历史每次 99999 都来自同一原因。
2. **只处理等价角仍不够。** KDL 返回的肘/腕分支可能离实际状态很远，必须有界搜索、按真实关节行程排序，并在接近分支上同时检查下降和抬升。否则会出现接近能规划、下一段笛卡尔下降失败。
3. **硬限位与 MoveIt 软限位不一致。** 旧代码接受腕关节 -6.21275 rad，但模型软下限为 -6.13319 rad。复现路径终点被限制在软限位附近，产生约 2.55 mm、0.0796 rad 末端误差。现在从 URDF 的 `safety_controller` 与硬限位交集读取范围，规划前选择有效等价角。
4. **全图缩到 512 会漏掉小碗。** 保留固定 OpenVINO 模型，增加全图及重叠 640 像素窗口推理、掩膜去重、窗口边界残缺过滤。低置信度候选须被两个不同窗口支持，或单次置信度至少 0.35。数量来自观测，不固定成两个。
5. **投影落在碗上不代表深度正确。** 当前白碗的两帧点云都有连接到主体的高位深度拖尾；旧候选条件只有下界，把约 6 cm 的异常高点也当作碗沿。两帧抓取高度相差约 3.6 cm。现在仅将锚点候选限制在稳健顶部百分位上下的高度带内，完整点云仍用于壁面拟合。离线回放两帧白碗接触点距离约 0.2 mm；这是重复性证据，不是绝对标定精度。
6. **测试结果误报。** MoveIt 会向 `/display_planned_path` 发布自动单段预览，执行器再发布完整六段。旧运行器全部计数，将已完成的两对象规划误报为超时。现在分别记录单段与完整批次；仍要求每个目标都有完整六段，缺批次明确报 `preview_batch_mismatch`。

## 最终几何策略

撤回针对粉色浅碗的 50° 壁面倾角、4 mm 可见壁高放宽，恢复 **40°、6 mm**；删除其专用实验夹具与测试。仍要求至少 20 个局部碗壁点、拟合残差不超过 2.5 mm、两个尺度法向变化不超过 12°，缺少实际壁面依据时不猜测抓取点。

保留通用的竖直接近夹碗沿设置 `max_tool_tilt_deg=0`：接触位置和水平夹合方向由有效壁面决定，工具保持竖直，适配现有竖直进退。实测壁面法向与工具倾角分别记录。此项并非按碗颜色特判，也不绕过壁面质量检查。

## 本轮修改文件

| 文件 | 作用 |
| --- | --- |
| `src/grasp_executor/grasp_executor/ik_angles.py` | 有限位的等价角选择、URDF 软限位解析 |
| `src/grasp_executor/grasp_executor/moveit_ros.py` | 多种 IK 初值、候选排序、有效状态检查、接近/下降/抬升联合验证、失败细节 |
| `src/grasp_executor/grasp_executor/config.py` | 有界 IK 搜索参数及严格校验 |
| `src/grasp_executor/config/config.json`、`place_config.json` | 显式设置 IK 搜索参数，原速度/超时保护保留 |
| `src/grasp_executor/test/test_ik_selection.py`、`test_grasp.py` | 真实绕圈/软限位、候选失败切换、取消与完整分组回归 |
| `src/yolo_vision/yolo_vision/config.py`、`segmentation.py` | 分块检测配置、掩膜恢复、去重和弱候选支持 |
| `src/yolo_vision/config/place_config.json` | 启用重叠窗口，仍检测视野全部目标 |
| `src/yolo_vision/test/test_tiling.py` | 0–3 对象、重复/相邻对象、窗口边缘、低置信度支持回归 |
| `src/rim_locator/rim_locator/config.py`、`rim_detection.py` | 工具倾角约束、上下有界的碗沿候选、主要拒绝原因统计 |
| `src/rim_locator/config/place_config.json` | 恢复原壁面门槛，启用竖直接近 |
| `src/rim_locator/test/test_supported_contacts.py`、`test/data/white_green_depth_tail_1.npz`、`white_green_depth_tail_2.npz`、`test/data/README.md` | 当前白绿碗真实深度拖尾回归与夹具说明 |
| `tools/test_live_place_pipeline.py`、`tools/test_place_runner.py` | 保存实际检测图/点云，区分自动单段预览和完整批次 |
| 本文 | 调试证据、验证结果与限制 |

运行器的调试图片使用 Pillow 读取 RGB/BGR 字节，避免用户 NumPy 2 环境中 `cv_bridge` 二进制接口冲突。

## 验证

- 最终单元回归：**336 passed, 7 skipped**，7 项需要独立 ROS 集成环境；另外显式运行 ROS 集成 **21 项全部通过**，包含这些默认跳过项。
- 隔离 MoveIt：**11 项通过**，含真实规划服务、模拟控制器六段抓放、三对象 18 段、不可达和碰撞拒绝。使用独立 ROS 域，不是实机抓取结果。
- `colcon build --symlink-install --packages-select grasp_executor rim_locator yolo_vision`：3 包构建通过。
- Python 编译、Shell 语法、`git diff --check`：通过。
- 当前白绿碗最终实机数据预览：检测 2、定位 2、两组完整六段规划通过，顺序绿碗后白碗，详见 `debug_output/planning_diagnosis_20260923/white_green_final_preview/result.json`。
- 真实执行结果在下节记录。

主要证据保存在 `debug_output/planning_diagnosis_20260923/`，其中 `invalid_path_states.json` 是自碰撞复现，`planner_calls_soft_limit_before.json` 是软限位问题，`pytest_final.log`、`ros_integration.log`、`moveit.log`、`build_final.log` 是测试日志。

## 实机执行结果

1. 首次真实执行检测 2、定位 2，按距离先处理绿碗；已完成接近、开爪、下降、闭爪并开始抬升。现场人员伸手进入操作区域后主动中止。日志记录 `lifting_after_grasp / cancelled`；动作取消未在短等待内确认，备用停止路径停用了运动控制器。随后只读验证：控制器 inactive，1505 个反馈样本最大关节速度约 `7.51e-7 rad/s`；原动作最终状态为 ABORTED，确认没有遗留活动目标。
2. 用户确认人员离开并要求继续后，使用一次性恢复脚本从实测停止位置继续抬升，**没有在半空重新张爪，也没有重新执行已认领批次**。相机确认绿碗离桌后才继续固定位置放置、释放、退回。`resume_result.json` 中 `released=true, completed=true`；现场图像确认绿碗位于桌面放置区。
3. **固定落点被前一个碗占用是当前多物体流程的实际限制。** 用户明确选择移走已放好的绿碗，继续使用原落点；实时画面确认放置区清空。随后用新观测匹配原批次尚未处理的白碗（不按 object_id 追踪，ID 会随置信度排序变化），重新验证完整六段，开始白碗抓放。

4. 白碗首次恢复尝试被设备预检拦截，没有运动：实测 `robot_program_running=false`，`robot_mode=7`、`safety_mode=1`，即安全状态正常但 External Control 已停止。用户通过示教器重新启动后，改用正式 `run_pipeline_place_execute.sh` 入口重新观察当前只剩白碗的桌面，结果目录为 `white_remaining_execution`。

5. **白碗正式执行成功。** `white_remaining_execution/result.json`：`passed=true, outcome=completed, detected_count=1, target_count=1, completed_count=1`，完整经过六段运动、开爪/闭爪/释放，最终 `success`，无 `stop_error`。保存的现场图像确认白碗被抬起并从原位置搬到放置区，机械臂随后退回。

结论：**本次白碗和绿碗均已实际抓起并放下。** 不能表述为“两碗无人干预连续抓放成功”：期间有人进入区域导致中止、External Control 停止，且放置区由人员清空。无人干预处理多个碗仍需解决固定落点占用问题；本次按用户选择保留固定位置，没有擅自增加并排放置或堆叠策略。

物理结果图：`green_lifted.png`、`green_placed.jpg`、`white_lifted.jpg`、`white_placed_final.png`。最终静止反馈与控制器状态见 `final_state.json`。

本轮恢复用的 `debug_output/planning_diagnosis_20260923/resume_interrupted.py` 与 `remaining_white.py` 是针对本次中止状态的诊断脚本，不是常规入口。原执行 journal 未删除或重置；恢复入口另有一次性认领文件，白碗使用新观测时间戳和正常批次认领。

检测/定位、可规划、控制器完成以及物体确实夹起/放下是不同验收层级。现有系统未提供独立夹持和放置确认，不能把控制器成功自动解释为物体成功搬运；本次使用现场图像人工复核，不代表系统已具备自动闭环验证。
