# 按视野中检测数量逐个抓放：修改与测试

本次按用户最新要求取代单／多物体选择：默认观察全视野的全部 `bowl` 实例，0 个不动作，1 个处理 1 个，N 个按抓取位置到基座的 XY 距离从近到远逐个完成抓取和放置。
不再通过 `--single`、`--multi` 或 `multi_instance` 选择数量。旧 JSON 的布尔型 `multi_instance` 兼容读取后忽略，非法类型仍报错，源 JSON 不被回写。

## 最终行为

- 先观察和定位，收到有效目标后才启动抓取节点。真实执行入口也只在此时调用位置控制脚本。
- 一组有效同步数据确定本批目标；0 个检测结果返回 `no_targets`，正常退出，不启动抓取节点、不切换控制器、不发夹爪命令。
- 默认全视野；若用户显式传 ROI 配置，仅观察该区域。目标类别仍为 `bowl`，没有扩展为任意物体。
- 一批只观察一次，每个目标抓放完成后继续下一个。放置位置、下降距离、夹爪反馈语义和冻结批次保持现有设计。
- 有目标但深度无效、模型错误、相机数据缺失均报错，不伪装成 0 个目标。
- 部分实例深度／定位被拒绝时，只能处理有合法位姿的实例；结果列出 `detected_count`、`target_count`、`completed_count`、逐实例拒绝原因。部分完成不报告整批通过。

直接使用：

```bash
./run_pipeline_place_test.sh
# 现场确认规划后才使用真实执行入口：
./run_pipeline_place_execute.sh
```

## 相机超时处理

原错误中的“两组”指连续两组传感帧，与两个物体无关。现场并行订阅 RGB 和 16 MB `/camera/depth/points` 时，收到 RGB 而未收到点云；单独订阅点云可收到消息。
改用原始 CDR 头读取、更换接收端 DDS 都未单独解决此现场问题，因此不能把原因断言为 Python 解码或某个 DDS 实现。

随后确认 `/camera/depth/image_raw` 与 RGB 能同时接收，1280×800、相同 `camera_color_optical_frame`，最近一次有效检测时间差为 0.002061824 秒。
深度 CameraInfo 和 RGB 的 K 矩阵相同，深度 CameraInfo 的 D 为零。现在 place 默认同步 RGB、对齐深度图和 CameraInfo，按原像素位置还原 XYZ，16UC1 从毫米转为米；32FC1 按米读取。
尺寸、坐标系、内参、畸变、编码、数据长度和采集时间仍严格校验。不放宽同步阈值，不改写采集时间，不将缺失数据视为空场景。
旧 PointCloud2 输入可通过 `depth_image_topic: null` 保留；这只是相机数据来源配置，不是目标数量模式。

place 入口删除固定参数的前置 `wait-camera` 检查，由视觉节点校验实际使用的观测。硬件准备的独立 `wait-camera` 探针也改为 RGB／对齐深度图，仍要求两组递增且可配对的帧，超时显示收到的帧数、尺寸、frame_id 和时间差。
原始消息头只用于配对，选中帧后才完整反序列化。

曾仅重新加载相机组件试验 `frame_aggregate_mode=full_frame`，该配置没有改善输出，已撤回文件改动并将相机全部参数恢复到试验前值，`frame_aggregate_mode` 恢复为 `ANY`。
试验前后参数差异仅为该字段；相机恢复记录保存在 [camera_restore_result.json](../debug_output/auto_count_20260923/camera_restore_result.json)。没有修改外部驱动源码、系统网络参数或机械臂控制器配置，没有发送真实机械臂／夹爪命令。

## 本轮修改文件

| 文件 | 内容 |
| --- | --- |
| `src/yolo_vision/yolo_vision/config.py` | 删除目标数量模式字段；兼容旧字段；增加对齐深度输入配置 |
| `src/yolo_vision/config/place_config.json` | 默认使用对齐深度图和 CameraInfo，移除数量模式 |
| `src/yolo_vision/yolo_vision/segmentation.py` | 统一返回所有实例；删除取最高置信度单实例的重复路径；空结果返回空列表 |
| `src/yolo_vision/yolo_vision/main.py` | 自动数量、0 目标结束、三路同步、逐实例计数与相机诊断 |
| `src/yolo_vision/yolo_vision/pointcloud.py` | 深度投影、单位转换、相机内参与数据格式校验 |
| `src/yolo_vision/yolo_vision/sensor_input.py` | 新增轻量 CDR 消息头读取与选中帧延迟解码 |
| `src/rim_locator/rim_locator/main.py` | 所有实例定位失败时保留各自拒绝原因 |
| `src/grasp_executor/grasp_executor/grasp.py`、`main.py` | 空批次正常结束且不访问运动接口 |
| `tools/test_live_place_pipeline.py` | 先感知后执行；取消模式参数；数量一致性、部分完成、0 目标与错误结果区分 |
| `tools/robot_control.py` | 相机探针改读对齐深度，补充原始采集时间、真正两组配对和坐标系检查及诊断 |
| `run_pipeline_place_test.sh`、`run_pipeline_place_execute.sh` | 统一入口和帮助，取消固定参数的前置相机检查 |
| `src/yolo_vision/test/test_config_modes.py`、`test_retry.py`、`test_multi_instance.py`、`test_roi.py` | 按新数量行为更新旧回归；ROI 仍保留全部实例 |
| `src/yolo_vision/test/test_sensor_input.py`、`test_depth_image.py`、`test_observation_pipeline.py` | 新增序列化、深度投影和 0／1／2／3 个目标贯通回归 |
| `src/grasp_executor/test/test_place.py`、`src/rim_locator/test/test_multi_pose.py` | 空批次无动作、三个目标排序执行、全部拒绝原因 |
| `tools/test_place_runner.py`、`tools/test_robot_control.py` | 先感知后启动、0 目标、相对路径、错误、部分完成和相机检查 |
| `tools/test_moveit_grasp.py` | 扩展为三个目标、18 段模拟执行和非零偏移检查 |
| `New_run/README.md`、`docs/0921/startup_scripts.md` | 新用法、计数语义与相机输入说明 |
| `docs/automatic_count_20260923.md` | 本报告 |

## 实际测试结果

| 测试 | 结果 |
| --- | --- |
| 三个 ROS 包最终构建 | 全部成功，2.49 秒 |
| 最终完整单元回归 | **254 passed、7 skipped**，6.13 秒 |
| 单独启用隔离 ROS 编排 | **21 tests 全部通过**，包含上行默认跳过的 7 项 |
| 隔离 MoveIt | **11 项全部通过**；旧三段、单目标六段、三个目标的连续规划及 18 段模拟执行，非零偏移恰好一次 |
| 0／1／2／3 目标贯通 | 真实消息序列化／解码、实际定位算法、批次执行代码与假机械臂，预览／执行两种用途均通过 |
| 深度投影 | 毫米／米、大小端、行填充、像素对应、零深度、非法内参／畸变／时间戳等回归通过 |
| 静态检查 | 57 个 Python 文件编译、三份 Shell 脚本语法、帮助文本与 git diff --check 通过 |
| 真实相机准备探针 | 对齐深度路径检查退出码 0 |
| 真实相机 + 真实模型 + 只规划入口 | **未通过整个流程**：识别到 2 个碗、同步通过，随后两个实例均被定位拒绝，未启动执行器 |

实际命令与日志：

```bash
PYTHONNOUSERSITE=1 ./run_pipeline_place_offline_test.sh
# 最终修改后另外完成三包构建与下列完整回归：
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src/grasp_executor:$PWD/src/rim_locator:$PWD/src/yolo_vision:${PYTHONPATH:-}"
unset ROBOT_STARTUP_ROS_TEST
/usr/bin/python3 -B -m pytest -q -rs \
  src/grasp_executor/test src/rim_locator/test src/yolo_vision/test \
  tools/test_place_runner.py tools/test_pipeline_runner.py \
  tools/test_robot_control.py tools/test_joint_stationarity.py
# 加载现有 MoveIt/UR 环境后单独执行隔离 ROS 编排：
ROBOT_STARTUP_ROS_TEST=1 /usr/bin/python3 -B tools/test_robot_control.py
```

[最终构建](../debug_output/auto_count_20260923/build_final.log)、[最终回归](../debug_output/auto_count_20260923/regression_final.log)、
[隔离 ROS](../debug_output/auto_count_20260923/startup_verified.log)、[MoveIt 结果](../debug_output/auto_count_20260923/moveit/result.json)、
[真实相机只规划结果](../debug_output/auto_count_20260923/live_depth_details/result.json)。
真实预览用临时 JSON 将视觉等待设为 20 秒，仅用于本轮诊断，正式配置仍为 90 秒；没有设置 ROI。

## 尚未通过的现场项目

真实视野的两个碗对应拒绝原因为：

1. `object_id=0`：`pose_not_found:insufficient_wall_points`，当前锚点附近没有足够碗壁点用于姿态估计。
2. `object_id=1`：`pose_not_found:wall_normal_out_of_range`，拟合法向超出允许范围。

程序已如实记录检测数 2、有效抓取位姿数 0，没有猜测位姿或放宽几何阈值来强制执行。因此可以确认数量编排、输入超时处理和模拟抓放通过，**不能宣称当前现场的这两个碗已具备可执行抓取位姿，也不能宣称实机抓放成功**。
试图同时采集 SDK 点云与深度图做同一采集时刻的 XYZ 对比时，未获得可配对样本，未将其算作验证通过；深度投影已有数值单元测试，但仍需现场定位质量验收。
原有固定放置点、无夹持／释放传感反馈、不重检测后续目标和缺少物体碰撞模型等限制仍在。
