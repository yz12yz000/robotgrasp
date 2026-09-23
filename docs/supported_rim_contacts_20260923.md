# 两个碗可检测但无抓取点：现场诊断与修改

本次按用户最新要求修改检测及抓取点生成。保留自动观察视野全部目标、0 个不动作、N 个按近到远处理的流程；不恢复历史文档中的单／多物体开关。

## 现场原因

已直接读取当前相机的同步 RGB、对齐深度、CameraInfo 和采集时刻 TF，保存了三次独立观测。并非只根据截图猜测深度。绿色碗和白色碗的分割掩码及实例数量正确，因此没有降低检测置信度或更换模型。

旧定位只对一个最高且近侧的锚点，在固定 25 mm 半径、锚点下方 6–30 mm 范围拟合一次平面。俯视时近侧外壁可见深度少；固定大邻域又容易包含圆弧过渡和碗底。任一局部失败都会拒绝整只碗。

第一份新采样中，旧绿色碗锚点有 29 个壁面候选点，PCA 法向倾角约 83.0°；白色碗有 185 个候选点，倾角约 50.9°。这复现了“检测两只、定位零只”，但本帧绿色碗的具体原因变成 `wall_normal_out_of_range`，与之前采样的 `insufficient_wall_points` 不同，体现了局部深度和选点的变化。

## 修改方法

- 保留实例内连通组件过滤、采集时间、坐标变换和每个实例的独立处理。
- 首先检查原近侧接触区域；无可靠壁面时，在观测到的顶部碗沿点带中以 8 mm 间距搜索其他局部区域，按到基座的距离优先，最多追加 96 个区域。
- 每个接触点检查 12.5、18.75、25 mm 三种壁面邻域。至少两个尺度有额外点支持，法向差不超过 12°，才接受较小尺度的局部法向，避免把大范围碗底曲率混进接触法向。
- 没有放宽 40° 最大倾角。额外要求壁面不少于 20 点、5–95% 高度跨度至少 6 mm、平面残差不超过 2.5 mm、锚点到壁面平面不超过 6 mm。仍拒绝纯碗沿、线状点云、无壁面和不稳定法向，不退回猜测竖直姿态。
- 检测调试图显示不同实例颜色、ID、置信度及有效深度数量，缺深度像素标红；重叠像素与输出点云使用相同的高置信度优先规则。即使全部实例缺深度，也保留检测图和逐实例原因。
- 定位失败也发布带 `object_id` 的 `base_cloud`；新增 `wall_points` 调试 Topic 和逐实例拟合质量、搜索次数、拒绝原因统计。

新配置均有兼容默认值和类型／范围校验，写入两份 rim JSON。算法使用 NumPy 与有上限的局部搜索，没有 N×N 点距离矩阵和新模型依赖。

## 修改文件

| 文件 | 修改 |
|---|---|
| `src/rim_locator/rim_locator/rim_detection.py` | 多接触区域、多尺度壁面验证，质量诊断 |
| `src/rim_locator/rim_locator/config.py` | 搜索及质量约束的配置校验 |
| `src/rim_locator/config/config.json`、`place_config.json` | 明确新参数，保留 40° 限制 |
| `src/rim_locator/rim_locator/main.py` | 失败点云、实际壁面调试输出和逐实例诊断 |
| `src/rim_locator/rim_locator/pointcloud.py` | 调试点云保留实例 ID |
| `src/yolo_vision/yolo_vision/main.py` | 有效深度统计、失败情况下的检测诊断 |
| `src/yolo_vision/yolo_vision/diagnostics.py` | 带实例 ID 和深度信息的图像叠加 |
| `src/rim_locator/test/test_supported_contacts.py`、`test_outlier_filter.py` | 真实记录、缺壁、退化、尺度一致性、异常配置回归 |
| `src/rim_locator/test/data/` | 两份真实双碗 XYZ 记录与来源说明 |
| `src/yolo_vision/test/test_detection_diagnostics.py`、`test_retry.py` | 实例对应、缺深度和失败诊断测试 |
| `tools/capture_grasp_observation.py` | 只读同步采集 RGB、深度、TF 和实例数据 |
| `tools/analyze_grasp_observation.py` | 不连接 ROS 的定位重放及抓取点投影图 |
| 本文 | 现场证据、验证和未解决问题 |

已有未提交修改以本次开始时的工作区为基线保留，基线差异在 `debug_output/rim_geometry_20260923/baseline.diff`。本次没有修改机械臂执行器及其运动参数。

## 验证结果

三次独立现场采样均为 **检测 2、定位 2**：

| 观测 | 绿色碗壁面点数／法向倾角 | 白色碗壁面点数／法向倾角 |
|---|---|---|
| frame01 | 30／11.73° | 33／31.89° |
| frame02 | 42／13.82° | 43／36.80° |
| frame03 | 50／8.82° | 23／37.37° |

六次定位的局部平面残差为约 0.46–0.74 mm；这是拟合残差，并非独立测量的定位精度。第三份采样未加入测试夹具，用作额外现场重放。抓取点投影图在 `debug_output/rim_geometry_20260923/frame01/contacts_detail.png`，完整元数据、图像和点云保留在各 frame 目录。

- 三个 ROS 包 `colcon build --symlink-install --packages-select yolo_vision rim_locator grasp_executor` 构建通过。
- 全套 pytest：**291 passed, 7 skipped**，日志 `debug_output/rim_geometry_20260923/pytest.log`。包括 0/1/2/3 目标自动数量、真实 ROS 消息序列化、定位、假接口抓放流程以及新几何回归。
- 跳过的 7 项为默认关闭的隔离 ROS 测试；另以 `ROBOT_STARTUP_ROS_TEST=1 /usr/bin/python3 -B tools/test_robot_control.py` 执行，**21 tests OK**，包含上述 7 项。
- 独立 ROS domain 207、localhost、模拟反馈/控制器的 MoveIt 集成：**11 项通过**，包括三目标共 18 段模拟运动、逐个释放和退回、非零偏移、不可达目标和碰撞拒绝。结果保存为 `debug_output/rim_geometry_20260923/moveit_result.json`。
- Python 编译、入口 Shell 语法、`git diff --check` 通过。

第一次真实完整入口只规划测试：

```bash
./run_pipeline_place_test.sh --timeout 360 \
  --output-dir debug_output/rim_geometry_20260923/live_preview
```

检测 2 个、定位 2 个、无实例拒绝，顺序 `[0, 1]`，源时间戳原样保留。绿色碗、白色碗壁面倾角分别约 11.59°、36.26°。MoveIt 在第一个碗的接近规划返回 `approach_planning_failed:99999`，全流程结果为失败，不能记作实机抓放通过。同一观测中的第一个接近目标随后仅调用规划服务复核，返回成功及 283 个轨迹点（`planner_probe.json`），因此失败存在规划重复性问题，不能直接断言点不可达。

随后使用新鲜观测再次运行完整入口，结果在 `debug_output/rim_geometry_20260923/live_preview_repeat/result.json`：仍是检测 2、定位 2、无拒绝、顺序 `[0, 1]`；两碗壁面倾角为约 8.64°、39.08°。第一个接近轨迹经过现有速度／加速度约束处理后需要 **244.03 秒**，超过 **90 秒** `motion_timeout`，因此报 `motion_timeout_too_short_for_trajectory:approach:duration=244.03s,scale=7.70`。这次也未完成完整抓放规划，没有为了让测试通过而增加超时、放宽运动约束或跳过后续检查。

**仍未解决：** 当前真实起始姿态到检测抓取姿态的规划重复性和过长接近轨迹；两只真实碗的完整六段轨迹尚未通过，实物抓起及放下尚未验证。以上两次完整入口失败都保留原始结果，三次采样重放成功和隔离仿真通过不能替代现场全流程验收。

## 使用及边界

继续用原 `./run_pipeline_place_test.sh` 检查当前场景；默认只规划。RViz 可观察 `/yolo_vision/overlay`、`/rim_locator/base_cloud`、`/rim_locator/wall_points` 和 `/rim_locator/grasp_poses`。

离线重放已保存的采样（先配置本项目 Python 包路径）：

```bash
PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src/rim_locator" /usr/bin/python3 \
  tools/analyze_grasp_observation.py \
  debug_output/rim_geometry_20260923/frame01/observation.npz
```

当前算法仍依赖分割、已标定的 RGB/深度与 TF、顶部可见点带及足够壁面深度。它不补造缺失深度，不保证所有材料、遮挡或倾斜状态都能定位；搜索成功也不证明机械臂路径可达或夹持稳定。现场未执行真实机械臂或夹爪动作，尚无真实抓起和放下的验证。完整现场规划结果及重复性需与感知通过分开记录。
