# 碗检测与抓取流程修复实施方案

日期：2026-09-23。工作区：`/home/rob/robot_grasp_ws`。

本文是供后续 Codex 执行的实施文档。本次仅生成本文，不修改程序、配置、测试和启动脚本，不启动硬件。下文中的新增函数、参数和测试均为待实现内容，不代表项目已经具备这些功能。

## 1. 项目目标与范围

### 1.1 本次应实施的内容

按 Chat 中最后一份面向用户的 1～6 项问题列表编号执行，避免与检查报告内部的排序混淆：

| Chat 编号 | 问题 | 本次处理 |
| --- | --- | --- |
| 1 | ROI 配置导致多碗流程意外切回单实例 | 修复配置继承，增加简便的单碗／多碗选择 |
| 2 | 后续目标过期、整批不重新检测 | **不修改** |
| 3 | 没有夹持成功与释放成功反馈 | **不修改** |
| 4 | 高位杂点污染碗沿锚点 | 增加按实例执行的空间连通性过滤与回归测试 |
| 5 | 多碗模式一次漏检就终止 | 改成总等待期限内重试新帧 |
| 6 | PoseArray 路径忽略 `grasp_offset` | 在批次执行入口统一应用一次偏移 |

注意：`docs/bowl_pipeline_review_20260923.md` 的第 2 节问题是漏检，第 3 节问题是过期；本方案以本表的问题名称和 Chat 编号为准，不能因为编号不同把用户排除的问题重新加入。

用户要求的简化切换：保留一个 `multi_instance` 配置字段，同一套抓取放置流程通过该字段决定处理一个或多个碗；提供不改文件的快捷参数。不要复制两套抓取、规划或放置实现。

### 1.2 明确排除

- 不增加逐碗目标有效期校验，不增加抓完一碗后的重检测、跟踪、重排序或自动恢复。继续使用现有冻结批次。
- 不修改 `ShellGripper`、夹持反馈、释放反馈以及成功判定的现有语义。
- 不恢复桌面模型，不增加碗、容器或携带物的碰撞模型，不修改场景清理策略。
- 不修改固定放置位姿、下降 0.20 m、停留 1 秒、释放、退回的流程；不增加堆叠或占位管理。
- 不修改壁面法向决定工具姿态、沿基座 Z 竖直进退的运动方式。
- 不顺带修复 `bash/` 复制入口的路径问题、废弃参数或其他历史问题。
- 不修改 YOLO 模型、置信度、标定、TF、控制器选择、速度与运动超时。

## 2. 已读取的代码及现有接口

### 2.1 检测与配置

- `src/yolo_vision/yolo_vision/config.py`：`VisionConfig.multi_instance` 默认 `False`；`load_config(path)` 直接执行 `VisionConfig(**json.load(...))`。目前没有配置继承、合并或命令行单多碗快捷开关。
- `src/yolo_vision/config/place_config.json`：显式 `multi_instance=true`，无 ROI，`input_timeout=90`。
- `src/yolo_vision/config/table_roi.example.json`：有 `[320,350,740,550]` ROI，未设置 `multi_instance`，`input_timeout=120`。直接加载后是单实例。
- `src/yolo_vision/yolo_vision/segmentation.py`：`predict()` 取置信度最高的一只碗；`predict_all()` 保留多个实例。ROI 裁剪后已将 mask 恢复到原图尺寸。
- `src/yolo_vision/yolo_vision/main.py`：按 `multi_instance` 选择上述方法。多实例点云带 UINT32 `object_id`，单实例输出 XYZ。成功后节点停止处理新帧。
- 当前 `no_target` 仅在单实例分支恢复 `ready`，并重新设置 `wait_started`；多实例转为 `failed`。持续单实例漏检可能不断延长等待期限。

### 2.2 定位

- `src/rim_locator/rim_locator/pointcloud.py`：XYZ 点云缺少 `object_id` 时自动视为实例 0，因此单碗与多碗已能复用同一个定位节点。
- `src/rim_locator/rim_locator/main.py`：按采集时刻做 TF，按实例调用 `locate_grasp_pose()`，逐实例捕获 `ValueError`，拒绝单个实例后可继续处理其他实例；按 `hypot(x,y)` 排序，发布 `PoseArray` 和最近点的 `PointStamped`。
- `src/rim_locator/rim_locator/rim_detection.py`：目前仅移除非有限值，无空间聚类或离群过滤。候选点是 `z >= percentile(z,98)-band`，最高杂点仍会保留；锚点随后取近侧局部高度上半部分的中位数。
- 同一个实例的全体点还用于中心估计和壁面 PCA，因此只过滤最终锚点或候选集不足以彻底隔离离群簇。
- 几何模块目前只依赖 NumPy；没有已存在的 Open3D、PCL、DBSCAN 或 RANSAC 实现可以直接调用。

### 2.3 抓取执行

- `src/grasp_executor/grasp_executor/main.py`：`grasp_pose_from_ros()` 复制 PoseArray 的位置和姿态，再交给 `run_batch_task()`，当前不加偏移。
- `src/grasp_executor/grasp_executor/grasp.py`：旧单点路径的 `build_grasp_pose()` 已按基座坐标分量加 `grasp_offset`；批次路径直接使用输入 Pose。`build_approach_pose()` 在抓取位上沿基座 Z 增加接近高度。
- `run_batch_task()` 已支持一只或多只碗、位姿合法性检查、排序、六段规划、批次认领、异常停止及完成计数。
- `on_poses` 回调会将批次实际使用的抓取、接近和放置位姿交给节点发布，因此把偏移放在批次入口即可使预览与执行使用同一个结果。
- `claim_batch()` 以采集时间和位姿集合生成持久化摘要；不能因新增偏移破坏现有防重复执行语义。

### 2.4 启动与测试

- `tools/test_live_place_pipeline.py` 是现有抓取放置运行器，实际也负责真实执行。它加载三份 `place_config.json`，允许 `--vision-config` 替换视觉配置，通过 `config_path:=...` 传给 YOLO 启动脚本。
- `run_pipeline_place_test.sh` 和 `run_pipeline_place_execute.sh` 均透传参数；真实执行入口额外传 `--execute`。
- `New_run/run_pipeline_place_*.sh` 只是根目录入口的转发，不必复制业务逻辑。
- 旧 `run_pipeline_test.sh`、`run_pipeline_execute.sh` 使用 PointStamped 的三段抓取流程，不含放置；继续保留其用途与参数语义。
- `src/yolo_vision/test/` 已有 ROI 和多实例分割测试；`src/rim_locator/test/test_multi_pose.py` 有合成碗壁、20° 斜壁、退化几何和实例隔离测试。
- `src/grasp_executor/test/test_place.py` 已有假机械臂、双目标顺序、失败停止、重复批次和停留时间测试；`tools/test_place_runner.py` 已 mock ROS、子进程和时间，可扩展验证配置传递。
- `run_pipeline_place_offline_test.sh` 已有构建、单元回归及隔离 MoveIt 模拟集成入口。此前 Chat 检查中现有回归为 97 passed、7 skipped；这只是修复前基线，本次文档生成不宣称重新运行了测试。

## 3. 可实施性与总体方案

结论：可以在现有模块边界内实施。消息类型、Topic、节点名称、MoveIt 后端和状态机主流程都不需要更换。

| 修改 | 实现位置 | 复用现有能力 |
| --- | --- | --- |
| 单／多碗选择、配置继承 | 视觉配置工具 + 放置运行器 | `multi_instance`、现有 `config_path` 参数 |
| 漏检重试 | `VisionNode` | 同步回调、`ready/processing/success/failed`、steady timer |
| 去除分离的杂点簇 | `rim_detection.py` | 按实例调用、纯 NumPy、异常实例隔离 |
| 抓取偏移 | `run_batch_task()` | `Pose`、workspace 检查、`on_poses`、现有六段规划 |

只改变目标数量的配置，不改变是否执行真实运动。单碗的选择规则继续是最高置信度；不改为最近碗。多碗仍按当前距离定义排序。

## 4. 单碗／多碗切换设计

### 4.1 统一配置与快捷参数

保留 `src/yolo_vision/config/place_config.json` 作为抓取放置入口的默认视觉配置；持久化开关仍只有：

```json
"multi_instance": true
```

`false` 表示单碗，`true` 表示多碗。JSON 中修改已有字段即可，不增加第二个 `mode` 配置字段，不创建互相容易漂移的 single/multi 两份完整配置。

在放置运行器中增加互斥参数 `--single`、`--multi`，只覆盖本次运行的同一个布尔值，不回写用户 JSON。未传参数时为 `None`，不能错误地把“未选择”当作 `false`。

实施后预期命令如下，目前尚未实现：

```bash
cd /home/rob/robot_grasp_ws
./run_pipeline_place_test.sh --single
./run_pipeline_place_test.sh --multi
```

真实执行入口对应使用 `run_pipeline_place_execute.sh --single` 或 `--multi`。这些命令属于后续使用说明，本次及离线测试均不执行真实入口。

两种模式都走 PoseArray 和完整抓取放置流程：单碗是数组长度 1，多碗是长度 N。不能因 `--single` 转去旧的不含放置流程。

### 4.2 配置覆盖规则

只对放置运行器引入继承规则；旧独立 YOLO 节点的 `load_config(path)` 保持原来完整加载语义。

优先级从低到高：

1. `VisionConfig` 字段默认值。
2. `src/yolo_vision/config/place_config.json` 的配置。
3. `--vision-config` JSON 中**实际写出的字段**。
4. 本次 `--single` 或 `--multi` 对 `multi_instance` 的覆盖。

必须先合并原始 JSON 字典，再创建 `VisionConfig`。不能先把自定义配置加载成带默认值的 dataclass 再合并，否则缺失的 `multi_instance` 会提前变成 `False`，重新引入问题。

| 输入 | 应用的模式 |
| --- | --- |
| 不带配置或模式参数 | 默认放置配置中的模式，出厂仍为多碗 |
| `--vision-config table_roi.example.json` | 继承默认多碗，应用该文件的 ROI 和其他显式字段 |
| 自定义 JSON 显式 `multi_instance=false` | 单碗，尊重用户明确配置 |
| 自定义 JSON 为 false，再加 `--multi` | 本次多碗，原文件不变 |
| 默认配置为 true，再加 `--single` | 本次单碗 |
| 同时传 `--single --multi` | 参数错误，运行器不启动节点或切换控制器 |

默认值、覆盖值及最后配置都必须校验，不能让非法字符串 `"false"`、数字或未知字段悄悄通过。CLI 覆盖也不能用来隐藏输入 JSON 的格式／类型错误。

### 4.3 落地方式

1. 在 `yolo_vision/config.py` 新增独立的纯函数，例如 `load_effective_config(base_path, override_path=None, multi_instance=None)`；原 `load_config()` 接口不变。函数只处理配置，不导入 ROS、OpenVINO 或模型。
2. 运行器在 `rclpy.init()`、启动项目节点和调用位置控制脚本前完成配置解析与验证。脚本入口原有只读环境 probe 不属于配置函数。
3. 将最终配置用 `dataclasses.asdict()` 保存到本次输出目录的 `effective_vision_config.json`，以绝对路径交给 `run_yolo_vision.sh config_path:=...`。保留文件供复查，不写入安装目录，不原地改源码配置。
4. 自定义配置路径按调用者当前目录解析；兼容从根目录及 `New_run/` 传入相对路径。`model_path` 保持原有相对包 share 的解释方式，不能因快照位于 debug_output 而改变模型查找路径。
5. 控制台打印最终“单碗／多碗、ROI、配置来源”；`result.json` 增加最终 `multi_instance`、来源路径和配置快照路径。保留现有 `mode=plan_only/execute` 的含义，不覆盖它。
6. 更新根目录两份放置入口的帮助文本与使用文档；`New_run/` 转发结构无需改动。旧入口保持原样。

`table_roi.example.json` 保持作为旧单碗入口也能使用的 ROI 示例，不直接改成全局多碗。问题通过放置入口正确继承默认模式解决。

## 5. 详细实施步骤

### 步骤 0：记录基线

读取 `git status --short` 并保存本次开始前的差异清单。当前已有大量未提交修改，实施者不得 reset、覆盖或恢复成仓库旧版本。以当前工作区为基线。

先确认本方案列出的函数仍存在；若用户期间改了接口，按实际情况调整局部实现并在交付记录说明。不要照抄历史 IMPLEMENT 文档里的已移除模块。

### 步骤 1：实现配置合并及快捷切换

按第 4 节实现并先完成纯配置测试、mock 运行器测试。保证配置文件可以单独设置单碗，不依赖新增 CLI；CLI 只是同一配置的便捷入口。

### 步骤 2：修复首次成功前的漏检重试

修改 `VisionNode`，保留一次成功发布后结束本次感知的行为：

1. 从进入初始 `ready` 开始记录固定的检测等待起点或 deadline；漏检后不再重置这个起点。
2. `on_pair()` 在进入新一轮推理前检查该期限。定时器继续处理没有输入时的超时，避免回调持续进来导致 timer 没机会终止。
3. 单碗和多碗遇到 `no_target:*`，在期限内都恢复 `ready` 并记录 `retry=true`，等待后续同步帧。到期则报告 `failed / input_timeout`。
4. 明确区分可重试的无目标／有效深度不足与不可重试的数据错误。单实例 `extract_object_points()` 的 `insufficient_pointcloud` 可以作为无有效目标重试；维度不匹配、非法 mask 类型、模型加载错误、错误 frame、已有同步／时间戳校验失败仍终止。
5. 多实例可以保留当前“单个深度不足实例被拒绝、其余继续”的行为；若所有实例仅因深度不足被拒绝，则重试。不要把格式错误也包装成 `no_target:all_masks_rejected`。
6. 推理返回后再检查总期限，过期结果不再发布为成功；同步推理本身仍不可由 timer 强制抢占。这里保证的是检查点上的期限，不宣称可以杀死卡住的模型调用。
7. 成功输出仍保留原采集时间；不能重写时间戳来使重试结果变新。成功后不再继续采集下一批，不涉及用户排除的逐碗重检测。

需要同步考虑下游等待期限：当前 rim 的 `input_timeout=60`，视觉是 90／120 秒，而且 rim 先于 YOLO 启动。只延长 YOLO 重试会让消费者先超时。

在**放置运行器生成的本次配置快照**中，为 rim 与 grasp 的 `input_timeout` 取 `max(原值, args.timeout + 启动等待预算35秒)`，分别传入 `effective_rim_config.json` 与 `effective_grasp_config.json`。只延长等待首次输入的期限，使运行器仍以其现有总体截止时间结束任务。不要改永久配置，不修改 `max_target_age`、未来时间容差、执行中的任何检查或 motion timeout。三个消费者均复用现有 `config_path`，不需要新增 Topic。

本项只针对完整放置入口协调等待预算；单独启动节点的使用者仍负责各自 JSON 中的输入等待时间，文档应说明这一点。

### 步骤 3：在碗沿定位前去掉空间分离的杂点簇

本次目标是修复已经复现的“少量高位、近侧、与碗主体分离的杂点使整碗被拒绝”。不承诺解决所有分割污染，也不增加第二套边沿搜索算法。

采用纯 NumPy 加 Python 容器实现的体素连通组件过滤，避免新增未声明的点云库依赖：

1. 对每个实例的有限 XYZ 点计算 `floor(points / voxel_size)`，构建占用体素到原始点索引／计数的映射。
2. 在占用体素上以 26 邻域连接，用队列遍历或并查集获得连通组件；按原始有效点数选最大的主组件，不按体素数量选。
3. 若主组件点数达到 `min_points`，且其占全部有效点的比例达到阈值，返回主组件对应的**原始点**。不以体素中心或均值替换原点，以免引入毫米级抓取坐标漂移。
4. 如果没有占优势的组件，拒绝该实例，返回明确的 `pose_not_found:ambiguous_components` 等原因，不能随意选近处的小簇。
5. 将同一份过滤结果用于高度分位数、锚点、实例中心和壁面 PCA。不能只过滤 `locate_rim()`，而在 `locate_grasp_pose()` 中又对未过滤的点求中心与法向。
6. 保留 `locate_rim(points, config)`、`locate_grasp_pose(points, config)` 的外部调用方式和结果字段。可增加内部 helper，确保一次定位不会重复进行两次过滤。
7. 不在合并所有实例的点云上聚类。保持实例 ID、采集时间和坐标系不变。`base_cloud` 可以继续表示原始变换点云，`rim_candidates/local_points` 表示实际参与定位的过滤后子集，并在调试说明中区分。

建议新增 `RimConfig` 参数，均为待实现的初始值：

| 参数 | 初始值 | 含义 |
| --- | --- | --- |
| `filter_disconnected_points` | `true` | 开启实例内空间连通过滤，可关闭以对比旧算法 |
| `component_voxel_size` | `0.008` m | 体素边长，须为有限正数 |
| `min_component_ratio` | `0.60` | 主组件最低占比，要求 `0.5 < value <= 1` |

两份 rim JSON 显式写入相同的过滤配置，配置类也提供默认值，使旧 JSON 仍能读取。8 mm、60% 是实施和离线验收起点，不声称已经标定过现场所有深度密度。

该算法不依赖水平碗沿，不通过直接删除最高 2% 点来修复问题；因此需用倾斜碗沿回归，防止把真实高侧边缘删掉。体素邻接是连通性近似，不是严格欧氏半径证明；近到落入同一连通簇的污染仍属于本次算法的边界。

禁止构建 N×N 距离矩阵。大点云使用占用体素索引，避免内存平方增长；在稀疏、远距离点云中若碗被切碎，应报告拒绝原因并通过配置调整，不能默默退回使用污染点。

### 步骤 4：统一批次偏移语义

沿用旧单点实现的定义：`grasp_offset` 单位为米，三个分量沿 `config.base_frame`（当前为 `base_link`）的 X/Y/Z 加到定位锚点上；不是工具坐标偏移。姿态不变。

1. 在 ROS-free 的 `grasp.py` 中新增小型纯 helper，例如 `apply_grasp_offset(pose, config)`：校验输入，使用 `vector()` 读取偏移，创建新的不可变 Pose 并检查偏移后的 workspace。
2. 在 `run_batch_task()` 的 preflight 内为所有原始 Pose 加一次偏移，再由补偿后的 Pose 创建 approach、发布调试消息和规划。不要在 ROS 转换函数、rim 节点或后端再次加偏移。
3. 保留原始输入列表，不原地修改调用者对象。对补偿后的抓取点仍按 `hypot(x,y)` 排序；零偏移时结果及顺序与当前一致。
4. 非零偏移可能改变距离顺序；这是按实际抓取位置排序的结果，不改变距离的定义。原始点和补偿后点可在日志中分别标注，避免把定位坐标与执行坐标混淆。
5. 保持旧单点 `build_grasp_pose()` 的一次偏移效果。若复用 helper，必须证明不会双加；也可以保留旧实现，新增测试对齐两条路径的语义。
6. 不修改时间戳验证时机，不增加逐碗复检。首次校验、首次执行前校验和冻结批次结构按现状保留。
7. 批次认领继续基于原始观测的 Pose：保存按原始距离稳定排序的原始列表，交给 `claim_batch()`，实际规划使用补偿列表。这样只改变偏移参数也不会让同一已认领观测被当成新任务。摘要格式、journal 目录和原有文件均不迁移、不清空。
8. `on_poses` 接收到的必须是最终补偿后的 grasp/approach；place ready/drop 不受抓取偏移影响。

正式配置中的偏移当前为零，本次不得擅自填入经验偏移值；修复的是参数生效路径，不是现场标定。

### 步骤 5：文档、构建与回归

完成单元回归后再运行现有隔离 MoveIt 测试。新测试只使用假接口或隔离模拟环境，不调用真实 execute 入口。

更新 `New_run/README.md`、启动说明及相关适配说明：统一推荐现有 place 入口加 `--single/--multi`；同时说明只改 JSON 也有效，单碗默认按置信度选取，旧单碗入口不含放置。

历史检查报告保留为修复前记录，不将原有“复现缺陷”的脚本继续当成修复后的通过标准。需要把属于本次范围的复现条件转为“修复后应成功／应拒绝”的 pytest 断言；用户排除的时间戳问题不作为本次失败门槛。

## 6. 文件修改清单

以下均指后续实施时的预期变更，本次只有 `IMPLEMENT_debug.md` 被新建。

| 文件 | 操作 | 内容 |
| --- | --- | --- |
| `src/yolo_vision/yolo_vision/config.py` | 修改 | 新增纯配置合并函数，保留原加载接口 |
| `src/yolo_vision/yolo_vision/main.py` | 修改 | 两种模式的期限内漏检重试、错误分类 |
| `tools/test_live_place_pipeline.py` | 修改 | 模式参数、配置解析与快照、下游首次输入等待预算、最终模式记录 |
| `run_pipeline_place_test.sh` | 修改 | 帮助文本说明新参数，保留参数透传及预览用途 |
| `run_pipeline_place_execute.sh` | 修改 | 帮助文本说明新参数，保留真实执行用途 |
| `src/rim_locator/rim_locator/config.py` | 修改 | 连通过滤配置和校验 |
| `src/rim_locator/rim_locator/rim_detection.py` | 修改 | 纯几何过滤、统一使用过滤后的点 |
| `src/rim_locator/config/config.json` | 修改 | 显式过滤参数，保留其他值 |
| `src/rim_locator/config/place_config.json` | 修改 | 显式过滤参数，保留其他值 |
| `src/grasp_executor/grasp_executor/grasp.py` | 修改 | 批次入口一次偏移、原始认领与实际规划位姿分离 |
| `src/yolo_vision/test/test_config_modes.py` | 新增 | 配置继承、布尔值、非法配置、覆盖优先级 |
| `src/yolo_vision/test/test_retry.py` | 新增 | 假时钟与假节点的漏检／成功／超时／错误分类 |
| `src/rim_locator/test/test_outlier_filter.py` | 新增 | 高位杂点、空间组件、倾斜边缘和边界条件 |
| `src/rim_locator/test/test_multi_pose.py` | 按需扩展 | 现有几何和实例隔离回归，不能放松原断言掩盖退化 |
| `src/grasp_executor/test/test_place.py` | 扩展 | 非零偏移、越界、姿态不变、只加一次、认领兼容 |
| `tools/test_place_runner.py` | 扩展 | 参数透传、有效配置快照、单／多碗完整流程、只读源配置 |
| `tools/test_moveit_grasp.py` | 按需扩展 | 使用现有隔离机制验证小幅非零偏移和六段流程，不改变隔离机制 |
| `New_run/README.md` | 修改 | 新的简便命令及旧入口区别 |
| `docs/0921/startup_scripts.md` | 修改 | 同步启动和配置覆盖说明 |
| `docs/hardware_adapter.md` | 按需修改 | 补充偏移属于基座坐标、PoseArray 输入锚点的约定 |

默认不修改：YOLO 两份单碗／多碗配置的现有模式值、`table_roi.example.json` 的默认单独加载语义、ROS launch 文件、消息和 Topic、`main.py` 的 ROS Pose 转换、`moveit_ros.py`、`interfaces.py`、`scene_cleanup.py`、旧启动运行器、外部夹爪工作区。

当前 `setup.py` 已发现包内 Python 模块并安装 `config/*.json`，本方案不需要新增运行依赖或手工增加数据文件安装规则。

## 7. 测试方法

### 7.1 配置与运行器

- 基础配置为 true，覆盖文件只有 ROI：最终仍为 true，ROI 精确保持。
- 自定义显式 false：最终单碗；CLI `--multi` 可仅覆盖本次模式，反向同理。
- 覆盖文件缺失字段与显式 false 必须区分。非法类型、未知字段、坏 JSON、缺失文件和互斥参数错误均有清晰报错。
- 模式参数不改变 `plan_only/execute`；假子进程断言 `--single` 本身不会调用位置控制或夹爪。
- 从根目录和 `New_run/` 解析相对配置路径，传入子进程的均为有效绝对路径。
- 校验生成的三个快照：视觉合并值正确，rim/grasp 仅调整首次输入等待参数，`max_target_age`、放置位姿、运动参数未变。
- 单碗产生一次六段预览；两个碗产生两次六段预览，保留原来“预览数量必须等于目标数量”的检查。
- 比较运行前后的源 JSON 内容，确保运行器和 CLI 均不写回用户配置。

### 7.2 感知重试

采用 fake node/bridge/segmenter/publisher 和虚拟 monotonic 时钟，禁止加载真实模型或连接相机：

- 单／多模式均测试：前两帧 `no_target`，第三帧有效，最终只发布一次成功输出。
- 连续漏检越过 `input_timeout` 必须失败，起点不被每次漏检刷新；无输入也能超时。
- 推理结束才超时，不得发布过期的本次检测结果；但不要求强制终止尚未返回的模型调用。
- 格式错误直接失败；深度不足可重试；部分实例深度不足不妨碍其他有效实例。
- `success` 后新增帧不会再触发推理，验证仍然是首次成功后冻结的一次任务。
- mock 运行器测试迟到但仍在视觉期限内的成功，验证消费者等待快照不会在旧的 60 秒点提前结束。

### 7.3 几何过滤

- 重建此前 9,600 个正常圆柱碗壁点与 80 个近侧高位杂点的案例。开启过滤后应恢复有效姿态，抓取位置相对干净输入误差不超过 2 mm，法向夹角不超过 2°。
- 对未混入杂点且全部连通的点云，返回原始点集合，锚点和姿态应与关闭过滤时一致（允许浮点误差）。
- 保留 `test_multi_pose.py` 中竖直壁、20° 斜壁、倾斜碗沿和 rim-only 拒绝等已有测试；增加真实高侧边缘，不能为去噪把它整段截掉。
- 非均匀采样、随机删除少量点、少量噪声下测试连通性；碎裂严重或双组件无明显主次时应明确拒绝。
- 两只碗各自带独立 object_id，其中一个异常时，另一个结果、ID 对应关系及采集时间仍正确。
- 大点云检查没有 N×N 数组；记录点数、体素数和耗时，不用脱离运行硬件的固定毫秒阈值宣称实时性能。
- 若有历史真实 `base_cloud.npy`，可离线对比开启／关闭过滤的结果；这些数据不一定具备人工真值，不得直接当作真实成功率证据。

### 7.4 抓取偏移

- 原始点 `(0.3,0.1,0.2)`，偏移 `(0.01,0,0.02)`，批次抓取点必须为 `(0.31,0.1,0.22)`，原始点不变。
- 设置非默认姿态，验证四元数不变；approach 使用补偿后位置加原接近高度；place ready/drop 完全不变。
- 一只与两只碗均只加一次偏移，`on_poses`、规划和假运动实际目标一致。
- 零偏移与当前六段流程一致；补偿后抓取点或 approach 越界时，在调用规划和任何运动前拒绝。
- 同一原始观测首次认领后，改变偏移再提交，仍判定重复；零偏移原有 journal 摘要行为保持兼容。
- 保留原有故障停止、不自动释放、不继续下一碗、时间戳入口校验和重复执行拒绝测试。不要新增后续目标过期必拒绝的要求。

### 7.5 命令与分阶段验证

后续实施时，先在已配置 ROS 环境中运行单元测试，例如：

```bash
cd /home/rob/robot_grasp_ws
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src/grasp_executor:$PWD/src/rim_locator:$PWD/src/yolo_vision:${PYTHONPATH:-}"
unset ROBOT_STARTUP_ROS_TEST
/usr/bin/python3 -B -m pytest -q \
  src/grasp_executor/test src/rim_locator/test src/yolo_vision/test \
  tools/test_place_runner.py tools/test_pipeline_runner.py \
  tools/test_robot_control.py tools/test_joint_stationarity.py
```

配置与重试测试需要 ROS 消息／cv_bridge 导入，但不能初始化真实 ROS 图。`PYTHONNOUSERSITE=1` 用于避免此前发现的用户 NumPy 2 与系统 ROS 扩展混用；正式 YOLO 启动脚本的虚拟环境处理保持原样。

单元测试通过后，可使用项目现有 `run_pipeline_place_offline_test.sh` 构建三个包并运行隔离 MoveIt 集成；先确认脚本仍保留随机空闲 ROS_DOMAIN_ID、localhost 和模拟控制接口。不要用 `run_robot_prepare.sh` 或真实 execute 脚本代替离线测试。

完成后执行修改文件的 Python 编译检查、两份根目录启动脚本的 `bash -n`、`git diff --check`。测试失败要定位原因，不得通过删除现有测试、降低几何断言或跳过关键新测试来通过验收。

## 8. 约束条件

1. 本次生成文档，后续按文档实施；不能把文档中的示例命令视为本轮启动硬件授权。
2. 模式开关只控制检测实例数量，两种模式都复用完整放置流程；旧三段入口继续存在。
3. 保留现有消息、坐标系、采集时间、工具轴约定和对外函数接口。新增配置需有兼容默认值和严格类型校验。
4. 不改用户排除的时间戳复检、夹持反馈、环境建模、放置策略、倾斜工具进退方向。
5. 不增加网络服务、GUI、数据库、跨帧跟踪或新模型依赖；不重构与修复无关的代码。
6. 不将 ROI 示例硬改为多实例而破坏旧单碗入口；通过新放置入口的继承规则解决。
7. 不把空间过滤说成完整的抗所有异常算法；不对缺乏可靠组件的点云生成猜测姿态。
8. 偏移保持零默认值、基座坐标含义，并确保只作用于抓取目标，不作用于固定放置位置。
9. 所有已有未提交变更必须保留。只有与本方案直接相关的代码、测试和文档可以修改。

## 9. 验收标准

- [ ] `IMPLEMENT_debug.md` 指定的用户排除项没有被引入实施范围。
- [ ] 只改一个 `multi_instance` 配置即可切换；也可通过同一套 place 入口的 `--single/--multi` 临时选择。
- [ ] 默认多碗配置、ROI 继承、显式单碗配置和 CLI 覆盖四种情况均有测试，最终模式能在日志及结果文件中确认。
- [ ] 模式切换不写回配置，不改变真实执行开关；单碗仍完成抓取、放置、释放和退回。
- [ ] 多碗短暂漏检后能使用新帧成功；连续漏检在总期限内结束，消费者不会因原先较短的等待配置抢先失败。
- [ ] 高位分离杂点复现案例修复，干净输入不漂移，倾斜边沿、壁面姿态和实例隔离回归保持通过。
- [ ] 非零 `grasp_offset` 在单个和多个 Pose 的批次中恰好应用一次，坐标系语义明确，越界被拒绝。
- [ ] 认领仍绑定原始观测，同一批次不能仅因调整偏移再次执行；不改变后续目标时效策略。
- [ ] 现有单元回归及新增测试通过；默认跳过的测试必须写明原因，不能当作已执行通过。
- [ ] 隔离 MoveIt 测试验证旧三段、现有六段和两目标顺序；测试不连接硬件，不把模拟通过表述为实机抓取成功。
- [ ] 构建、Python 编译、Shell 语法及差异检查通过；最终交付列出修改文件、运行命令、实际测试结果和仍保留的设计限制。

## 10. 建议执行顺序

先做配置合并与单多切换，再做有界感知重试；随后实现并验证实例内空间过滤；最后处理批次偏移与认领兼容，运行整套离线回归和隔离 MoveIt。每一步都以实际结果推进，不提前声称完整流程已验收。
