> **2026-09-23 更新：** 用户已要求移除桌面模型与额外夹爪保护盒，相关建模入口和抓取加载逻辑已删除。下文涉及桌面建模的实现/测试为历史记录，不代表当前启动要求。当前行为及测试见 `docs/collision_model_removal_20260923.md`（相对工作区根目录），启动见 `New_run/README.md`。固定放置流程仍保留。

# 桌面碰撞模型实施方案（IMPLEMENT_pengzhuang）
首次：采集点云 → 生成桌面模型 → 核对位置和范围 → 保存
以后：启动 MoveIt → 加载保存的模型 → 检查加载成功 → 抓取
## 1. 项目目标

本方案只处理**桌面**，暂不把周围所有未知物体做成 OctoMap，也不接入 Pilz。
目标是：在抓取规划和执行前，使用深度相机的完整 XYZ 点云估计桌面位置，在 MoveIt Planning Scene 中加入桌面碰撞模型；规划时把 UR5e 的机械臂和夹爪碰撞几何一起纳入检查。

期望达到的行为：

1. 机械臂任何一个**已在 URDF 中定义碰撞几何的 link**进入桌面安全区域时，规划被拒绝或改为可行路径。
2. 目标上方的接近、竖直下降和竖直抬升三段路径都检查桌面碰撞。
3. 桌面高度、位置和边界来自本次相机点云，并在执行前验证场景确实存在。
4. 桌面估计失败、点云过期、TF 不可用、模型为空或规划场景未确认时，不允许真实执行。
5. 预览模式可以显示桌面并验证碰撞；真实执行仍使用现有的 `scaled_joint_trajectory_controller`。

这里的“整个机械臂和夹爪不能碰桌面”指 MoveIt 能够检查的机器人碰撞模型。它不能自动保护 URDF 没有描述的实体，也不能替代真实 UR 安全停止。现有夹爪碰撞模型的缺口见第 4 节，必须在真机验收前处理。

本文件是给后续 Codex 执行的实施文档。本次只生成文档，不修改现有项目代码、不启动机械臂、不加入桌面场景。

## 2. 当前项目实际状态

### 2.1 相机与点云接口

最近一次只读检查是在机器人和相机已经启动时完成的，结果如下：

| 项目 | 当前结果 |
|---|---|
| 完整环境点云 | `/camera/depth/points` 正在发布 |
| 点云类型 | `sensor_msgs/msg/PointCloud2` |
| 字段 | `x`、`y`、`z`，没有 RGB 字段 |
| 点云坐标系 | `camera_color_optical_frame` |
| 点云尺寸 | `1280 x 800`，有组织点云 |
| 观测接收频率 | 约 3.1 Hz（相机端仍可能以不同频率发布，需以运行时测量为准） |
| `base_link` TF | 当前可以查询到 |
| `/camera/depth/registered_points` | 检查期间没有收到消息 |

因此第一版应直接使用 `/camera/depth/points`。桌面碰撞建模只需要 XYZ，彩色信息不是必要条件；彩色点云不会自动提高碰撞几何的准确度，反而可能增加带宽和内存压力。

[tools/grasp_camera.launch.py](tools/grasp_camera.launch.py) 已启用点云、深度和彩色图，点云 QoS 使用传感器数据 QoS；[src/yolo_vision/config/config.json](src/yolo_vision/config/config.json) 也已经使用 `/camera/depth/points`。YOLO 使用同一份点云提取目标，桌面建模必须另订阅**完整原始点云**，不能使用 YOLO 输出的 `/yolo_vision/object_cloud`，否则只会得到碗的点云而没有桌面。

### 2.2 相机 TF 与机器人模型

当前自定义 URDF 文件为：

`/home/rob/ur_sim_ws/src/mobile_ur_description/urdf/ur5e_on_new_base_with_gripper_mobile.urdf.xacro`

其中已有：

- `base -> camera_link` 固定安装变换；
- 相机驱动负责 `camera_link -> camera_color/depth optical frame`；
- `base_link`、UR5e 各关节 link、`tool0`、夹爪基座和 `grasp_center` 的模型链。

点云转桌面模型必须在带时间戳的点云时刻查询：

```text
camera_color_optical_frame -> base_link
```

不能只用静态配置中的相机安装参数进行手工变换。TF 缺失、时间戳过期或变换方向错误时必须拒绝建模。

### 2.3 MoveIt 规划与现有碰撞接口

当前抓取后端为 [src/grasp_executor/grasp_executor/moveit_ros.py](src/grasp_executor/grasp_executor/moveit_ros.py)：

- `check_ready()` 检查 MoveIt 模型、规划组、关节状态、工具链和 FK/TF 一致性；
- `_scene()` 可将一个 `shape_msgs/SolidPrimitive.BOX` 加入 `/apply_planning_scene`；
- `prepare_plans()` 在每次任务开始时调用 `_scene()`；
- `_pose_plan()` 的 IK 请求设置了 `avoid_collisions=true`；
- `_linear_plan()` 的 Cartesian 请求设置了 `avoid_collisions=true`；
- `_validate_and_time()` 对轨迹采样状态调用 `/check_state_validity`；
- `_begin()` 在每一段真实执行前再次对缓存轨迹采样状态调用 `/check_state_validity`；
- 当前使用 OMPL 接近，Cartesian 下降和抬升，不需要 Pilz。

因此，**桌面盒体一旦正确加入 Planning Scene，现有规划和分段碰撞检查可以使用它**。现在的问题不是没有碰撞检查，而是桌面开关关闭且没有自动估计桌面的代码。

当前配置 [src/grasp_executor/config/config.json](src/grasp_executor/config/config.json) 中已有：

```json
"table_enabled": false,
"table_id": "grasp_table",
"table_center": {"x": 0.6, "y": 0.0, "z": -0.15},
"table_size": {"x": 0.8, "y": 0.8, "z": 0.05},
"require_collision_world_for_execution": false
```

这些中心和尺寸是禁用状态下的示例值，**不是现场标定结果**，不能直接打开使用。

### 2.4 当前机器人碰撞几何

URDF/SRDF 中已经包含标准 UR5e 各 link 的碰撞模型，MoveIt 的 `ur_manipulator` 状态有效性检查会检查这些 link 与世界物体的碰撞。

当前夹爪宏文件为：

`/home/rob/ur_sim_ws/src/Universal_Robots_ROS2_Description/urdf/gripper_macro.xacro`

实际定义情况：

- `gripper_base_link` 有一个盒体碰撞几何，默认尺寸为 `0.16 x 0.10 x 0.18 m`；
- `grasp_center` 是固定 link，但自身没有 collision geometry；
- 文件中没有可独立随夹爪开合变化的两根手指碰撞 link。

这意味着现有 MoveIt 能保护“UR5e 本体 + 一个较大的夹爪基座盒体”，但**不能严格证明两根真实手指的所有实体都已被覆盖**。如果夹爪的实际手指超出这个盒体、或盒体位置和实物不一致，桌面碰撞检测会产生漏检或误检。后续必须核对实物尺寸；建议先使用覆盖实物的保守几何，再考虑细化成掌部和两个手指。

### 2.5 当前测试覆盖范围

现有 [tools/test_moveit_grasp.py](tools/test_moveit_grasp.py) 已在隔离 ROS domain 中：

- 启动本机 MoveIt 和项目 UR5e/夹爪模型；
- 验证 OMPL 接近、Cartesian 下降/抬升；
- 验证不可达目标拒绝；
- 临时加入与 `gripper_base_link` 重叠的测试箱体，验证碰撞拒绝；
- 验证模拟 `scaled_joint_trajectory_controller` Action 链路。

但现有测试没有验证：

- 从点云估计桌面；
- 将桌面估计结果转换到 `base_link`；
- 桌面盒体的上表面安全余量；
- 机械臂每个实际碰撞 link 都不进入桌面；
- 没有桌面模型时真实执行被拒绝。

## 3. 缺少的功能

1. **桌面点云采集器**：订阅完整 `/camera/depth/points`，等待若干帧新鲜数据，而不是使用旧缓存。
2. **点云坐标转换**：按点云时间戳将点云转换到 `base_link`。
3. **桌面平面估计**：从点云中提取近似水平的桌面平面，剔除 NaN、机械臂、碗和其他离群点。
4. **桌面范围估计**：得到桌面在 `base_link` 下的 XY 范围，不能因为被碗或机械臂遮挡就把模型缩得过小。
5. **安全盒体生成**：根据平面高度和安全余量生成带厚度的桌面盒体。
6. **Planning Scene 应用和回读**：使用 `/apply_planning_scene` 加入 `grasp_table`，再用 `/get_planning_scene` 确认 ID、尺寸和场景版本。
7. **执行前强制检查**：执行模式必须要求有效桌面模型；不能继续采用当前 `require_collision_world_for_execution=false` 的宽松行为。
8. **夹爪碰撞几何核对/补齐**：保证基座盒体覆盖实际夹爪，或在 URDF 中增加手指碰撞几何。
9. **失败状态和日志**：记录点云帧数、TF、平面内点数、桌面高度、XY 边界、余量和最终场景确认结果。
10. **离线和隔离测试**：可用合成平面点云测试算法，不连接真实机械臂；使用已有 MoveIt 隔离测试验证碰撞拒绝。

## 4. 推荐实施方案

### 4.1 为什么第一阶段只做桌面盒体，不直接接 OctoMap

这个场景是固定桌面抓取，桌面位置相对机器人基本不变。只建桌面模型有以下优点：

- CPU 和内存负荷远小于持续 OctoMap；
- 不会把目标碗的点云自动加入环境地图，避免抓取目标被当成不可接触障碍；
- 现有 `_scene()` 已有盒体接口，改动范围小；
- 桌面是主要的不可碰撞边界，适合先完成验证。

第一版不处理桌面上的其他碗、杯子、支架和动态障碍。它只保证桌面盒体参与碰撞检查。后续确实需要避开杂物时，再单独评估 OctoMap，并处理目标分离和附着物体问题。

### 4.2 建议的低负荷处理流程

建议新增一个一次性桌面建模流程，而不是持续高频更新：

```text
/camera/depth/points
        ↓ 采集 5～10 帧新鲜点云
        ↓ NaN 清理、空间范围过滤、体素/步长抽样
        ↓ TF: camera_color_optical_frame -> base_link
        ↓ 水平桌面平面拟合（RANSAC 或稳健中位数/分位数）
        ↓ 得到桌面高度和可见 XY 范围
        ↓ 增加安全余量并扩展边界
        ↓ 生成 base_link 下的 BOX
        ↓ ApplyPlanningScene(grasp_table)
        ↓ GetPlanningScene 回读确认
        ↓ 允许 grasp_executor 预览/执行
```

推荐初始参数（实现时必须做成配置项，不能散落在代码中）：

| 参数 | 初始建议 | 目的 |
|---|---:|---|
| 采集帧数 | 5～10 帧 | 静止桌面提高稳定性 |
| 建模更新方式 | 每次任务前一次 | 避免持续 CPU 负荷 |
| 点云处理频率 | 只处理采集帧，不订阅回调中每帧计算 | 降低负荷 |
| 体素/步长抽样 | 2～4 cm 起步 | 桌面不需要毫米级点云 |
| 平面法向角度 | 与 `base_link +Z` 的夹角小于 10°～15° | 排除墙面、设备和斜面 |
| 平面内点最少数 | 由现场点云测量后设定，不能硬编码猜测 | 防止小片噪声被识别为桌面 |
| 桌面上表面安全余量 | 初始 5～10 mm | 防止点云噪声和模型误差造成擦碰 |
| 盒体厚度 | 现场桌板厚度不可测时先用 5～10 cm | 只要覆盖桌面以下空间即可 |
| 点云最大距离 | 只保留桌面/机械臂工作区 | 减少处理量和误检 |

平面估计得到的 `z_top` 不能直接作为盒体顶面。建议把桌面盒体顶面设置为：

```text
table_box_top_z = estimated_table_z + safety_margin
```

盒体中心和尺寸按盒体顶面、厚度、XY 边界计算。这样规划器会在物理桌面上方保留明确的空隙。安全余量过大可能使接近/抓取目标不可达，需要通过预览和现场测量调整。

### 4.3 桌面范围的保守策略

仅从可见点云计算 XY 最小/最大值可能低估桌面，尤其是被碗、机械臂或画面边缘遮挡的位置。实现时按以下优先级：

1. 点云得到可见范围；
2. 对 XY 四边增加可配置 margin；
3. 如果桌面边缘不可见或点云只覆盖局部，使用配置的已知桌面尺寸/工作区边界覆盖，而不是假装估计完整桌面；
4. 将最终范围打印并在 RViz 检查；不接受空范围、极小范围或超出可信工作区的结果。

当前相机点云坐标系和 `base_link` TF 可用，但没有证明相机能看到桌面四条边。因此**桌面 XY 长宽仍需要现场确认**；点云主要用于高度和可见区域，不能保证自动推断整张桌面的真实边界。

### 4.4 与当前抓取后端的最小接入方式

推荐先不改变 OMPL、Cartesian、控制器和夹爪命令流程，做以下接口改造：

- 新增桌面建模模块，例如 `src/grasp_executor/grasp_executor/table_model.py`，负责点云采集、TF 转换、平面估计和盒体参数输出；
- 新增一次性入口，例如 `tools/build_table_model.py` 或 `run_table_model.sh`，在硬件准备完成且机器人静止时运行；
- 将结果保存为带版本和时间戳的 JSON，例如 `debug_output/table_model/latest.json`，包含 `base_link`、`z_top`、`center`、`size`、`safety_margin`、点数、TF 时间和点云来源；
- 扩展 [src/grasp_executor/config/config.py](src/grasp_executor/grasp_executor/config.py) 和 `config.json`，支持 `table_model_path`、模型新鲜度、余量和强制执行检查；
- 扩展 [src/grasp_executor/grasp_executor/moveit_ros.py](src/grasp_executor/grasp_executor/moveit_ros.py) 的 `_scene()`：优先读取经过验证的桌面模型，构造现有 `grasp_table` BOX；保留原有 `table_center/table_size` 作为明确手工回退，但真实执行默认禁止回退；
- 在 `_scene()` 应用后立即回读 `GetPlanningScene`，核对 `grasp_table` 存在、frame 为 `base_link`、尺寸和顶面高度在容差内；
- 在 `check_ready()` 或第一次 `prepare_plans()` 中加入执行模式强制条件：桌面模型存在且通过新鲜度、TF、尺寸、余量和 Planning Scene 回读；
- 保持 `_valid()`、`avoid_collisions=true`、每段执行前重新检查，使接近、下降、抬升都检查桌面；
- 预览模式也加载同样的桌面模型，确保看到的规划与实际执行条件一致。

桌面模型生成与执行节点可以是两种模式：

- **推荐模式：预先生成模型文件，再启动抓取节点。** 逻辑简单、容易回滚，适合固定桌面；
- **集成模式：抓取节点启动时自动采集一次点云并应用场景。** 操作更方便，但启动流程更复杂，故障排查也更困难。

第一版采用预先生成模型文件，验证稳定后再考虑自动集成到 `run_pipeline_execute.sh`。

### 4.5 机器人和夹爪必须覆盖的碰撞模型

要满足“整个机械臂包括夹爪都不能碰桌面”，不能只验证 `tool0` 或 `grasp_center`：

1. 使用当前 URDF 的所有 UR5e link 碰撞几何；
2. 核对 `gripper_base_link` 的默认盒体是否完全包住真实掌部；
3. 测量夹爪张开时两根手指的最大外廓；
4. 如果现有盒体不覆盖手指，修改 `gripper_macro.xacro` 增加保守碰撞盒，或建立掌部、左指、右指的独立 link/几何；
5. 在张开和闭合两种状态下都验证；
6. 把碰撞模型略微放大，而不是缩小到只贴合视觉网格；
7. 检查 SRDF 中是否误禁用了夹爪与桌面相关的碰撞（桌面是 world object，不应通过 SRDF 禁用）。

注意：当前 `grasp_center` 只是抓取工具参考坐标，不是有体积的碰撞 link；不能用它替代真实夹爪几何。

## 5. 需要修改/新增的文件清单

以下是后续实施时的建议清单，本次没有修改：

### 5.1 工作区内文件

| 文件 | 计划修改 |
|---|---|
| `IMPLEMENT_pengzhuang.md` | 本实施文档；本次已新增 |
| `src/grasp_executor/grasp_executor/table_model.py` | 新增：点云采集、TF 转换、平面拟合、桌面盒体估计 |
| `tools/build_table_model.py` | 新增：一次性桌面建模命令，输出 JSON 和诊断信息 |
| `run_table_model.sh` | 新增：加载 ROS 环境、检查节点/TF/机器人静止、调用建模工具 |
| `src/grasp_executor/config/table_model.json` | 新增：点云 topic、范围、抽样、平面阈值、余量和输出路径 |
| `src/grasp_executor/config/config.json` | 将桌面模型路径和执行强制检查加入配置；启用前必须使用现场结果 |
| `src/grasp_executor/grasp_executor/config.py` | 校验桌面模型路径、版本、坐标系、尺寸、余量和强制执行开关 |
| `src/grasp_executor/grasp_executor/moveit_ros.py` | 读取桌面模型，应用/回读 `grasp_table`，执行模式强制场景就绪 |
| `src/grasp_executor/grasp_executor/main.py` | 把桌面场景失败作为明确状态输出，而不是只报告普通规划失败 |
| `src/grasp_executor/launch/grasp_executor.launch.py` | 如需要，暴露桌面模型路径或场景强制参数；默认不能悄悄使用旧示例尺寸 |
| `tools/run_live_grasp.py` | 执行前增加桌面模型就绪检查，预览和执行使用同一份模型 |
| `run_pipeline_execute.sh` | 在真实抓取前调用/检查一次性桌面模型；模型失败立即退出 |
| `tools/test_table_model.py` | 新增：合成平面、倾斜平面、NaN、离群物、缺 TF、范围不足测试 |
| `tools/test_moveit_grasp.py` | 新增桌面上方成功、进入桌面失败、整条轨迹碰撞拒绝测试 |

### 5.2 外部工作区文件（必须谨慎修改）

| 文件 | 计划 |
|---|---|
| `/home/rob/ur_sim_ws/src/Universal_Robots_ROS2_Description/urdf/gripper_macro.xacro` | 仅在确认当前盒体不能覆盖真实手指时修改碰撞几何；修改后必须重新构建并验证 robot_description |
| `/home/rob/ur_sim_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/srdf/mobile_ur.srdf.xacro` | 一般不需要改；只有碰撞矩阵验证发现误禁用时才改 |
| `/home/rob/ur_sim_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/launch/ur_moveit.launch.py` | 第一阶段不改；只建桌面盒体不需要启动 OctoMap。以后接 OctoMap 时才增加 sensors 配置 |
| `tools/grasp_camera.launch.py` | 第一阶段不改，继续使用现有 `/camera/depth/points` |

不建议为了只建桌面而修改相机为彩色点云，也不建议第一阶段修改 MoveIt 安装目录或在外部工作区复制一套新的 MoveIt 配置。

## 6. 详细实施步骤

### 阶段 A：离线和只读验证

1. 检查相机点云持续发布，记录实际频率、frame、字段、NaN 比例和点云时间戳。
2. 检查 `base_link <- camera_color_optical_frame` 的 TF 在多个点云时间戳都可查询。
3. 在不启动抓取、不切换控制器的情况下采集 5～10 帧。
4. 用合成水平平面和带离群点的数据测试平面拟合；先不碰真机。
5. 把估计结果写成诊断 JSON，包含点数、平面残差、法向、边界和余量。
6. 如果估计高度在多帧之间波动超过阈值，拒绝生成模型，不取一个看似合理的平均值掩盖问题。

### 阶段 B：生成桌面模型

1. 机器人保持静止，夹爪处于不遮挡桌面的安全姿态。
2. 运行 `run_table_model.sh`，使用 `/camera/depth/points`。
3. 转换到 `base_link`，对每帧做范围过滤和抽样，再合并/投票。
4. 拟合近似水平桌面平面；桌面上碗、杯子和机械臂点应作为离群点或局部非平面点排除。
5. 计算 `z_top` 和可见 XY 边界，增加配置的 XY margin 和上表面 safety margin。
6. 生成 `grasp_table` 的 BOX 参数，保存模型文件。
7. 在 RViz 中显示桌面盒体，人工确认它覆盖真实桌面，且没有明显穿过桌面或缩小到只覆盖一角。
8. 未通过人工确认前，不允许把模型用于真实抓取。

### 阶段 C：接入抓取预览

1. 让 `grasp_executor` 在 `prepare_plans()` 前加载桌面模型。
2. 调用 `/apply_planning_scene` 添加或更新 `grasp_table`。
3. 回读 `/get_planning_scene`，确认 `grasp_table` 存在且坐标系为 `base_link`。
4. 规划接近、下降、抬升三段；所有轨迹采样状态调用 `/check_state_validity`。
5. 对一个明显高于桌面的目标验证 `planned`。
6. 对一个会让当前 UR5e/夹爪碰到桌面的合成目标验证拒绝，并记录接触 link 与 `grasp_table`。
7. 验证不改变桌面模型时，旧模型不会被重复叠加；更新时使用同一个 ID 执行替换。

### 阶段 D：真实执行前置检查

1. 执行模式必须要求桌面模型存在，且模型未超过配置的新鲜度。
2. 重新确认机器人、相机、桌面相对位置没有移动。
3. 重新检查 URDF 中夹爪碰撞几何覆盖真实张开外廓。
4. 先用预览模式确认整条路径与桌面有间隙。
5. 位置控制器和 External Control 就绪后，才允许进入真实执行。
6. 真实执行仍按已有流程：接近 → 开爪 → 下降 → 合爪 → 抬升；每个运动段开始前重新做桌面碰撞检查。
7. 第一次真机测试降低速度并保持旁路急停可用；发现模型与实物不一致立即停止，不靠调大规划容差继续执行。

## 7. 约束条件和风险

1. **模型只保护可见/估计到的桌面。** 点云看不到的桌面区域、遮挡下的桌沿和桌下物体不能被自动证明安全。
2. **TF 和外参必须准确。** 相机点云在 `base_link` 中偏移几厘米，就可能使模型边界错误；需要现场确认相机安装外参。
3. **点云平面不是桌面真值。** 深度噪声、反光、边缘和桌面纹理都会影响高度；必须加安全余量并做 RViz/实物验证。
4. **桌面范围可能被低估。** 不能只把可见点的 min/max 当成整张桌面边界；需要 XY 扩展或配置尺寸。
5. **当前夹爪模型不完整。** `grasp_center` 无碰撞体，真实手指不一定被当前 `gripper_base_link` 盒体覆盖；这项不补齐就不能宣称“整个夹爪”已保护。
6. **Collision checking 不是实时防撞。** 现有后端在规划时、轨迹采样时和每段执行前检查；执行期间遇到新障碍不能保证自动急停。
7. **离散采样不是连续数学证明。** 现有 `validation_time_step`、Cartesian 步长和轨迹采样只能降低漏检风险，不能取代低速真机验证。
8. **桌面模型不代表目标模型。** 第一阶段不把碗加入环境碰撞物；因此不会自动检查碗与桌面以外物体的碰撞，也不处理抓起后附着物体。
9. **移动底盘必须保持不动。** 桌面模型在 `base_link` 下建立，底盘移动后模型随坐标关系失效，必须重新建模或改用固定世界坐标系。
10. **不能使用当前示例中心和尺寸直接执行。** `table_center`/`table_size` 当前值未经现场标定，执行前必须由点云模型或测量结果覆盖。
11. **执行强制策略必须改变。** 当前 `require_collision_world_for_execution=false` 会允许无桌面模型执行；正式接入后真实执行必须改为强制要求桌面场景。
12. **QoS 和时钟必须匹配。** 点云订阅使用 sensor-data QoS；模型只接受新鲜系统时间戳，不接受被冻结、未来或明显过期的数据。

## 8. 测试方法

### 8.1 不连接真机的单元测试

使用合成 `PointCloud2` 或等价 XYZ 数组测试：

- 水平桌面加随机噪声，估计高度在容差内；
- 桌面有碗状/局部凸起离群物，不能把凸起当桌面高度；
- 倾斜平面、墙面、零点数、NaN、无 TF、过期 TF、时间戳异常均拒绝；
- 点数少、XY 范围过小、模型尺寸非正数均拒绝；
- 安全余量正确加到盒体上表面，而不是错误加到盒体中心；
- 多帧高度不一致超过阈值时拒绝；
- 同一个模型 ID 更新时不会产生重复世界物体。

### 8.2 隔离 MoveIt 碰撞测试

沿用 `tools/test_moveit_grasp.py` 的隔离 ROS domain 和模拟反馈，不连接真实 UR：

1. 加入桌面盒体，当前状态和目标均在桌面上方，规划成功；
2. 构造一条让 `gripper_base_link` 进入桌面盒体的目标，规划失败，错误包含 `grasp_table`；
3. 构造让前臂/腕部进入桌面的关节状态，`/check_state_validity` 返回无效；
4. 让下降轨迹中间点进入桌面，验证 `_validate_and_time()` 拒绝，而不是只检查起点和终点；
5. 移除桌面后执行模式被 `collision_world_empty` 或等价错误拒绝；
6. 预览和执行使用同一桌面模型参数；
7. 验证夹爪张开、闭合两种 URDF 碰撞几何均覆盖桌面检查范围。

### 8.3 现场只读/预览测试

建议顺序：

```bash
# 仅查看环境，不启动硬件
./run_robot_prepare.sh --check

# 采集并生成桌面模型（后续新增入口，第一版实现后使用）
./run_table_model.sh

# 只规划，不移动机械臂
./run_pipeline_test.sh --vision-config src/yolo_vision/config/table_roi.example.json
```

需要同时观察：

- 桌面盒体在 RViz 中的位置；
- `grasp_table` 是否出现在 Planning Scene；
- `/get_planning_scene` 是否包含桌面；
- 抓取状态是否为 `planned`；
- 明显低于安全桌面的目标是否被拒绝；
- `result.json` 是否记录桌面模型的版本和诊断数据。

### 8.4 首次真机测试

只有单元、隔离 MoveIt 和预览均通过后，才进行：

1. 清空桌面工作通道，仅保留测试碗；
2. 使用低速度和较大的现场观察距离；
3. 先只执行接近段，确认夹爪和前臂与桌面有余量；
4. 再执行完整抓取；
5. 任何模型偏移、桌面高度错误或夹爪外廓未覆盖都立即停止并重新建模。

## 9. 验收标准

### 必须通过

- [ ] `/camera/depth/points` 能稳定提供带时间戳 XYZ 点云。
- [ ] 点云在其时间戳下能转换到 `base_link`。
- [ ] 桌面平面估计在连续采样中稳定，诊断数据完整。
- [ ] 生成的 `grasp_table` 使用 `base_link`，上表面包含明确安全余量。
- [ ] Planning Scene 回读确认桌面模型存在且尺寸正确。
- [ ] 当前 UR5e 各 link 的状态有效性检查能识别与桌面碰撞。
- [ ] 接近、下降、抬升三段的规划和轨迹采样都使用桌面场景。
- [ ] 明显碰桌面的目标被拒绝，安全目标能完成规划预览。
- [ ] 无桌面模型、点云过期、TF 失败或场景回读失败时，真实执行被阻止。
- [ ] 模型更新不会叠加旧桌面对象，不会遗留旧尺寸。
- [ ] 夹爪碰撞几何经过实物核对，覆盖张开状态的真实外廓。

### 不能宣称已经满足的事项

在以下条件完成前，不能宣称“整个机械臂和夹爪绝对不会碰桌面”：

- 没有现场确认相机外参和桌面模型位置；
- 没有验证桌面 XY 边界没有被遮挡低估；
- 没有核对或补齐两根夹爪手指的 URDF collision geometry；
- 没有通过隔离 MoveIt 的中间轨迹碰撞拒绝测试；
- 没有完成低速真机预览和急停条件下的首次执行。

## 10. 结论

这个方案可以在现有项目中实现，且第一阶段不需要安装新 ROS 包，不需要 Pilz，不需要 OctoMap。现有代码已经具备：相机 XYZ 点云、相机到 `base_link` 的 TF、MoveIt Planning Scene 盒体接口、OMPL/Cartesian 规划和轨迹采样碰撞检查。

实际缺口是：从点云得到可信桌面盒体、强制执行前场景确认，以及补齐夹爪真实外廓的碰撞模型。最简单可靠的实施顺序是：

```text
一次性点云建桌面模型
→ RViz 检查桌面盒体和安全余量
→ 隔离 MoveIt 验证整条轨迹碰撞拒绝
→ 预览抓取
→ 低速真机执行
```

持续 OctoMap 可作为以后动态障碍方案，当前固定桌面抓取不应作为第一阶段的必需依赖。

## 11. 本次实施结果（2026-09-22）

已按本文方案完成第一版实现，当前已经建立并验证了一份可复用的桌面模型。

### 已实现的文件

- `src/grasp_executor/grasp_executor/table_model.py`：PointCloud2 XYZ 解码、时间戳点云转换、水平桌面平面估计、模型校验和模型哈希。
- `src/grasp_executor/grasp_executor/table_scene.py`：桌面 BOX、夹爪保护盒、Planning Scene 应用对象和场景回读验证。
- `tools/build_table_model.py`：一次性建模、保存、加载和只读检查；建模过程检查机器人静止、点云新鲜、TF 和多帧稳定性。
- `run_table_model.sh`：`--rebuild` 重新采集，默认加载并复用，`--check` 只读检查。
- `tools/test_table_model.py`：桌面平面、变换、过期模型测试。
- `tools/test_moveit_grasp.py`：增加保存桌面模型加载和 Planning Scene 验证。

抓取配置已启用桌面保护：

```json
"table_enabled": true,
"table_model_path": "~/.local/state/robot_grasp_ws/table_model.json",
"table_model_max_age": 0.0,
"require_collision_world_for_execution": true
```

`table_model_max_age=0` 表示固定桌面模型不按文件年龄过期；文件内容、坐标系、尺寸、旋转、安全余量和场景回读仍然严格校验。桌子或机器人底座移动后必须手动执行 `./run_table_model.sh --rebuild`。

### 本次实际建立的模型

- 输入：`/camera/depth/points`，XYZ 点云，frame 为 `camera_color_optical_frame`；
- 采集：5 帧，机器人关节保持静止；
- 桌面实测平面高度约 `-0.0669 m`；
- 保护盒上表面约 `z=-0.0451 m`，包含约 `2 cm` 总安全余量（点云/模型误差和安全边界）；
- 盒体尺寸约 `1.12 x 0.479 x 0.10 m`，按用户提供的 `1.10 x 0.45 m` 加 1 cm 平面余量；
- 在 `base_link` 下的中心约 `(0.6568, 0.1834, -0.0964) m`；
- 桌面绕 `base_link` Z 轴约 `-41.7°`；
- 保存文件：`~/.local/state/robot_grasp_ws/table_model.json`。

模型不是只检查 `tool0`：现有 UR5e link 的 MoveIt 碰撞几何继续参与状态有效性检查，并新增了附着在 `gripper_base_link` 上的保守夹爪保护盒，约 `0.22 x 0.20 x 0.23 m`，允许它与自身相关 touch links 保持重合，但不允许它或机械臂 link 与 `grasp_table` 碰撞。

当前 URDF 的 `mobile_base_link` 是固定移动底座，不属于六轴机械臂或夹爪；其粗略碰撞盒与测得桌面边缘重叠，因此场景只保留这一对固定几何的明确例外。`base_link`、六个关节连杆、`gripper_base_link` 和保护盒仍禁止与桌面接触。更新场景时会保留 MoveIt 原有 SRDF 自碰撞矩阵，不会用临时矩阵覆盖它。

### 已完成的验证

- `python3 -m pytest -q src/grasp_executor/test tools/test_table_model.py src/rim_locator/test src/yolo_vision/test`：**54 passed**；
- `python3 -B tools/test_moveit_grasp.py`：**5 项隔离 MoveIt 检查通过**，包括桌面模型加载、三段规划、夹爪保护盒、不可达目标和碰撞拒绝；无真实硬件运动；
- `./run_table_model.sh --rebuild --output-dir debug_output/table_validation/final_capture`：成功采集 5 帧并应用桌面/夹爪场景；
- `./run_table_model.sh --check`：成功回读并确认桌面模型、保护盒和 Planning Scene；
- `colcon build --symlink-install --packages-select grasp_executor yolo_vision`：成功；
- 根目录启动脚本 `bash -n`：通过。

真实 `run_pipeline_test.sh` 已完成相机、YOLO 和 rim locator；单目标 XYZ 点云现在自动按 object_id=0 处理。当前现场预览在规划阶段因 `approach_ik_failed:-31` 被拒绝，说明当前机械臂姿态下该目标的固定抓取姿态没有可用 IK，未发送运动。桌面模型建立、Planning Scene 回读和隔离 MoveIt 规划已通过。
