# rim_locator 第一版设计说明

## 1. 项目定位

`rim_locator` 是 `robot_grasp_ws` 中的第二个独立 ROS2 Python package。

第一版只负责：

> 接收 `yolo_vision` 发布的目标物体点云，通过已有 TF 转换到机器人基坐标系，再使用简单的 Height-based Geometric Filtering 提取顶部 rim 候选区域，得到一个稳定的 `rim_point`，并通过 ROS2 Topic 发布给后续抓取程序。

本模块与 YOLO、OpenVINO、相机图像处理完全解耦。

当前第一版只验证这一种 rim 定位方法，但项目名称不绑定具体物体类别。

---

## 2. 整体 Workspace 结构

`rim_locator` 与 `yolo_vision` 放在同一个 workspace：

```text
robot_grasp_ws/
├── src/
│   ├── yolo_vision/
│   │   └── ...
│   │
│   └── rim_locator/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── rim_locator
│       ├── config/
│       │   └── config.json
│       └── rim_locator/
│           ├── __init__.py
│           ├── main.py
│           └── rim_detection.py
│
├── build/
├── install/
└── log/
```

核心算法实际上只有：

```text
main.py
rim_detection.py
```

`package.xml`、`setup.py`、`setup.cfg` 和 `resource/` 只是最小 ROS2 Python package 外壳。

---

## 3. 输入与输出

### 输入 Topic

订阅：

```text
/yolo_vision/object_cloud
```

消息类型：

```text
sensor_msgs/msg/PointCloud2
```

输入点云坐标系由消息自身：

```text
header.frame_id
```

确定，当前通常为：

```text
camera_color_optical_frame
```

### 输出 Topic

发布：

```text
/rim_locator/rim_point
```

消息类型：

```text
geometry_msgs/msg/PointStamped
```

例如：

```text
header.frame_id = "base"

point.x = 0.518
point.y = -0.162
point.z = 0.824
```

该点就是当前方法输出的最终目标位置点，可由后续抓取程序作为 Grab Center 的位置输入。

---

## 4. 第一版总体流程

```text
接收：
/yolo_vision/object_cloud
        ↓
读取有效 XYZ
        ↓
根据 cloud.header.frame_id 查询已有 TF
        ↓
source frame → target_frame
        ↓
一次性将整个目标点云转换到机器人基坐标系
        ↓
Height-based Geometric Filtering
        ↓
计算 Z 的高百分位
        ↓
保留顶部约 15 mm
        ↓
得到 rim candidates
        ↓
选择靠机器人一侧的局部候选点
        ↓
XYZ 中位数
        ↓
得到 rim_point
        ↓
发布：
/rim_locator/rim_point
```

第一版只处理第一份有效的 `object_cloud`。

---

## 5. Python 文件职责

### `main.py`

负责 ROS2 和 TF：

- 初始化 node。
- 读取配置。
- 订阅 `/yolo_vision/object_cloud`。
- 读取 PointCloud2 中的 XYZ。
- 删除 NaN / Inf。
- 从消息 `header.frame_id` 获取源坐标系。
- 查询到 `target_frame` 的 TF。
- 将整个点云一次性转换到目标坐标系。
- 调用 `rim_detection.py`。
- 发布 `PointStamped`。
- 本次计算完成后不再处理新的 object cloud。

为了让后续抓取程序可靠获得一次性结果，建议发布后保持节点存活，直到外部停止。

---

### `rim_detection.py`

只负责纯几何算法，不包含 ROS 通信。

输入：

```text
N × 3 XYZ
```

并且这些点已经位于：

```text
target_frame
```

输出：

```text
rim_point = [x, y, z]
```

---

## 6. TF 处理

第一版不在 `yolo_vision` 中做机器人坐标转换，而是在 `rim_locator` 中完成。

流程：

```text
object cloud
frame = camera_color_optical_frame
        ↓
已有 TF
        ↓
target_frame
```

配置中的 `target_frame` 必须是现场实际存在的机器人基坐标系，例如：

```text
base
```

或：

```text
base_link
```

要求该坐标系的 Z 轴代表实际竖直方向，否则不能直接用 Z 做顶部高度筛选。

TF 只查询一次，然后构造 4×4 刚体变换矩阵，对整个 `N×3` 点云批量转换，不逐点调用 TF。

---

## 7. Height-based Geometric Filtering

第一版使用最简单的顶部高度带方法。

### Step 1：顶部高度估计

不直接使用：

```text
max(z)
```

而使用：

```python
z_top = np.percentile(points[:, 2], 98)
```

避免少量飞点直接决定顶部高度。

### Step 2：顶部高度带

第一版：

```text
rim_height_band = 0.015 m
```

保留：

```python
rim_candidates = points[
    points[:, 2] >= z_top - rim_height_band
]
```

即目标点云自身顶部约 15 mm 的区域。

### Step 3：选择靠机器人一侧

因为点云已经在机器人基坐标系中，可计算水平距离：

```python
d = np.sqrt(x**2 + y**2)
```

第一版从 `rim_candidates` 中保留距离机器人基座较近的一小部分点。

例如：

```text
near_side_ratio = 0.1
```

即保留最近约 10% 的候选点。

### Step 4：稳定代表点

对局部候选点：

```python
rim_point = np.median(local_points, axis=0)
```

最终得到：

```text
[x, y, z]
```

第一版只输出位置，不计算姿态。

---

## 8. 与 `yolo_vision` 的通信

数据链：

```text
yolo_vision
        ↓
/yolo_vision/object_cloud
        ↓
rim_locator
```

输入 Topic：

```text
/yolo_vision/object_cloud
sensor_msgs/msg/PointCloud2
```

建议订阅端使用与上游一致的：

```text
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL
Depth: 1
```

这样 `rim_locator` 可以获得上游保留的最后一次目标点云。

---

## 9. 与后续抓取程序的通信接口

`rim_locator` 不实现抓取程序，只负责提供最终位置。

输出 Topic：

```text
/rim_locator/rim_point
```

消息类型：

```text
geometry_msgs/msg/PointStamped
```

通信关系：

```text
rim_locator
        ↓
/rim_locator/rim_point
        ↓
后续抓取程序
        ↓
读取 frame_id + x + y + z
        ↓
作为 Grab Center 的目标位置
```

抓取姿态、approach pose、运动规划和夹爪动作仍由后续抓取程序负责。

建议输出 Topic 使用：

```text
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL
Depth: 1
```

并让 `rim_locator` 在发布完成后保持节点存活，以便稍晚启动的抓取程序仍可读取最后一次 `rim_point`。

---

## 10. 配置文件

建议：

```json
{
  "input_topic": "/yolo_vision/object_cloud",
  "output_topic": "/rim_locator/rim_point",
  "target_frame": "base",
  "top_percentile": 98.0,
  "rim_height_band": 0.015,
  "near_side_ratio": 0.1
}
```

其中：

- `target_frame`：实际机器人基坐标系。
- `top_percentile`：稳定顶部高度估计。
- `rim_height_band`：顶部高度带厚度。
- `near_side_ratio`：靠机器人一侧保留的候选点比例。

---

## 11. 失败检查

第一版至少处理：

```text
empty_cloud
```

输入点云为空。

```text
tf_unavailable
```

无法获得源坐标系到 `target_frame` 的 TF。

```text
insufficient_points
```

有效点数量太少。

```text
rim_not_found
```

顶部候选点或近侧局部点不足。

失败时不发布伪造的 `rim_point`。

---

## 12. 构建与运行

两个 package 在同一 workspace 中统一构建：

```bash
cd ~/robot_grasp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select yolo_vision rim_locator
source install/setup.bash
```

先启动 `rim_locator`：

```bash
ros2 run rim_locator rim_locator
```

再运行 `yolo_vision`：

```bash
ros2 run yolo_vision yolo_vision
```

由于输出 Topic 采用一次性结果设计，也可以在实际集成时统一启动两个节点。

---

## 13. 第一版实现原则

1. 与 `yolo_vision` 共用一个 `robot_grasp_ws`，但保持独立 ROS2 package。
2. `rim_locator` 不依赖 YOLO、OpenVINO 或相机图像。
3. 输入只认 `object_cloud`。
4. 核心算法只使用 Python + NumPy。
5. 不使用 C++ / PCL C++。
6. TF 从输入消息的 `header.frame_id` 转换到配置的 `target_frame`。
7. 第一版只使用 Height-based Geometric Filtering。
8. 最终 `rim_point` 使用局部多点中位数，不使用单个点。
9. 通过 ROS2 Topic 将 `rim_point` 提供给后续抓取程序。
10. 不包含抓取规划、运动控制和夹爪逻辑。
11. 第一版优先验证：
    - TF 是否正确；
    - rim candidates 是否位于目标顶部区域；
    - `rim_point` 是否稳定。
