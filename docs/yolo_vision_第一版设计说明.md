# yolo_vision 第一版设计说明

## 1. 项目定位

`yolo_vision` 是 `robot_grasp_ws` 中的一个独立 ROS2 Python package。

第一版只负责：

> 获取一组同步的 RGB 图像和注册点云，使用 YOLO26n-Seg + OpenVINO 得到目标 mask，再利用 mask 与 organized PointCloud2 的像素级对应关系提取目标物体三维点云，并发布给下游 `rim_locator`。

本模块到 `object_cloud` 为止，不负责 TF 到机器人基坐标系后的几何分析，也不负责抓取点计算、抓取规划或机器人控制。

---

## 2. 整体 Workspace 结构

第一版把 `yolo_vision` 和 `rim_locator` 放在同一个 ROS2 workspace 中，统一构建和运行：

```text
robot_grasp_ws/
├── src/
│   ├── yolo_vision/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   │   └── yolo_vision
│   │   ├── config/
│   │   │   └── config.json
│   │   ├── models/
│   │   │   └── yolo26n-seg_openvino_model/
│   │   └── yolo_vision/
│   │       ├── __init__.py
│   │       ├── main.py
│   │       ├── segmentation.py
│   │       └── pointcloud.py
│   │
│   └── rim_locator/
│       └── ...
│
├── build/        # colcon 自动生成
├── install/      # colcon 自动生成
└── log/          # colcon 自动生成
```

这里：

```text
src/yolo_vision/              → ROS2 package
src/yolo_vision/yolo_vision/  → Python module
```

核心算法全部使用 Python。

---

## 3. 已确认的输入条件

相机为 Orbbec Gemini2L。

输入 Topic：

```text
/camera/color/image_raw
/camera/depth_registered/points
```

当前 registered point cloud 已开启：

```text
ordered_pc = true
```

并确认：

```text
height = 720
width  = 1280
```

因此 RGB、Depth Registered PointCloud2 可以按同一 `1280 × 720` 像素网格对应。

点云原始坐标系为：

```text
camera_color_optical_frame
```

---

## 4. 第一版总体流程

```text
启动 yolo_vision
        ↓
加载 YOLO26n-Seg OpenVINO 模型
        ↓
等待第一组同步的 RGB + PointCloud2
        ↓
YOLO 推理一次
        ↓
选择目标类别
        ↓
得到目标 mask
        ↓
将 mask 恢复到 RGB 原始尺寸 1280×800
        ↓
检查 RGB / mask / PointCloud2 尺寸一致
        ↓
mask 与 organized PointCloud2 逐像素对应
        ↓
只保留 mask 内对应的 XYZ
        ↓
删除 NaN / Inf
        ↓
得到 object point cloud
        ↓
发布：
/yolo_vision/object_cloud
```

第一版采用 **one-shot perception**：

> 每次任务只执行一次 YOLO 推理，不持续循环识别。

---

## 5. Python 文件职责

### `main.py`

负责 ROS2 流程组织：

- 初始化 node。
- 读取配置。
- 加载 YOLO 模型一次。
- 订阅 RGB 和 PointCloud2。
- 使用 `ApproximateTimeSynchronizer` 获取一组同步数据。
- 调用 `segmentation.py`。
- 调用 `pointcloud.py`。
- 发布 `object_cloud`。
- 本次推理完成后不再重复处理新的相机帧。

为了让下游节点可靠获得一次性结果，第一版建议节点在发布结果后保持存活，直到外部停止。

---

### `segmentation.py`

只负责 YOLO 分割：

```text
RGB
 ↓
YOLO26n-Seg OpenVINO
 ↓
class + confidence + mask
```

当前测试目标通过配置指定，例如：

```text
target_class = bowl
```

项目本身不绑定具体物体类别。

如果画面中出现多个同类目标，第一版选择置信度最高的一个。

#### Mask 尺寸

虽然 YOLO 推理可设置：

```text
imgsz = 512
```

但用于点云索引前必须确保：

```text
mask.shape = (720, 1280)
```

如果尺寸不同，使用最近邻插值：

```text
INTER_NEAREST
```

恢复到 RGB 原始尺寸后，再转换成布尔 mask。

---

### `pointcloud.py`

只负责：

```text
2D mask + organized PointCloud2
                ↓
         object point cloud
```

核心对应关系：

```text
mask[v,u] == True
        ↓
保留 cloud[v,u] 的 [X,Y,Z]
```

程序逻辑上将 PointCloud2 的 XYZ 保持为：

```text
xyz.shape = (720, 1280, 3)
```

然后：

```python
object_points = xyz[mask]
```

结果为：

```text
N × 3
```

随后删除：

```text
NaN
Inf
```

最终得到目标物体点云。

注意：发布后的 `object_cloud` 不需要继续保持 720×1280 的 organized 结构，可以发布为只包含目标点的普通 PointCloud2。

---

## 6. 为什么不再做投影矩阵

当前使用：

```text
/camera/depth_registered/points
```

并且已经：

```text
ordered_pc = true
```

所以 RGB mask 与 registered point cloud 可以直接按 `(u,v)` 对应。

第一版不需要重新计算：

```text
相机内参
RGB-Depth 外参
3D → 2D 投影矩阵
```

这也是本方案相比“独立相机 + LiDAR 投影”方案的主要简化点。

---

## 7. ROS2 通信接口

### 输入

```text
/camera/color/image_raw
sensor_msgs/msg/Image

/camera/depth_registered/points
sensor_msgs/msg/PointCloud2
```

相机输入订阅使用与传感器 Topic 兼容的 Sensor Data QoS。

### 输出

```text
/yolo_vision/object_cloud
```

消息类型：

```text
sensor_msgs/msg/PointCloud2
```

`header.frame_id` 保持输入 registered cloud 的坐标系，例如：

```text
camera_color_optical_frame
```

### 与 `rim_locator` 的连接

```text
yolo_vision
    ↓
/yolo_vision/object_cloud
    ↓
rim_locator
```

建议 `object_cloud` 使用：

```text
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL
Depth: 1
```

并让 `yolo_vision` 在发布完成后保持节点存活。这样即使 `rim_locator` 稍晚开始订阅，也能获得最后一次发布的目标点云。

---

## 8. 配置文件

建议：

```json
{
  "rgb_topic": "/camera/color/image_raw",
  "pointcloud_topic": "/camera/depth_registered/points",
  "output_topic": "/yolo_vision/object_cloud",
  "target_class": "bowl",
  "confidence": 0.25,
  "imgsz": 512,
  "device": "intel:cpu"
}
```

模型和配置文件路径不要写死绝对路径。

程序使用：

```text
ament_index_python
```

获取 package share 路径，再定位：

```text
config/config.json
models/yolo26n-seg_openvino_model/
```

---

## 9. 第一版失败检查

至少检查：

```text
no_target
```

没有检测到目标类别。

```text
dimension_mismatch
```

RGB、mask、PointCloud2 尺寸不一致。

```text
insufficient_pointcloud
```

mask 内有效 XYZ 点太少。

失败时不发布伪造的目标点云。

---

## 10. 构建与运行

在统一 workspace 中构建：

```bash
cd ~/robot_grasp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select yolo_vision rim_locator
source install/setup.bash
```

相机保持独立启动，并确保：

```bash
ros2 launch orbbec_camera gemini2L.launch.py ordered_pc:=true
```

确认：

```text
/camera/depth_registered/points
height = 720
width  = 1280
```

然后运行：

```bash
ros2 run yolo_vision yolo_vision
```

---

## 11. 第一版实现原则

1. 与 `rim_locator` 共用一个 `robot_grasp_ws`，但保持独立 ROS2 package。
2. 全部核心逻辑使用 Python。
3. 使用现有 YOLO26n-Seg OpenVINO 模型。
4. 不使用 CUDA。
5. 不使用 C++ / PCL C++。
6. YOLO 只运行一次。
7. RGB 和 PointCloud2 必须先同步。
8. mask 必须恢复到 RGB / organized point cloud 的真实尺寸。
9. 直接使用像素级对应提取目标 XYZ。
10. `yolo_vision` 的职责到 `object_cloud` 为止。
11. 第一版优先验证：
    - YOLO mask 是否正确；
    - RGB / mask / cloud 尺寸是否一致；
    - 提取出的 object cloud 是否只包含目标物体。
