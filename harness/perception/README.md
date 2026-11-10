# harness/perception/ —— 头部 RGBD 相机

## 这里干什么

从 G2 头部相机取同步的彩色图、深度图和头部位姿,把每个深度像素换算成 `base_link` 下的 3D 点。颜色识别本身在 [`harness/backends/color_object.py`](../backends/color_object.py),这里只负责“拿到图和 3D 点”。

代码**复制自 `agi@g2_robot` 的 `task_board_pose`**(提交 `ecdfb5d`),没有改动。

## 这里的文件

| 文件 | 作用 |
|---|---|
| `rgbd_source.py` | `RosSource`:订阅 `/gdk/camera/head_color`、`/gdk/camera/head_depth`、`/gdk/tf`,按时间戳配对彩色和深度 |
| `scene3d.py` | `Rgbd.points_base`:深度图 → `base_link` 下的 3D 点 |
| `calib.py` | 头部相机内参、外参(2026-11-03 从真机读出) |
| `sensor_node.py` | 用时才订阅、用完就退订的传感器节点,避免一直接收大流量图像 |

## 和哪里联动

```text
G2 机器人上的 GDK
   /gdk/camera/head_color(JPEG)
   /gdk/camera/head_depth(16 位,毫米)
   /gdk/tf(base_link → head_link3)
        ▼
rgbd_source.RosSource ──→ (彩色, 深度, 头部位姿)
        ▼
scene3d.Rgbd.points_base ──→ 每个像素的 base_link 坐标
        ▼
backends/color_object.find_objects ──→ 物体坐标
```

使用者:[`harness/backends/ros.py`](../backends/ros.py) 定位物体,[`scripts/hsv_pick.py`](../../scripts/hsv_pick.py) 实时取色。

这些话题由机器人本身发布,不需要 `hardware.launch.py`,但需要 `source ~/agi@g2_robot/env_g2.sh` 加载 GDK 的 DDS 配置。

## 示例

```python
from harness.perception.rgbd_source import RosSource
from harness.perception.scene3d import Rgbd

source = RosSource("my_node")              # 自己 init rclpy、自己 spin
frame = source.next(None)                  # 没有新帧时返回 None,要循环等
if frame is not None:
    bgr, depth_mm, T_base_head, _ = frame
    points, camera_origin = Rgbd().points_base(depth_mm / 1000.0, T_base_head)
    print(points[200, 320])                # 画面中心像素在 base_link 下的坐标
source.close()
```

## 注意

G2 的彩色图和深度图已经由 GDK 逐像素对齐,同一个 (u, v) 的颜色和深度就是同一个点,不需要再配准。

标定值换机器人或重新标定后要更新 `calib.py`。`agi@g2_robot` 里有读取工具:`ros2 run genie_robot_agibot_g2_perception dump_head_camera_calib`。
