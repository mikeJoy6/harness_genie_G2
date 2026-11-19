# sim/：G2 的 MuJoCo 仿真

仿真程序冒充 GDK：收发和真机一样的 `/gdk/*` 话题。所以真机那一整套都原样运行，不用改：

- ros2_control，用的是真机插件 `GenieG2ArmHardware`；
- 5 个轨迹控制器；
- MoveIt；
- 头部深度点云 → octomap；
- harness 本身。

和真机只有两处不同：

1. **`find_objects` 用仿真的真值分割，代替 SAM 3。**
   - 仿真每一帧都会额外发一张分割图，时间戳和彩色图、深度图相同，每个像素标明属于哪个物体。
   - harness 拿这张图当掩码，后面“掩码 + 深度 → 坐标”的代码和真机完全一样。
   - 仿真的深度没有误差，所以算出来的就是**完美坐标**，含义和真机一样：物体的可见表面中心。
2. **末端不连真设备。**
   - 夹爪：不连 GDK，型号按 URDF 认（`configs/sim.yaml` 的 `sim_models: from_urdf`），开合指令照样走 `/gdk/joint_control`；
   - 吸盘：`suction_control` 换成 `sim_left` / `sim_right` 驱动。仿真里吸盘开着时，一碰到能动的物体就会把它粘住，关掉就松开。

### 两个进程

MuJoCo 4.x 要求 Python 3.11 以上，而 ROS Humble 的 rclpy 只有 3.10 版，装不进同一个 Python。所以仿真拆成两个进程，用本机的 Unix 套接字通信（[link.py](link.py)，只传 JSON 和原始字节）：

| 进程 | Python | 做什么 |
|---|---|---|
| [physics.py](physics.py) | 3.12（`~/mujoco_sim/.venv`，MuJoCo 4.0.1） | 1 kHz 物理、渲染相机、模拟吸盘、MuJoCo 窗口；不碰 ROS |
| [ros_bridge.py](ros_bridge.py) | 系统 3.10（rclpy） | 冒充 GDK 收发 `/gdk/*` 话题，提供 `/sim/*` 服务 |

物理进程重启后，ROS 桥会自动重连。

## 安装（只做一次，装在项目外）

物理进程用最新的 MuJoCo，装在一个独立的 Python 3.12 环境里。它不需要 ROS，所以和系统的 Python 3.10 互不影响：

```bash
mkdir -p ~/mujoco_sim && cd ~/mujoco_sim
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python "mujoco==4.0.1" numpy opencv-python-headless pyyaml
```

ROS 桥、面板用系统的 `python3`，不用额外装东西。2026-11-19 之前用的是 MuJoCo 3.17（Python 3.10，一个进程），旧环境留在 `~/mujoco_sim/.venv-py310-mujoco317`，不需要可以删掉。

## 启动

```bash
# 终端 1:仿真 + ros2_control + MoveIt + octomap(+ RViz)
source ~/robot_harness/sim/sim_env.sh
ros2 launch ~/robot_harness/sim/launch/sim.launch.py            # RViz + MuJoCo 窗口 + 末端控制面板,两手 omnipicker
ros2 launch ~/robot_harness/sim/launch/sim.launch.py use_rviz:=false viewer:=false panel:=false   # 三个窗口都不开
ros2 launch ~/robot_harness/sim/launch/sim.launch.py left_ee:=suction right_ee:=suction   # 换末端(和真机 hardware.launch.py 同样的参数)

# 终端 2:harness(不用开 SAM 3 服务)
source ~/robot_harness/sim/sim_env.sh
cd ~/robot_harness && python3 scripts/run_task.py --sim
```

`sim_env.sh` 和真机的 `env_g2.sh` 故意不同：

- ROS 域号设为 **77**，并开 **`ROS_LOCALHOST_ONLY=1`**，话题只在本机传；
- **不加载** GDK 的 DDS 配置。

所以仿真里发出的指令到不了真机。另外两道保护：`run_task.py --sim` 和启动文件发现域号是 4，都会直接拒绝运行。

## 控制末端

启动后默认会弹出一个**末端控制面板**（`panel:=false` 不开）：

- 每只手一列，按钮按装的末端生成：夹爪是“张开 / 闭合”，吸盘是“吸 / 放”（下面显示吸住了什么）；没装末端的手没有按钮；
- 底下有一个“物体复位”按钮：物体放回初始位置，机器人不动。

面板走的接口和真机完全一样：夹爪用 harness 的 `GdkGripper` 发 GDK 伺服，吸盘用 `suction_control`。所以在面板里能用，harness 里也就能用。

写脚本或者不想开窗口时，用命令行：

```bash
source ~/robot_harness/sim/sim_env.sh
python3 ~/robot_harness/sim/ee.py                  # 两只手装了什么、吸盘状态
python3 ~/robot_harness/sim/ee.py right open       # 夹爪:open / close
python3 ~/robot_harness/sim/ee.py left suck        # 吸盘:suck / release
python3 ~/robot_harness/sim/ee.py reset            # 物体复位
python3 ~/robot_harness/sim/panel.py               # 单独开面板
```

仿真吸盘的规则：开着时，吸盘一碰到能动的物体，就按当时的相对位置把它粘住（MuJoCo 焊接约束）；关掉就松开。所以要先开吸再把吸盘贴上去，或者先贴上去再开吸，两种顺序都可以。吸住的物体重量会计入手腕六维力。

## 文件

| 文件 | 作用 |
|---|---|
| [scene.py](scene.py) | 读官方 MJCF，改造成仿真要用的模型，再按场景文件摆上桌子和物体。改了什么见文件头 |
| [physics.py](physics.py) | 物理进程（MuJoCo 4）：1 kHz 物理、相机、吸盘、窗口；发给桥的消息见文件头 |
| [ros_bridge.py](ros_bridge.py) | ROS 桥（rclpy）：冒充 GDK 的话题，`/sim/*` 话题和服务（话题表见文件头） |
| [link.py](link.py) | 两个进程之间的通信：一段 JSON 头加若干段字节 |
| [ee_control.py](ee_control.py) | 末端控制的共用逻辑（面板和命令行都用它） |
| [panel.py](panel.py) / [ee.py](ee.py) | 末端控制面板 / 命令行 |
| [scenes/tabletop.yaml](scenes/tabletop.yaml) | 桌面场景，用 base_link 坐标。物体的 `labels` 就是 `find_objects` 能找到的英文名 |
| [launch/sim.launch.py](launch/sim.launch.py) | 先启动物理进程和 ROS 桥，3 秒后启动 ros2_control、MoveIt、点云节点和面板。照真机 `hardware.launch.py` 写 |
| [sim_env.sh](sim_env.sh) | 仿真专用的 ROS 环境 |
| [check_scene.py](check_scene.py) | 不用 ROS：渲染一帧，把深度反投影回 base_link，检查桌面和物体高度对不对，图存到 `sim/out/` |

harness 里和仿真有关的代码：

- [configs/sim.yaml](../configs/sim.yaml)：`--sim` 时叠加在 `harness.yaml` 上；
- [harness/perception/ground_truth.py](../harness/perception/ground_truth.py)：真值分割，输出格式和 SAM 3 相同；
- [rgbd_source.py](../harness/perception/rgbd_source.py)：彩色、深度、分割三张图按时间戳配对；
- [gripper_gdk.py](../harness/backends/gripper_gdk.py)：`sim_models`；
- 吸盘的仿真驱动在 agi@g2_robot 的 `suction_control/suctions/sim/`。

## 末端（`left_ee` / `right_ee`）

可选 `none`、`suction`、`omnipicker`，默认两只手都是 omnipicker。和真机 URDF 用的是同一组参数，见 [agi@g2_robot 的 end_effectors/README.md](../../agi@g2_robot/src/genie_robot_description/urdf/end_effectors/README.md)。仿真里：

| 末端 | 处理 |
|---|---|
| omnipicker | 用官方模型里的夹爪（和 URDF 比，底座、指根位姿差 0.00 mm） |
| none | 删掉那只手的夹爪，连同约束、驱动器 |
| suction | 删掉夹爪，换成和 `suction.xacro` 尺寸相同的长方体 |

## 对官方模型的改动（以及为什么）

| 改动 | 原因 |
|---|---|
| 底座固定，base_link 放在世界原点 | 原地操作；场景文件直接写 base_link 坐标 |
| **手臂、腰、头的网格打开碰撞** | 官方模型里只有夹爪和轮子能碰撞，手臂会直接穿过桌子 |
| 机器人自己的连杆之间不碰撞 | 自碰撞由 MoveIt 检查；关节处的网格本来就重叠 |
| 夹爪内部互不碰撞 | 官方 omnipicker 的连杆和底座本来就互相穿透（最深 7.9 mm），会产生不存在的内力 |
| 椭圆摩擦锥，`impratio` 10 | 夹东西不容易滑 |
| 头部相机用真机标定 | 内参 fx=309.359，外参用 `head_camera.yaml` 的数值 |

已核对：官方 MJCF 和 MoveIt 用的 URDF 在 20 组随机关节角下，手腕和头部的位姿差 0.00 mm、0.000°。

## 踩过的坑（2026-11-19）

| 现象 | 原因 | 办法 |
|---|---|---|
| 反投影出来的桌面高了 1 cm | MuJoCo 的主点偏移（`principal_pixel`）横纵都和图像坐标反号 | 按反号填，用小球投影验证过，误差小于 0.2 像素 |
| 物体穿过桌子掉下去 | 官方模型默认 `contype=0`，新加的几何体继承了这个默认值 | 显式设置碰撞位（见 `scene.py` 文件头） |
| 只有 0.4 倍实时 | ① 往 ROS 消息的 `data` 直接塞 bytes，一张深度图要 24 ms；② numpy 数组直接和 MuJoCo 枚举比较，会逐像素调用 Python，一帧 100 ms；③ 同一个渲染器来回切换彩色、深度、分割模式 | 用 `array.array("B", ...)`；先 `int()` 枚举；三个渲染器各管一种模式。现在每仿真 1 秒只用约 0.2 秒计算 |
| 同一进程里第二个渲染器读出的深度全是远裁剪面 | GLFW 后端的问题 | 用 `MUJOCO_GL=egl` |

## 已知的不同 / 没做的

- 拍照相机（`head_stereo_left`）和深度相机放在同一位置，视场角 90° 是估的。真机双目的外参没有标定。
- 六维力 = 夹爪上的接触力 + 夹爪自重，没有噪声，也没有死区。
- 没有腕部相机，也没有底盘雷达。
- 机器人关节不能复位（复位后 ros2_control 的指令会跳变），要复位就重启仿真。
