# 手臂运动链路：哪些运动查点云、touch_object 怎么碰到物体

2026-11-13 定。代码在 [harness/backends/ros.py](../harness/backends/ros.py)。

## 规则

| 工具 | 规划入口 | 点云碰撞 |
|---|---|---|
| `approach_object`、`move_arm_end`、`point_at_object` | [`plan_arm_to`](../harness/backends/ros.py#L241) | **全程查**。规划失败就是失败，如实告诉模型 |
| `move_joints`、`go_to_pose`、`look_at_object` | `plan_joints`（WholeBody） | 全程查 |
| `touch_object` | [`plan_touch`](../harness/backends/ros.py#L383) | 第一段查；**最后几厘米只放过物体附近的点云**（MoveIt 不查点云，改用 [`_keep_out_ok`](../harness/backends/ros.py#L480) 自己按几何查），桌面、别的东西、自碰撞、碰撞体照查 |

以前的“接触兜底”挂在 `plan_arm_to` 上，所有手臂运动规划失败时都会用，而且最后一段连自碰撞都不查。现在删了。

## plan_arm_to：按顺序试的步骤

`plan_arm_to` 把参数打包成 [`ArmGoal`](../harness/backends/ros.py#L43)，再按顺序试一组步骤（`_step_*`），第一个成功的就用：

1. [`_step_orientations`](../harness/backends/ros.py#L276)：终点姿态按 rpy / tool_axis / keep·comfort·free 依次试。每种都先走直线，再试 `via_then_rrt` 绕障。
2. [`_step_tilt_here`](../harness/backends/ros.py#L297)：给了 tool_axis 时，同一位置找容差（15°）内最接近要求的朝向。
3. [`_step_tilt_retreat`](../harness/backends/ros.py#L301)：给了 tool_axis 和 retreat 时，往身体方向后撤着转。
4. [`_step_body_assist`](../harness/backends/ros.py#L304)：手臂够不着时先前倾、转腰，腰到位后再规划手臂。

每一步失败都把原因记进 `failures`，全部失败时把原因拼在一起返回。要加一种兜底，就写一个 `_step_xxx(goal, failures)`，然后放进列表。

## touch_object 的固定链路

物体本身在点云里，直接规划到它跟前必然“会撞”。所以 touch 不先试再兜底，**一开始就走两段**：

```text
d  = 接近方向(夹爪尖朝 d)          依次试:末端→物体、肩→物体水平(物体在台面上时不用)、从上往下
                                   某个方向接触点够不着:先动腰再试这个方向,不直接换方向
C  = 物体 − gap·d                  接触点
P  = C − b·d                       预接触点,b = 5, 8, 12, 16, 20, 25, 30 cm(从近到远)

⓪ 在 C 定姿态和关节角 q_C         _touch_contact_pose:朝 d、roll 锁住(+0/±45/±90 依次试);
                                   严格朝 d 做不到,接受容差 15° 内最接近的;q_C 只要求不自碰撞
① 从 q_C 沿直线反着积分到 P → q_P  _touch_candidate(便宜,只算 IK + 碰撞查询):
                                   q_P 不撞(含点云);P→C 这段 MoveIt 除了点云不撞,
                                   夹爪胶囊不碰物体 8cm 以外的点云格(_keep_out_ok)
② 第一段:当前 → q_P(全程查碰撞)  _reach_precontact:
                                   直线 → via_then_rrt → 关节空间(经录好位姿中转 / RRT)
③ 最后一段:P → C                  _execute_touch:从实际位姿、保持实际姿态,
                                   用最新点云再做一次 _keep_out_ok,0.03 m/s 直线
```

### 2026-11-14 真机撞桌面:最后一段改成只放过物体附近的点云

“抓取最近的鼠标”:`approach_object` 到鼠标上方 12cm(朝下差 12.8°)没问题,接着 `touch_object` 却把手臂收回 30cm,夹爪水平往前推,撞上桌面。原因有三层:

1. **朝下在接触点够不着。** 鼠标离右肩 0.75m,接近伸直。离线用真实 URDF 算,夹爪尖离竖直朝下最少要偏:鼠标上方 12cm 12.6°、8cm 12.1°、4cm 13.5°,到接触点**无解**。
2. **动腰轮不到。** 原来只有所有方向都没候选时才动腰。“肩→物体水平”有候选,就直接换成了它。
3. **水平那段“不查点云”= 连桌面也不查。** 预接触点 5~25cm 都在点云里被拒,30cm 才通过,说明这段直线一路都在桌面和鼠标的点云里。桌面只在点云里(planning scene 没有桌面盒子),张开的夹爪比夹爪尖低几厘米,扫到桌面。

改法([harness/touch_keep_out.py](../harness/touch_keep_out.py)):

- **只放过物体附近的点云。** 最后一段仍然让 MoveIt 忽略 `<octomap>`(物体自己在里面),但另取 octomap 的格([`octomap_base`](../harness/motion/arm_motion.py#L426)),去掉物体坐标 `object_radius_m`(8cm)以内的,剩下的一个都不许碰。夹爪近似成胶囊:从夹爪尖沿工具轴往回 `gripper_length_m`(13cm),半径 `gripper_radius_m`(4cm)。规划时(验 P、验第一段终点)和执行前(用最新点云)各查一次。
- **够不着先动腰,再试同一方向**,不直接换方向。
- **物体放在台面上时不用“肩→物体水平”**([`_touch_directions`](../harness/backends/ros.py#L461),`rests_on_surface`:物体四周 6~20cm、比它低 0.5~10cm 处有 ≥5 个点云格)。举在空中的手四周下方是空的,击掌照样能水平碰。
- **结果里写上每个没用上的方向为什么没用上**(`direction_notes`),这次真机就是因为没有这一项,只能离线复现推原因。

胶囊是粗略近似:张开的 omnipicker 两指横向比 4cm 宽。几何检查拦的是“一路贴着桌面”这种情况;夹爪正下方、物体 8cm 以内的台面是放过的(要碰物体就得允许靠近它脚下的台面)。

### 2026-11-14 最后一段加六维力：碰到就停，往回退 2 cm

[`_execute_touch`](../harness/backends/ros.py) → `_run_force_guarded`。力来自 GDK `/gdk/motion_control_status` 的 `frame_wrenchs`（100 Hz，`arm_r_end_link`），读取在 [wrench_gdk.py](../harness/backends/wrench_gdk.py)，和纸板任务 `arm_touch_down` 同一个来源、同一种做法。

```text
到预接触点 P 后静止 0.3s 取零点(夹爪重量一起减掉)
gap 为 0:终点 = 接触点再往前 overshoot_m(5cm);够不着就 3cm、1.5cm、0
最后一段边走边读力:|F − 零点| > threshold_n(5N)→ 取消轨迹
停稳后沿接近方向往回退 retreat_m(2cm,给闭合夹爪留空间)→ 这次 touch 完成
力 0.1s 没更新 → 立刻停,报失败
收不到力 → 退回原来的做法:只走到接触点,不多走
```

- **为什么要越过接触点**：物体坐标是可见表面的中位点（鼠标就是顶面）。张开的夹爪要罩住鼠标往下，碰到桌面才算到底，这个“底”以力为准。
- **规划按最深点定**：gap 为 0 且有力时，`plan_touch` 的姿态、关节角、动腰都按“接触点 + 5 cm”来算。离线踩过：按接触点规划，手臂到了那里已经快伸直，只能再往下 1.5 cm，碰不到桌面。
- **gap > 0**（击掌停在手前 2 cm）：不多走；途中力超了（对方的手迎上来）同样停下并退回。
- 结果里的 `contact`：`felt`（True 感到接触 / False 没感到 / None 没有力数据）、`force_change_n`、`moved_m`、`overshoot_m`、`retreat_m`。走完 5 cm 也没感到力，消息里会写“可能没碰到”。
- **阈值 5 N 没在真机上调过**。慢速移动时外力估计会不会有读数、桌面碰上去是多少牛，要先用 `arm_monitor.py --arm right` 看。

### 为什么第一段“能去就一定规划得出来”

- 终点不是只给一个位姿，而是一个**已经验证过不撞的关节角 q_P**。末端规划（直线、绕障）不通时，改用关节空间直接去 q_P。终点构型本身合法，只要起点和它之间连得通，关节 RRT 就找得到。
- 末端规划到了 P，但构型分支可能和验过的不一样。[`_final_segment_ok`](../harness/backends/ros.py#L547) 会再验一次“从这里直线下得去”，下不去也改走关节空间。

### 为什么从接触点反着积分（2026-11-13 离线踩的坑）

起初是在 P 处解 IK，再验 P→C 直线。用真实 URDF 测鼠标那个场景：P 处 IK 给出关节 1 约 −163°，接触点的解是 −7°，两者不在同一个构型分支，P→C 直线在关节限位上卡住。于是 37 个候选全部“最后一段走不通”。

改成先解 C，再从 q_C 沿直线反着积分到 P。这样 P 和 C 一定在同一个分支，P→C 一定走得通，鼠标场景当场就有解。姿态也在 C 定，因为最后一段全程保持这个姿态，C 是最难够到的点。

### 够不着时

如果连接触点都没有 IK 解（不考虑碰撞），先动腰（`_body_assist`，`after = {"kind": "touch"}`）。腰到位后**整个 plan_touch 重来**，这次不再动腰。

## 参数（[configs/harness.yaml](../configs/harness.yaml#L50) `motion.touch`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `backoffs_cm` | 5…30 | 预接触点离接触点多远。最后一个值就是“不查点云”那段的最长距离。不从 3 cm 开始，因为点云 3 cm 一格，外扩以后 3 cm 处基本都“撞” |
| `roll_offsets_deg` | 0, ±45, ±90 | 同一方向下手腕依次多转这些角度再试 |
| `max_plan_tries` | 4 | 最多真规划几个验过的 P（每个最坏约 10 s） |
| `final_speed_m_s` | 0.03 | 最后一段的速度 |
| `joint_speed_rad_s` | 0.20 | 第一段改走关节空间时的速度 |

## 起点就在接触里（夹着东西、贴着桌面）

| 执行器 | 规则 |
|---|---|
| ArmMotion（[`path_clear`](../harness/motion/arm_motion.py#L400)） | 末端离起点 `ESCAPE_M` 6 cm 以内的碰撞不算；一旦脱离就不许再撞 |
| WholeBody（[`_near_start`](../harness/motion/whole_body.py#L122)） | 起点就有、和这次不动的部件有关的接触：一直不算。和要动的部件有关的接触：每个关节离起点都在 `ESCAPE_RAD` 0.10 rad 以内时不算。所以夹着鼠标也能 `go_to_pose(init)` |

## 验证

- `test_ros_backend_logic.py` 的六维力测试（替身力：夹爪尖低于桌面 20 N；替身 JTC 一个点一个点走，能被取消）：越过鼠标顶面，碰到桌面停下并退 2 cm；没感到力时如实说；收不到力只走到接触点；力中途断了立刻停；gap > 0 不多走。

- [tests/test_ros_backend_logic.py](../tests/test_ros_backend_logic.py)：用真实 `G2_t2_crsB_ee.urdf` 的 IK，加上替身点云（鼠标周围 10 cm）：
  - 先动腰到鼠标上方，再 touch 从上往下到鼠标（夹爪离竖直 5.9°）；
  - 直线和绕障都不通时，第一段改走关节空间；
  - `plan_arm_to` 到点云里的目标会失败，而且从不用 `ignore_cloud`；
  - 够不着时 touch 先动腰，然后只重规划一次，不再动腰。
- [tests/test_tcp_and_escape.py](../tests/test_tcp_and_escape.py) `test_ignore_cloud_only_ignores_octomap`：`ignore_cloud` 只放过 `<octomap>`，自碰撞和碰撞体照样拦。
- [tests/test_touch_keep_out.py](../tests/test_touch_keep_out.py)：胶囊几何检查和台面判断（水平贴桌面被拦、朝下 / 偏 15° 下去不拦、空中的手不算在台面上）；`test_ros_backend_logic.py` 后 3 个测试复现 11-14 那次（不再水平平推，改成先动腰再朝下）。
- **真机没跑。**
