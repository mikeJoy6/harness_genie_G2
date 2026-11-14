# 手臂运动链路：哪些运动查点云、touch_object 怎么碰到物体

2026-11-13 定。代码在 [harness/backends/ros.py](../harness/backends/ros.py)。

## 规则

| 工具 | 规划入口 | 点云碰撞 |
|---|---|---|
| `approach_object`、`move_arm_end`、`point_at_object` | [`plan_arm_to`](../harness/backends/ros.py#L240) | **全程查**。规划失败就是失败，如实告诉模型 |
| `move_joints`、`go_to_pose`、`look_at_object` | `plan_joints`（WholeBody） | 全程查 |
| `touch_object` | [`plan_touch`](../harness/backends/ros.py#L382) | 第一段查；**只有最后几厘米不查点云**，自碰撞和碰撞体（例如桌面盒子）仍然查 |

以前的“接触兜底”挂在 `plan_arm_to` 上，所有手臂运动规划失败时都会用，而且最后一段连自碰撞都不查。现在删了。

## plan_arm_to：按顺序试的步骤

`plan_arm_to` 把参数打包成 [`ArmGoal`](../harness/backends/ros.py#L42)，再按顺序试一组步骤（`_step_*`），第一个成功的就用：

1. [`_step_orientations`](../harness/backends/ros.py#L275)：终点姿态按 rpy / tool_axis / keep·comfort·free 依次试。每种都先走直线，再试 `via_then_rrt` 绕障。
2. [`_step_tilt_here`](../harness/backends/ros.py#L296)：给了 tool_axis 时，同一位置找容差（15°）内最接近要求的朝向。
3. [`_step_tilt_retreat`](../harness/backends/ros.py#L300)：给了 tool_axis 和 retreat 时，往身体方向后撤着转。
4. [`_step_body_assist`](../harness/backends/ros.py#L303)：手臂够不着时先前倾、转腰，腰到位后再规划手臂。

每一步失败都把原因记进 `failures`，全部失败时把原因拼在一起返回。要加一种兜底，就写一个 `_step_xxx(goal, failures)`，然后放进列表。

## touch_object 的固定链路

物体本身在点云里，直接规划到它跟前必然“会撞”。所以 touch 不先试再兜底，**一开始就走两段**：

```text
d  = 接近方向(夹爪尖朝 d)          依次试:末端→物体、肩→物体水平、从上往下
C  = 物体 − gap·d                  接触点
P  = C − b·d                       预接触点,b = 5, 8, 12, 16, 20, 25, 30 cm(从近到远)

⓪ 在 C 定姿态和关节角 q_C         _touch_contact_pose:朝 d、roll 锁住(+0/±45/±90 依次试);
                                   严格朝 d 做不到,接受容差 15° 内最接近的;q_C 只要求不自碰撞
① 从 q_C 沿直线反着积分到 P → q_P  _touch_candidate(便宜,只算 IK + 碰撞查询):
                                   q_P 不撞(含点云);P→C 这段除了点云不撞
② 第一段:当前 → q_P(全程查碰撞)  _reach_precontact:
                                   直线 → via_then_rrt → 关节空间(经录好位姿中转 / RRT)
③ 最后一段:P → C                  _execute_touch:从实际位姿、保持实际姿态,
                                   不查点云,0.03 m/s 直线
```

### 为什么第一段“能去就一定规划得出来”

- 终点不是只给一个位姿，而是一个**已经验证过不撞的关节角 q_P**。末端规划（直线、绕障）不通时，改用关节空间直接去 q_P。终点构型本身合法，只要起点和它之间连得通，关节 RRT 就找得到。
- 末端规划到了 P，但构型分支可能和验过的不一样。[`_final_segment_ok`](../harness/backends/ros.py#L506) 会再验一次“从这里直线下得去”，下不去也改走关节空间。

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

- [tests/test_ros_backend_logic.py](../tests/test_ros_backend_logic.py)：用真实 `G2_t2_crsB_ee.urdf` 的 IK，加上替身点云（鼠标周围 10 cm）：
  - 先动腰到鼠标上方，再 touch 从上往下到鼠标（夹爪离竖直 5.9°）；
  - 直线和绕障都不通时，第一段改走关节空间；
  - `plan_arm_to` 到点云里的目标会失败，而且从不用 `ignore_cloud`；
  - 够不着时 touch 先动腰，然后只重规划一次，不再动腰。
- [tests/test_tcp_and_escape.py](../tests/test_tcp_and_escape.py) `test_ignore_cloud_only_ignores_octomap`：`ignore_cloud` 只放过 `<octomap>`，自碰撞和碰撞体照样拦。
- **真机没跑。**
