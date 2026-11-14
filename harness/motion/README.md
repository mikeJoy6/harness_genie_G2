# harness/motion/ —— 手臂运控与绕障规划

## 这里干什么

把“去某个点”变成一条安全的关节轨迹,发给 `hardware.launch.py` 里的 JTC 控制器执行。包括运动学、IK、笛卡尔直线规划、绕障规划、全身位姿运动。

这里的代码**复制自 `agi@g2_robot`**(提交 `ecdfb5d`,2026-11-08)。这样 harness 只需要 `hardware.launch.py` 在跑,不再 import `agi@g2_robot` 的任何包。

除了 import 路径,只改了一处:`whole_body.py` 的 `WholeBody.goto` 拆成了 `plan()` 和 `run()` 两步(`goto` 仍然存在,等于 plan + run),这样 harness 能在规划和执行之间让人确认;`run()` 发现机器人离开了规划起点就不执行。

## 这里的文件

- `goal_pose.py`:手臂终点姿态的三种来源(保持 / 三角函数舒适姿态 `ComfortPose` / 只管位置 `free_goal`),见 [docs/goal_pose.md](../../docs/goal_pose.md)

| 文件 | 作用 | 来源 |
|---|---|---|
| `arm_motion.py` | `ArmMotion` 节点:直线规划、绕障规划、自碰撞检查、发给 JTC、确认到位 | `genie_robot_agibot_g2_motion` |
| `kin.py` | URDF → KDL 运动链:正运动学、雅可比 | 同上 |
| `algorithms.py` | 积分笛卡尔(直线、构型连续)、多种子 IK、`path_sane` 防大回环 | 同上 |
| `ik_lab/` | 轨迹计时(五次多项式)、终点精修用的自适应 LM | 同上 |
| `planners/` | 绕障规划器 `via_then_rrt` 及其依赖 | 同上 |
| `octomap_util.py`、`voxel_util.py` | 读 MoveIt octomap、四元数转矩阵 | `genie_robot_agibot_g2_perception` |
| `whole_body.py`、`joints.py`、`joint_detour.py` | 全身(腰、头、双臂)关节点到点,直线会撞时经录好的位姿中转或关节 RRT | `task_board_pose` |

`planners/` 只拷了 `via_then_rrt` 用到的部分。原项目的另外 8 个规划器是对比实验用的,没有拷。

## 为什么选 via_then_rrt

`agi@g2_robot` 用离线基准 `planner_bench` 对比了 9 个规划器。`via_then_rrt` 在 3 组留出题(共 196 道绕障题)里成功 183 道,成绩最好,规划耗时中位数 0.58 秒,是原项目的默认规划器。

流程:

```text
直线可达?        → 直接走直线
V / 梯形 / U 型绕行模板(几何上最短的先试)
                 → 找到就用
末端空间 RRT 兜底 + 捷径平滑
```

每一段都用积分笛卡尔小步走,构型连续,不会“末端走一点、胳膊抡一大圈”。

## 和哪里联动

```text
harness/backends/ros.py
   └─ ArmMotion("robot_harness_arm")
        ├─ 订阅 /robot_description、/joint_states
        ├─ 调用 /check_state_validity、/get_planning_scene        ← move_group
        └─ 发送 /left_arm_group_controller/follow_joint_trajectory ← JTC

scripts/goto_pose.py
   └─ WholeBody + JointDetour
        ├─ 读 configs/poses.yaml
        └─ 发送 4 个 JTC:body / head / left_arm / right_arm
```

上面这些接口全部由 `hardware.launch.py` 提供。

## 示例

`ros.py` 里的用法:先试直线,不行再绕障。

```python
plan = self.arm.plan_cartesian("left", target)        # 直线,姿态不变
if isinstance(plan, str):                              # 失败时返回原因字符串
    plan = self.arm.plan_detour("left", target)        # via_then_rrt
ok, msg = self.arm.execute(plan)
```

单独测试规划器(需要 ROS 环境):

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest tests/test_motion.py -q
```

`test_planner_detour_around_post` 会在右臂前方立一根柱子,要求规划器必须绕开它。

## 和原项目同步

原项目修了 bug 时,用 `diff` 对比后手动合并:

```bash
diff ~/agi@g2_robot/src/genie_robot_agibot_g2_motion/genie_robot_agibot_g2_motion/arm_motion.py harness/motion/arm_motion.py
```

唯一预期的差异是 `arm_motion.py` 开头两行 import(`octomap_util`、`voxel_util` 改成了本目录的相对导入)。
