# 点云：空中零星格为什么清不掉、现在怎么清

2026-11-13。代码不在本仓库，在 `~/agi@g2_robot/src/genie_robot_agibot_g2_perception/`（`hardware.launch.py` 默认会拉起）。`~/ai_assistant/src/g2_head_pointcloud/` 里的副本已同步。

## 链路

```text
/gdk/camera/head_depth
  └─ head_depth_cloud(十几道过滤,源头删 <4 格的小块)──→ /head_camera/depth_points ──→ MoveIt head_depth 传感器 → octomap
        └─ MoveIt 滤掉机器人自身 ──→ /head_camera/filtered_points
              └─ shadow_cloud:每个点沿“相机→点”往后补 25cm(~8 格)──→ head_shadow 传感器 → octomap
octomap_sweeper:定时取 octomap,挑出要清的格,沿相机射线发“只清空”的远点 ──→ head_sweep 传感器
```

octomap 里的格只有被射线穿过才会被清掉。不在视野里、被挡住的格子，只能靠 `octomap_sweeper` 发假射线去清。sweeper 的假射线不看真实画面，所以视野外的格子也清得到。

## 问题

1. **背面补全把假点变成了“大块”。** `head_depth_cloud` 只能在源头删掉小于 4 格的小块。漏进来的点经过 `shadow_cloud` 往后补 25 cm，一个点就变成约 8 格的一根“条”。sweeper 原来只清小于 4 格的块，所以这些条永远不会被当成目标。只要它们不在视野里，或者被挡住，就一直留在 octomap 里。这就是“空中飘着零星点云”。
2. **相机一动就停清。** sweeper 的射线是按取 octomap 那一刻的相机位置算的。相机动了超过 1 cm（转头、弯腰）就整轮跳过，只能等下一轮再算。
3. **清得慢。** 一个占满的格子要被清空射线穿过大约 9 次才会消失。原来是 5 Hz，加上 1 s 一轮的周期，一个格子要 2~3 s 才清掉。

## 改法（[octomap_sweeper.py](../../agi@g2_robot/src/genie_robot_agibot_g2_perception/genie_robot_agibot_g2_perception/octomap_sweeper.py)）

- 新增 `pick_targets()`（第 47 行，纯计算）。octomap 的格按 `connect_reach: 2` 连成块，满足下面任一条的块整块清掉：
  - **小块**：小于 `min_voxels`（4）格，不管在哪，和原来一样；
  - **悬空块**：小于 `floating_max_voxels`（40）格，**且**整块最低一格的中心高于 `support_z`（0.15 m，base_link 坐标；地面约 −0.06）。这样的块既不落地，也没和桌子、人、箱子这些大块连在一起。
- 新增 `_build(o)`：按相机位置 o 算清空射线和保护点。`_sweep` 发现相机动了，就按新位置重算，不再跳过。
- `period` 1.0 → 0.5 s；`burst_rate` 和 `sensors_3d.yaml` 里 head_sweep 的 `max_update_rate` 都是 5 → 10 Hz，一个格大约 1 s 清掉。

保护机制没改：清空射线路上碰到大块占据格时，同一帧在那些格上放点，MoveIt 就不会顺带清掉它们。

### floating_max_voxels 怎么选

| 东西 | 格数（3 cm 格，含背面补全） | 默认 40 时 |
|---|---|---|
| 一个假点 + 补全 | ~8 | 清 |
| 一小团假读数 + 补全 | 20~40 | 清 |
| 举在空中、身体没被拍到的人手 | 50~150 | **留** |
| 桌面（桌腿被挡住） | 几百 | 留（大） |
| 桌上的鼠标 | 几格，但和桌面连着 | 留 |

如果调大，空中的真东西也会被删掉，机器人就“看不见”它了。如果调小，大一点的假块会漏掉。

## 离线验证

[tests/test_octomap_sweeper_pick.py](../tests/test_octomap_sweeper_pick.py)（直接 import agi@g2_robot 里的 `pick_targets`）。构造了桌面（腿被挡住）、桌上鼠标、站着的人、空中假点条、空中的手、低处单格：

```text
桌面      540 格  清掉 0
鼠标        4 格  清掉 0
人      5400 格  清掉 0
空中假点条     8 格  清掉 8
空中的手     72 格  清掉 0
单格        1 格  清掉 1
```

`_build` 用假实例跑过：相机移动后会重新算出 16 条射线。**真机没跑**，需要重新编译并重启 hardware.launch，命令见 devlog。
