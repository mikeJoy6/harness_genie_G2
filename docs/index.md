# 文档目录

| 文档 | 内容 |
|---|---|
| [harness_guide.md](harness_guide.md) | 一篇读懂全部代码:每个函数、调用顺序、时序图、动手练习 |
| [perception_plan.md](perception_plan.md) | 感知升级:SAM 3、G1 点云聚类、对比与接入顺序 |
| [sam3_service.md](sam3_service.md) | 本机 SAM 3 分割服务:安装、权重、接口、踩坑 |
| [look_at_object.md](look_at_object.md) | 技能:SAM 3 按名字找物体 + 转头看它,日志格式 |
| [goal_pose.md](goal_pose.md) | 手臂终点姿态:保持 / 三角函数舒适姿态 / 放松,离线对比 |
| [gripper.md](gripper.md) | 夹爪:GDK 接口、关节名、取值、怎么判断成功 |
| [touch_chain.md](touch_chain.md) | 手臂运动链路:哪些查点云、touch_object 两段(验过的预接触点 → 不查点云贴上去)、起点在接触里怎么脱离 |
| [pointcloud_cleanup.md](pointcloud_cleanup.md) | 点云:空中零星格为什么清不掉(背面补全变成 8 格条)、悬空块清理、相机动了接着清 |
| [object_handles.md](object_handles.md) | 物体编号表设计(未实现)+ 相关论文 |
| [media/harness_overview.png](media/harness_overview.png) | 一张图看全部文件夹、文件、函数和数据流;由 [make_overview.py](media/make_overview.py) 生成(**已过期**,还画着 HSV) |
| [media/SHOT_LIST.md](media/SHOT_LIST.md) | README 缺的图和动图:拍什么、怎么拍、文件名 |

开发过程按日期记在 [../devlog.md](../devlog.md)。
