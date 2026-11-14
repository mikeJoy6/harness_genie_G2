# 感知升级方案:从颜色到“任意物体”

现状:[`locate_object(color)`](../harness/gateway.py#L69) 只能按 HSV 颜色找一个物体([`color_object.py`](../harness/backends/color_object.py)),工具参数是颜色枚举。目标:用户说“去笔那里”“最左边那个积木”也能找到。

本文比较三种识别来源,结论是**点云聚类负责几何,SAM 3 负责语义,两者互补**。物体找到之后怎么交给运动工具,见 [object_handles.md](object_handles.md)。

## 1. SAM 3(文字 → 掩码)

Meta 2025-11-19 发布,全称 *Segment Anything with Concepts*。

| | SAM 1 / SAM 2 | SAM 3 |
|---|---|---|
| 提示 | 点、框、掩码 | 加上**短名词短语**和示例图 |
| 输出 | 你点的那一个 | 和短语匹配的**所有**实例,各有掩码 |

```python
model = build_sam3_image_model()
processor = Sam3Processor(model)
state = processor.set_image(image)
out = processor.set_text_prompt(state=state, prompt="green pepper")
out["masks"], out["boxes"], out["scores"]          # (N,1,H,W) bool、xyxy 像素、分数
```

- 8.48 亿参数;Python ≥ 3.12、PyTorch ≥ 2.7、CUDA ≥ 12.6。
- 权重门控(Hugging Face `facebook/sam3`),SAM License。
- 显存、速度无官方数据。第三方在 RTX 5060 Ti 上 fp16 约 1.9 GB、100~180 ms/张;本机要实测,见 [sam3_service.md](sam3_service.md#测试结果)。
- **Python 3.12 和 ROS Humble 的 3.10 冲突**,所以做成独立服务 → [sam3_service.md](sam3_service.md)。

### 候选对比

| 方案 | 输出 | 许可证 | 结论 |
|---|---|---|---|
| **SAM 3** | 文字 → 所有实例掩码 + 框 + 分数 | SAM License | 首选:一个模型做完 |
| Grounding DINO + SAM 2 | 文字 → 框 → 掩码 | Apache 2.0 | 备选:许可证宽、Python 要求低 |
| YOLO-World / YOLOE | 文字 → 框(YOLOE 有掩码) | GPL-3.0 / AGPL-3.0 | 不需要:优势是实时,我们每条指令只识别一次 |

## 2. G1 点云聚类(无标签)

来源:`~/g1_pointcloud_grasping/src/genie_perception/genie_perception/depth_cloud.py`(1356 行),在 G1 上抓取成功过(提交 `655a918`)。

```text
深度图 → 每 4 像素取一点 → 相机系点云(带 RGB)
 ① RANSAC 拟合桌面,只留桌面以上            fit_plane_ransac()            :52
 ② DBSCAN(半径 2.5cm,最少 6 邻居,<15 点丢)  sklearn                       :666
 ③ 只留底部贴桌面的簇(4cm 容差)             table_contact_metrics()       :113
 ④ 去桌面条纹(扁长贴桌簇),颜色差大的小物体保护  table_stripe_reason()        :360
                                              small_object_color_evidence() :253
 ⑤ 每簇 FPS 采 16 点                          fixed_fps_per_cluster()       :174
 ⑥ 转 base_link,算中心、顶面中心、主轴、半宽   fps_geometry.py
 → /perception/fps_geometry(每物体一条,带 object_id)
```

抓取端 `genie_task_grasp/src/grasp_keyboard.cpp:301` 选离肩最近的物体,用顶面中心 + 主轴定姿态;执行前按 2 cm 内的几何位置重新匹配(:322),不信每帧编号。

| 优点 | 缺点 |
|---|---|
| 类别无关,不用训练、不用 GPU | 没有语义,不知道哪个是“笔” |
| 直接给 3D 几何(中心、顶面、主轴、尺寸) | 前提强:要有主平面;物体间距 > 2.5 cm;≲1.5 cm 的扁物体会被滤掉;透明/黑/反光物体没深度 |
| 只要 numpy + sklearn,Python 3.10 能跑 | 条纹、颜色保护阈值是为白桌调的 |
| 阈值有 2026-10-22 真机日志支撑 | DBSCAN 编号每帧都变,不能当跟踪 ID |
| | G1 话题 `/sdk/head_color`,G2 是 `/gdk/camera/...`,ROS 层要重写 |

## 3. 对比与建议

| | 点云聚类 | SAM 3 |
|---|---|---|
| 回答 | 桌上**哪里**有东西 | 图里**哪个**是 “pen” |
| 输出 | 3D 簇 + 几何 | 2D 掩码 + 分数 |
| 挨着的物体 | 合成一个 | 能分开 |
| 依赖 | numpy、sklearn | GPU、Python 3.12、独立服务、门控权重 |

**顺序:**

1. **先接聚类**:新工具 `list_objects()` 返回每个物体的编号、中心、尺寸、平均颜色、离手距离,模型按颜色/大小/位置挑。只搬 RANSAC、DBSCAN、贴桌过滤、几何计算;条纹过滤和跨帧跟踪先不搬。取帧和 [`points_base`](../harness/perception/scene3d.py#L36) 复用 harness 现有的。
2. **再加 SAM 3**:掩码和簇做匹配,给簇贴上 “pen” 之类的标签。
3. **顺带解决绕障看不到桌子**:聚类已经拟合出桌面平面,可以把它作为碰撞体加进 MoveIt planning scene(`use_head_cloud:=false` 时 MoveIt 没有 octomap,见 [devlog 2026.11.11](../devlog.md#20261111))。

## 参考

- [SAM 3 官方仓库](https://github.com/facebookresearch/sam3) · [Meta 博客](https://ai.meta.com/blog/segment-anything-model-3)
- [第三方显存与速度实测(fastforks/sam3)](https://huggingface.co/fastforks/sam3)
- [SAM 2 到 SAM 3 综述,arXiv:2512.06032](https://arxiv.org/pdf/2512.06032v1)
- [Roboflow:Grounded SAM vs SAM 3](https://playground.roboflow.com/models/compare/grounded-sam-vs-sam-3)
