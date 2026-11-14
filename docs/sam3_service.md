# SAM 3 分割服务(`~/sam3_service`)

给一张图和一个英文短语(如 `red block`),返回图里**所有**匹配物体的掩码、框和分数。它是 harness 将来“按名字找物体”的识别器,背景和选型见 [perception_plan.md](perception_plan.md)。

> 模型服务在本机 `~/sam3_service/`(不在本仓库,链接是本机相对路径,GitHub 上打不开);调用它的代码都在本仓库。

## 代码放在哪:按 Python 环境拆成两边

| | `~/sam3_service`(只有模型) | `robot_harness`(机器人这边全部) |
|---|---|---|
| Python | 3.12(SAM 3 要求),uv 建的 `.venv` | 3.10(ROS 2 Humble 自带) |
| 知道 ROS / G2 吗 | **不知道**:只管“图 + 短语 → 掩码” | 知道:话题、TF、标定、运动 |
| 文件 | `server.py` | `sam3_client.py`、`sam3_live.py`、`sam3_test_image.py`、`capture_head.py` |

两个 Python 版本不能在一个进程里 import,所以必然是两个进程;那么**分界线就按进程画**:跑在 3.12 里的只有模型,其他(包括客户端)都和 harness 放一起,因为它们要用 ROS、要被 harness import。

- 不放进 `agi@g2_robot`:harness 已经去掉了对它的依赖(见 [devlog 2026.11.09](../devlog.md#20261109))。
- 换识别器(比如 Grounded-SAM-2)时,只要新服务接口一样,harness 一行不改。

```text
robot_harness(Python 3.10)                          sam3_service(Python 3.12)
  /gdk/camera/head_color(JPEG)                          server.py
     │ Sam3Client.segment_jpeg(jpeg, ["red block"])     │ 模型常驻显存
     └──── HTTP:请求体 = JPEG 原始字节,短语在头里 ───▶ │
     ◀──── JSON:每个实例 mask(PNG)、box、score ───────┘
  掩码 + 深度 → base_link 3D 点 → 物体坐标(Rgbd.points_base)
```

最后一步复用 harness 现有的 [`Rgbd.points_base`](../harness/perception/scene3d.py#L36)。

## 文件

| 文件 | 作用 | 用哪个 Python |
|---|---|---|
| [`~/sam3_service/server.py`](../../sam3_service/server.py) | 服务:[`load_model`](../../sam3_service/server.py#L48) → [`Segmenter.segment`](../../sam3_service/server.py#L77) → [`make_handler`](../../sam3_service/server.py#L109) 处理 `/health`、`/segment` | `.venv/bin/python` |
| [`harness/perception/sam3_client.py`](../harness/perception/sam3_client.py) | [`Sam3Client`](../harness/perception/sam3_client.py#L25):`segment(bgr)`、`segment_jpeg(jpeg)`(掩码解码成 bool 数组);[`draw`](../harness/perception/sam3_client.py#L81) 画掩码 | 3.10,只依赖 numpy、OpenCV |
| [`scripts/sam3_live.py`](../scripts/sam3_live.py) | **实时**:头部相机 + SAM 3,终端里随时换要找的物体,见下面“实时分割” | 3.10 + ROS |
| [`scripts/sam3_test_image.py`](../scripts/sam3_test_image.py) | 拿一张图测服务,结果图存 `traces/sam3_<名字>.jpg` | 3.10,不需要 ROS |
| [`scripts/capture_head.py`](../scripts/capture_head.py) | 录头部相机:彩色 + 深度(mm)+ `base_link→head_link3`,只订阅不动机器人 | 3.10 + ROS |
| `recordings/head_<时间>/` | 录下的帧,格式同 [`DatasetSource`](../harness/perception/rgbd_source.py)(`frames.jsonl`);已 gitignore | — |
| `~/sam3_service/sam3_repo/` | 官方代码 `facebookresearch/sam3`(提交 `f92a5ef`),`uv pip install -e` 装进 `.venv` | — |

## 安装(从零重建)

```bash
cd ~/sam3_service
git clone https://github.com/facebookresearch/sam3 sam3_repo     # 不能叫 sam3/,见下面“踩坑”
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python torch==2.10.0 torchvision --index-url https://download.pytorch.org/whl/cu128
uv pip install --python .venv/bin/python -e ./sam3_repo
uv pip install --python .venv/bin/python einops pycocotools psutil scipy   # 官方列为可选,实际 import 链要用
```

结果:Python 3.12.13、torch 2.10.0+cu128、RTX 4070 Laptop 8GB 可见,`.venv` 约 6.9 GB。用 uv 而不用 conda:装包快很多,环境就是目录里的 `.venv`,删掉即卸载。

## 权重(门控)

1. 在 <https://huggingface.co/facebook/sam3> 申请访问(SAM License,不是 Apache)。
2. 登录。本机终端设了 `all_proxy=socks://127.0.0.1:7897`,`hf` 不认 `socks://`,临时改成 `http://`(Clash 7897 是混合端口):

   ```bash
   export all_proxy=http://127.0.0.1:7897 ALL_PROXY=http://127.0.0.1:7897
   .venv/bin/hf auth login        # 选 "Log in with your browser",不用粘贴 token
   ```

   token 是密码,不发到聊天里、不写进代码和 git。
3. 下载(约 3.4 GB,到 `~/.cache/huggingface/hub/models--facebook--sam3`):

   ```bash
   HF_HUB_DISABLE_XET=1 https_proxy=http://127.0.0.1:7897 .venv/bin/hf download facebook/sam3 sam3.pt config.json
   ```

   [`build_sam3_image_model`](../../sam3_service/sam3_repo/sam3/model_builder.py) 第一次也会自己下,但走 Xet,见“踩坑”。

## 启动和测试

```bash
cd ~/sam3_service
.venv/bin/python server.py                       # 看到“服务已启动 http://127.0.0.1:8765”
# 另一个终端,在 ~/robot_harness
python3 scripts/sam3_test_image.py recordings/head_20261112_015829/color_04.png "red block" "pen"
```

## 实时分割(`sam3_live.py`)

```bash
cd ~/sam3_service && .venv/bin/python server.py           # 终端 1
source ~/agi@g2_robot/env_g2.sh                           # 终端 2
cd ~/robot_harness && python3 scripts/sam3_live.py "red block" "pen"
#   然后在终端 2 输入 green block, black bag 回车 → 画面上的掩码换掉;空行清空;q 退出
python3 scripts/sam3_live.py --sync "pen"                 # 显示被分割的那一帧:掩码严格对齐,但画面慢 0.35 s
```

只订阅 `/gdk/camera/head_color`,机器人不动。

### 快输入、慢处理

相机约 27 帧/秒,SAM 3 约 0.35 秒一帧。每帧都排队的话,队列会越堆越长、画面越来越旧。所以相机和分割之间**不用队列,只用一个“最新帧格子”** [`LatestSlot`](../scripts/sam3_live.py):新帧直接盖掉旧的,分割线程每次只拿当时最新那一帧。

```text
ROS 回调线程   on_image(msg): frames.put((msg, 收到时间))      只换引用,不解码不复制
      │
      ▼  frames:LatestSlot,只有最新一帧
分割线程       segment_loop: frames.wait_newer() → segment_jpeg(msg.data) → results.put()
      │
      ▼  results:LatestSlot,只有最新一次结果
主线程         有新帧才 decode() → draw(最新掩码) → cv2.imshow       (imshow 必须在主线程)
输入线程       input() → Prompts.set(短语)                         (input 会阻塞,单独一个线程)
```

| 问题 | 处理 |
|---|---|
| 旧图堆积 | 两层都只留 1 帧:ROS 订阅 QoS `KEEP_LAST depth=1`;程序里 `LatestSlot` 覆盖写。跳过多少帧会显示在画面上(`skipped`) |
| 分割线程空转 | `LatestSlot.wait_newer` 用 `threading.Condition` 睡着等,`put()` 时叫醒;没有新帧就不分割同一帧第二次 |
| 指向最新但不真取图 | 回调和格子里存的是 ROS 消息对象的**引用**;只有真要用时才解码:显示线程有新帧才解一次,分割线程不解码 |
| 零拷贝 | G2 发的就是 JPEG。`msg.data` 是 rclpy 反序列化出来的 `array.array`;分割线程用 `memoryview` 把它**原样**作为 HTTP 请求体发出(服务端新加的 `Content-Type: image/jpeg` 接口,不 base64);显示用 `np.frombuffer` 在同一块内存上解码。JPEG 字节全程只有 rclpy 那一份 |
| 改了短语,旧结果还在画 | `Prompts` 每改一次 `version + 1`,结果带着算它时的 `version`,对不上就不画 |
| 掩码和画面错位 | 默认画在最新帧上(流畅,物体或头在动时会差 0.35 s);`--sync` 画在被分割的那一帧上(对齐,画面慢) |
| 服务没开 / 出错 | 画面上显示 `ERROR`,1 秒后重试,不退出 |

实测(假相机在 ROS domain 77 回放录的 5 帧,30 Hz):画面 27 fps,分割 2~3 fps,`infer` 350~390 ms,掩码比画面晚 350~410 ms;中途改短语、清空都即时生效;15 秒里跳过 288 帧旧图,没有堆积。**真机还没跑过。**

## 接口

`POST /segment` 有两种写法。实时用第一种(`segment_jpeg`),一次性脚本两种都行。

1. 请求体 = JPEG 原始字节,`Content-Type: image/jpeg`,头 `X-Sam3-Request: {"prompts": [...], "threshold": 0.5}`。
2. JSON:

```json
{"image_jpg_b64": "...", "prompts": ["red block", "pen"], "threshold": 0.5}
```

回复:每个短语一组实例,按分数从高到低。

```json
{"ok": true, "width": 640, "height": 400,
 "results": [{"prompt": "red block",
              "instances": [{"score": 0.91, "box_xyxy": [x0, y0, x1, y1], "area_px": 5321, "mask_png_b64": "..."}]}],
 "timing_ms": {"decode": 3, "image": 120, "text": [25, 22], "encode": 4}}
```

- 短语用**英文名词短语**。中文由大模型翻译。
- 同一张图的多个短语只算一次图像特征(`image`),每个短语只多花 `text`。
- `threshold` 默认 0.5,找不到可降到 0.3。
- 任何异常(坏图片、缺字段、显存不够)都回 500 + 错误信息,服务不退出。

## 踩坑记录

| 现象 | 原因 | 解决 |
|---|---|---|
| `hf auth login` 报 `Unknown scheme for proxy URL URL('socks://127.0.0.1:7897/')` | httpx 不认 `socks://`,要 `socks5://` 或 `http://` | 命令前设 `all_proxy=http://127.0.0.1:7897` |
| 粘贴 token 后 `Invalid user token` | 屏幕不回显,`Ctrl+V` 输入的是控制字符或粘了两次 | 改用浏览器登录 |
| 启动报 `TypeError: expected str ... not NoneType`(`pkg_resources.resource_filename`) | 克隆目录叫 `sam3/`,在 `~/sam3_service` 里运行时 `import sam3` 先找到这个文件夹(无 `__init__.py` 的命名空间包,没有 `__file__`),而不是装好的包 | 目录改名 `sam3_repo/`,重装 `-e ./sam3_repo`。和 learning 里 `config.py` 跟库重名是同一类问题 |
| 权重下载到 645 MB 后会话重启,重新下又从 0 开始,17:35 起彻底卡住 | 默认 Xet 协议走代理不稳定,且部分下载不能续传 | `HF_HUB_DISABLE_XET=1` 走普通 HTTP,可断点续传;用 `setsid nohup` 起,不随会话退出 |
| 权重已下好,`server.py` 启动仍报 `Unknown scheme for proxy URL 'socks://...'` | `build_sam3_image_model()` 每次启动都先联网查新版本;`HF_HUB_OFFLINE=1` 对 huggingface_hub 2.4 的这条路径无效 | [`load_model`](../../sam3_service/server.py) 先用 `try_to_load_from_cache` 找本机权重,找到就传 `checkpoint_path`,启动完全不联网(代理坏了也能起);另外进程内把 `socks://` 改成 `http://`,第一次下载也不报错 |
| `ModuleNotFoundError: einops` 等 | 官方 `pyproject` 把它们列为可选 | 手动装 `einops pycocotools psutil scipy` |

## 测试素材

`recordings/head_20261112_015829/`:5 帧,640×400,深度有效约 90%。两张白桌,红/橙/黄/绿/蓝积木、笔、黑包、盒子;**画面里有人**,只留本机,不上传。

## 测试结果

2026-11-12,RTX 4070 Laptop 8GB,bf16,阈值 0.5,`recordings/head_20261112_015829/` 的 `color_04`、`color_00`(640×400)。结果图用 `sam3_test_image.py` 重新生成。

| 短语 | `color_04` | `color_00` | 判断 |
|---|---|---|---|
| `black bag` | 1 个 0.945 | 1 个 0.969 | 对 |
| `pen` | 1 个 0.871 | 2 个 0.887 / 0.531 | 主结果对;第二个是右桌上的小东西,误检 |
| `green block` | 2 个 0.848 / 0.824 | 2 个 0.863 / 0.840 | 对,两块都找到 |
| `yellow block` | 3 个 0.676 / 0.664 / 0.570 | 3 个 | 前两个是黄积木;第三个是右桌的黄色物体(不是积木,但确实是黄的) |
| `blue block` | 1 个 0.641 | 1 个 0.680 | 对,只有约 70 像素也找到了 |
| `orange block` | 2 个 0.750 / 0.660 | 2 个 0.723 / 0.566 | 第一个对;**第二个其实是红积木** |
| `red block` | 1 个 0.668 | 1 个 0.742 | 对 |
| `box` | 1 个 0.519(地上的灰色托盘) | 0 个 | 桌上装杯子的黑盒子没认出来 |
| `person` | 0.973 | 0.957 | 对 |

**速度**:模型加载 7 s(权重已缓存)。图像特征在第一个短语时才算,所以

- 一张图 + 1 个短语 ≈ **320 ms**(服务端),含传输约 345 ms;
- 每多一个短语 + **45 ms**;9 个短语共约 700 ms。
- 服务刚启动后的第一张图多 0.5 s 预热。

**显存**:常驻 3.4 GB,推理峰值 5.1 GB(`/health` 的 `gpu_mem_mb`),8 GB 卡够用,但同时再跑别的 GPU 程序要注意。

**结论与接入时要处理的问题**:

1. 积木这类小东西(70~270 像素)都能分出来,掩码贴合,能用。
2. **相近颜色会串**:红积木同时匹配 `orange block`(0.66)和 `red block`(0.67)。接入时同一张图多个短语的掩码要做**跨短语去重**:掩码重叠大的,只留分数最高的那个短语。
3. 同一短语可能多一个低分误检(0.53),接入时用 0.6 左右的阈值或让模型看分数挑。
4. 泛称(`box`)不稳,短语要具体(`black box with cups` 之类,待测)。
