# fixtures/ —— 测试数据(暂未使用)

## 这里干什么

以后用来放从真机录下来的数据,比如相机帧,让测试更接近真实情况。现在的测试用代码生成假图像,这里目前是空的。

## 和哪里联动

```text
真机录制的 RGBD → fixtures/ → tests/ 和 harness/backends/color_object.py 离线回放
```

## 示例

录一组头部 RGBD,之后可以拿来调红色阈值:

```bash
source ~/agi@g2_robot/env_g2.sh
ros2 run task_board_pose record_dataset --rate 2
# 输出在 ~/datasets/g2_head/<时间戳>/
```
