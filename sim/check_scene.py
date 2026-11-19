"""检查仿真场景(不用 ROS):机器人摆到初始姿态,渲染头部相机,用真机同一套内参 / 外参把深度反投影到 base_link,
看桌面、鼠标顶面算出来的高度对不对;再存三张图(彩色、深度、分割)到 sim/out/。

    ~/mujoco_sim/.venv/bin/python sim/check_scene.py
"""
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from scene import HEAD_CAMERA, build_model  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
# 分割图第二通道是对象类型。必须先转成 int:numpy 数组直接和 MuJoCo 的枚举比较会逐像素调用 Python,一帧 100ms
GEOM_TYPE = int(mujoco.mjtObj.mjOBJ_GEOM)


def pose_matrix(xyz, xyzw):
    x, y, z, w = xyzw
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def set_pose(model, data, pose):
    for joint, value in pose.items():
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if j < 0:
            continue
        data.qpos[model.jnt_qposadr[j]] = value
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "position_" + joint)
        if a >= 0:
            data.ctrl[a] = value


def main():
    model, info = build_model(Path(__file__).parent / "scenes/tabletop.yaml")
    data = mujoco.MjData(model)
    poses = yaml.safe_load((ROOT / "configs/poses.yaml").read_text(encoding="utf-8"))
    pose = poses[info["initial_pose"]]["joints"]
    set_pose(model, data, pose)
    for _ in range(1000):                       # 1 秒,让物体落稳
        mujoco.mj_step(model, data)
    worst = 0.0
    for joint, value in pose.items():
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if j >= 0:
            worst = max(worst, abs(data.qpos[model.jnt_qposadr[j]] - value))
    print(f"初始姿态保持:1 秒后关节最大偏差 {np.degrees(worst):.2f}°")

    c = HEAD_CAMERA
    renderer = mujoco.Renderer(model, height=c["height"], width=c["width"])
    renderer.update_scene(data, camera="head_depth")
    rgb = renderer.render()
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="head_depth")
    depth = renderer.render()
    renderer.disable_depth_rendering()
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="head_depth")
    seg = renderer.render()
    renderer.disable_segmentation_rendering()
    geom_id = seg[:, :, 0]
    objects = np.where(geom_id >= 0, info["geom_object"][np.clip(geom_id, 0, None)], 0)
    objects[seg[:, :, 1] != GEOM_TYPE] = 0

    # 和 harness/perception/scene3d.py 一样:像素 → 光学系 → head_link3 → base_link
    h3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "head_link3")
    T_base_head = np.eye(4)
    T_base_head[:3, :3] = data.xmat[h3].reshape(3, 3)
    T_base_head[:3, 3] = data.xpos[h3]
    T_head_cam = pose_matrix(c["pose_in_head_link3"][:3], c["pose_in_head_link3"][3:])
    v, u = np.mgrid[0:c["height"], 0:c["width"]]
    ray = np.dstack([(u - c["cx"]) / c["fx"], (v - c["cy"]) / c["fy"], np.ones_like(u, dtype=float)])
    P_cam = ray * depth[:, :, None]
    P = P_cam.reshape(-1, 3) @ (T_base_head @ T_head_cam)[:3, :3].T + (T_base_head @ T_head_cam)[:3, 3]
    P = P.reshape(c["height"], c["width"], 3)

    print("反投影检查(base_link z,应等于场景里的高度):")
    table_mask = np.zeros_like(objects, dtype=bool)
    tb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "table")
    table_mask[(seg[:, :, 1] == GEOM_TYPE) & (model.geom_bodyid[np.clip(geom_id, 0, None)] == tb)] = True
    top = table_mask & (np.abs(P[:, :, 2] - 0.69) < 0.05)
    print(f"  桌面:中位 z = {np.median(P[top][:, 2]):.4f}(应为 0.6900),像素 {top.sum()}")
    for o in info["objects"]:
        m = objects == o["index"]
        if m.sum() == 0:
            print(f"  {o['name']}:不在画面里")
            continue
        pts = P[m]
        body_pos = data.xpos[o["body_id"]]
        print(f"  {o['name']}:像素 {m.sum()},可见表面中位 {np.round(np.median(pts, axis=0), 4).tolist()},"
              f"物体中心 {np.round(body_pos, 4).tolist()}")

    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    cv2.imwrite(str(out / "head_color.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / "head_depth.png"), np.clip(depth * 1000, 0, 65535).astype(np.uint16))
    cv2.imwrite(str(out / "head_seg.png"), (objects * 40).astype(np.uint8))
    print(f"图存在 {out}/head_color.png、head_depth.png(16 位 mm)、head_seg.png")


if __name__ == "__main__":
    main()
