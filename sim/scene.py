"""把 AgiBot 官方的 G2 MuJoCo 模型改成仿真要用的样子,再按场景文件摆上桌子和物体。

只用 MuJoCo(~/mujoco_sim/.venv),不碰 ROS,可以单独跑来检查:
    ~/mujoco_sim/.venv/bin/python sim/scene.py sim/scenes/tabletop.yaml [左末端] [右末端]   # 打印物体表

对官方模型做的改动(都在 build_model 里,按顺序):
  1. 底座固定:删掉 base_link 的自由关节,base_link 放在世界原点 → 世界坐标 = base_link 坐标,场景文件直接写 base_link 坐标。
  2. 夹爪内部不互相碰撞:官方模型里 omnipicker 的连杆和底座本来就互相穿透(最深 7.9mm),会产生不存在的内力。
  3. 接触参数:椭圆摩擦锥 + impratio,夹东西不容易滑。
  4. 头部 RGB-D 相机:位姿和内参用真机标定值(agi@g2_robot 的 head_camera.yaml),
     另加一个拍照用的大分辨率相机(真机是双目左目,外参没标定,先放在同一位置、视场角放大)。
  5. 桌子、物体、人(按场景文件)。
  6. 末端按 left_ee / right_ee(和 genie.urdf.xacro 同一组参数):官方模型两只手都是 omnipicker,
     none 就删掉那只手的夹爪(连同约束、驱动器);suction 删掉夹爪、换成和 suction.xacro 一样的长方体。
"""
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

OFFICIAL_MJCF = Path.home() / "agi@g2_robot/reference/G2_description/genie_robot_description/mjcf/G2_t2v2_crsB_omnipicker_flat.xml"

# 真机头部深度相机(和彩色逐像素对齐),agi@g2_robot/src/genie_robot_agibot_g2_perception/config/head_camera.yaml
HEAD_CAMERA = {
    "width": 640, "height": 400,
    "fx": 309.359, "fy": 309.359, "cx": 319.0, "cy": 202.5,
    # 光学系(z 朝前、y 朝下)在 head_link3 里的位姿 [x y z qx qy qz qw]
    "pose_in_head_link3": [0.109538, 0.0245318, 0.0793004, 0.601425, -0.591243, 0.383415, -0.376447],
}
PHOTO_CAMERA = {"width": 1920, "height": 1536, "fovy_deg": 90.0}   # 真机 head_stereo_left 1920x1536,广角;视场角是估的

GRIPPER_BODIES = {
    "left": ["gripper_l_base_link", "gripper_l_inner_link1", "gripper_l_inner_link2", "gripper_l_inner_link3",
             "gripper_l_inner_link4", "gripper_l_outer_link1", "gripper_l_outer_link2", "gripper_l_outer_link3",
             "gripper_l_outer_link4"],
    "right": ["gripper_r_base_link", "gripper_r_inner_link1", "gripper_r_inner_link2", "gripper_r_inner_link3",
              "gripper_r_inner_link4", "gripper_r_outer_link1", "gripper_r_outer_link2", "gripper_r_outer_link3",
              "gripper_r_outer_link4"],
}
FLOOR_Z = -0.05          # 官方模型 base_link 离地 5cm
EE_TYPES = ("none", "suction", "omnipicker")
SIDE = {"left": "l", "right": "r"}
# 吸盘长方体(agi@g2_robot/src/genie_robot_description/urdf/end_effectors/suction.xacro):法兰系 x 11.6cm、y 10.2cm,
#   沿法兰 z 伸出 12.8cm。法兰 = link7 沿 x 走 0.09m、再转向(法兰 z = link7 x,法兰 y = -link7 y,法兰 x = link7 z),
#   所以在 link7 系里:中心 (0.09 + 0.064, 0, 0),半尺寸 x 0.064、y 0.051、z 0.058。
SUCTION_CENTER_IN_LINK7 = [0.09 + 0.064, 0.0, 0.0]
SUCTION_HALF_IN_LINK7 = [0.064, 0.051, 0.058]
SUCTION_MASS = 0.3       # 估的
# 仿真吸盘:每只吸盘和每个能动的物体之间预先建一个焊接约束(默认关),吸盘开、贴着哪个物体就开哪个(sim/physics.py)。
#   默认约束太软(拖着快速移动时偏 1cm),调硬:时间常数 4ms(物理步长 1ms,不能小于 2 步)
SUCTION_WELD_SOLREF = [0.004, 1.0]
SUCTION_GRAB_DISTANCE = 0.005    # 吸盘端面离物体这么近就能吸住

# 碰撞位(geom 的 contype / conaffinity;A 的 contype & B 的 conaffinity 或反过来不为 0 才碰):
#   官方模型里手臂、腰、头的网格 contype=0(根本不碰撞),只有夹爪和轮子开了 —— 那样手臂会穿过桌子。这里重新分:
#   环境(地面、桌子、物体、人)  contype 1, conaffinity 1|2|4 → 和一切碰
#   机器人连杆(手臂、腰、头)      contype 2, conaffinity 1     → 只和环境碰;连杆之间不碰(自碰撞 MoveIt 管,关节处网格本来就重叠)
#   夹爪                          contype 4, conaffinity 1     → 只和环境碰,不和自己的手臂碰
#   轮子                          关掉(底座固定了)
ENV_BITS = (1, 7)
LINK_BITS = (2, 1)
GRIPPER_BITS = (4, 1)
ROBOT_LINK_PREFIXES = ("base_link", "body_link", "head_link", "arm_l_link", "arm_r_link")


def quat_wxyz(xyzw):
    return [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]


def quat_mul(a, b):
    """wxyz 四元数乘法。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def optical_to_mujoco(q_optical_wxyz):
    """光学系(z 朝前、y 朝下)→ MuJoCo 相机系(朝 -z 看、y 朝上):绕自身 x 轴转 180°。"""
    return quat_mul(q_optical_wxyz, [0.0, 1.0, 0.0, 0.0])


def yaw_quat(deg):
    half = math.radians(deg) / 2
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def add_head_cameras(spec):
    head = spec.body("head_link3")
    c = HEAD_CAMERA
    cam = head.add_camera()
    cam.name = "head_depth"
    cam.pos = c["pose_in_head_link3"][:3]
    cam.quat = optical_to_mujoco(quat_wxyz(c["pose_in_head_link3"][3:]))
    cam.resolution = [c["width"], c["height"]]
    cam.sensor_size = [c["width"] * 1e-5, c["height"] * 1e-5]      # 每像素 10µm,只是为了让下面按像素填内参
    cam.focal_pixel = [c["fx"], c["fy"]]
    # MuJoCo 的主点是“离画面中心多少像素”,而且横纵两个方向都和图像坐标(u 朝右、v 朝下)反号。
    #   2026-11-19 用小球投影实测:按同号填,斜看桌面时深度差 1.3cm、桌面高 1cm;反号后投影误差 < 0.2 像素。
    cam.principal_pixel = [(c["width"] - 1) / 2 - c["cx"], (c["height"] - 1) / 2 - c["cy"]]

    p = PHOTO_CAMERA
    photo = head.add_camera()
    photo.name = "head_photo"
    photo.pos = c["pose_in_head_link3"][:3]
    photo.quat = optical_to_mujoco(quat_wxyz(c["pose_in_head_link3"][3:]))
    photo.fovy = p["fovy_deg"]
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, p["width"])
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, p["height"])


def add_geom(body, shape, size, rgba, pos=(0, 0, 0), quat=(1, 0, 0, 0), friction=(1.0, 0.005, 0.0001), condim=4,
             fromto=None):
    """size 用“看得懂”的全尺寸:box [长 宽 高];cylinder [半径 高];sphere [半径];capsule [半径 长]。
    capsule / cylinder 给了 fromto(两端点,相对物体中心)时,size 只要半径。"""
    g = body.add_geom()
    if fromto is not None:
        g.type = mujoco.mjtGeom.mjGEOM_CAPSULE if shape == "capsule" else mujoco.mjtGeom.mjGEOM_CYLINDER
        g.size = [size[0], 0, 0]
        g.fromto = list(fromto)
    elif shape == "box":
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [size[0] / 2, size[1] / 2, size[2] / 2]
    elif shape == "cylinder":
        g.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        g.size = [size[0], size[1] / 2, 0]
    elif shape == "sphere":
        g.type = mujoco.mjtGeom.mjGEOM_SPHERE
        g.size = [size[0], 0, 0]
    elif shape == "capsule":
        g.type = mujoco.mjtGeom.mjGEOM_CAPSULE
        g.size = [size[0], size[1] / 2, 0]
    else:
        raise ValueError(f"不认识的形状 {shape}")
    g.rgba = list(rgba)
    g.pos = list(pos)
    g.quat = list(quat)
    g.friction = list(friction)
    g.condim = condim
    g.contype, g.conaffinity = ENV_BITS
    return g


def half_height(shape, size):
    if shape == "box":
        return size[2] / 2
    if shape == "cylinder":
        return size[1] / 2
    if shape == "sphere":
        return size[0]
    if shape == "capsule":
        return size[0]
    raise ValueError(shape)


def add_table(spec, table):
    cx, cy = table["center"]
    sx, sy = table["size"]
    top = table["top_z"]
    thick = 0.04
    body = spec.worldbody.add_body()
    body.name = "table"
    add_geom(body, "box", [sx, sy, thick], table.get("rgba", [0.85, 0.85, 0.82, 1]), pos=[cx, cy, top - thick / 2])
    legs = top - thick - FLOOR_Z
    for dx in (-1, 1):
        for dy in (-1, 1):
            add_geom(body, "box", [0.04, 0.04, legs], [0.5, 0.5, 0.5, 1],
                     pos=[cx + dx * (sx / 2 - 0.04), cy + dy * (sy / 2 - 0.04), FLOOR_Z + legs / 2])


def add_object(spec, obj, table_top):
    """一个物体 = 一个 body。parts 有多个几何体时组合起来(人)。fixed: true 的不会动。"""
    parts = obj.get("parts") or [{"shape": obj["shape"], "size": obj["size"], "rgba": obj.get("rgba", [0.5, 0.5, 0.5, 1])}]
    pos = list(obj["pos"])
    if len(pos) == 2:                                    # 只给 x y:放在桌面上
        pos.append(table_top + half_height(parts[0]["shape"], parts[0]["size"]) + 0.001)
    body = spec.worldbody.add_body()
    body.name = obj["name"]
    body.pos = pos
    body.quat = yaw_quat(obj.get("yaw_deg", 0.0))
    if not obj.get("fixed", False):
        body.add_freejoint()
    for part in parts:
        g = add_geom(body, part["shape"], part["size"], part.get("rgba", [0.5, 0.5, 0.5, 1]),
                     pos=part.get("pos", [0, 0, 0]), quat=part.get("quat", [1, 0, 0, 0]), fromto=part.get("fromto"))
        g.mass = obj.get("mass", 0.1) / len(parts)      # 直接给质量,不按体积 × 默认密度算
    return body


def remove_gripper(spec, side):
    """删掉官方模型里一只手的 omnipicker:约束(mimic)、驱动器、夹爪 body 整棵子树。"""
    mark = f"_gripper_{side}_"
    for eq in list(spec.equalities):
        if mark in eq.name1 or mark in eq.name2:
            spec.delete(eq)
    for act in list(spec.actuators):
        if mark in act.name:
            spec.delete(act)
    spec.delete(spec.body(f"gripper_{side}_base_link"))


def add_suction(spec, side):
    body = spec.body(f"arm_{side}_link7").add_body()
    body.name = f"suction_{side}_link"
    g = body.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_BOX
    g.pos = SUCTION_CENTER_IN_LINK7
    g.size = SUCTION_HALF_IN_LINK7
    g.rgba = [0.1, 0.1, 0.1, 1]
    g.mass = SUCTION_MASS
    g.condim = 4
    g.friction = [1.0, 0.005, 0.0001]
    g.contype, g.conaffinity = GRIPPER_BITS
    # 离物体 5mm 以内就算“贴上”(能吸住):margin = gap = 5mm → 这个距离内检测到接触、但不产生力(不会把东西推开)。
    #   2026-11-19:要求真接触时,touch 被力控停下后手臂微微回弹,吸盘离开积木零点几毫米,开吸没吸住;真吸盘的皮碗有弹性
    g.margin = SUCTION_GRAB_DISTANCE
    g.gap = SUCTION_GRAB_DISTANCE


def build_model(scene_path, left_ee="omnipicker", right_ee="omnipicker", mjcf_path=OFFICIAL_MJCF):
    """返回 (model, info)。info["objects"]:[{"index", "name", "labels"}],index 从 1 开始(分割图里 0 = 不是物体);
    info["geom_object"]:每个 geom 属于哪个物体的 index;info["end_effectors"]:{arm: 末端类型};
    info["gripper_bodies"]:{arm: [末端的 body 名]}(六维力算这些 body 上的接触)。"""
    ee = {"left": left_ee, "right": right_ee}
    for arm, kind in ee.items():
        if kind not in EE_TYPES:
            raise ValueError(f"{arm}_ee = {kind},只能是 {EE_TYPES}")
    scene = yaml.safe_load(Path(scene_path).read_text(encoding="utf-8"))
    spec = mujoco.MjSpec.from_file(str(mjcf_path))

    # 1. 底座固定在世界原点
    spec.delete(spec.joint("fixed_base_joint"))
    spec.body("base_link").pos = [0, 0, 0]
    floor = spec.geom("floor")
    floor.pos = [0, 0, FLOOR_Z]
    floor.contype, floor.conaffinity = ENV_BITS

    # 2. 碰撞位(见文件头):机器人连杆只和环境碰,夹爪只和环境碰,轮子不碰
    gripper_names = GRIPPER_BODIES["left"] + GRIPPER_BODIES["right"]
    for body in spec.bodies:
        for g in body.geoms:
            if body.name in gripper_names:
                if g.contype or g.conaffinity:            # 只改碰撞用的几何体,纯显示的保持不碰
                    g.contype, g.conaffinity = GRIPPER_BITS
            elif body.name.startswith("chassis_"):
                g.contype, g.conaffinity = 0, 0
            elif body.name.startswith(ROBOT_LINK_PREFIXES) and g.type == mujoco.mjtGeom.mjGEOM_MESH:
                g.contype, g.conaffinity = LINK_BITS

    # 末端:官方模型两只手都是 omnipicker,不是 omnipicker 的手删掉,suction 再装上长方体
    ee_bodies = {}
    for arm, kind in ee.items():
        side = SIDE[arm]
        if kind == "omnipicker":
            ee_bodies[arm] = list(GRIPPER_BODIES[arm])
            continue
        remove_gripper(spec, side)
        ee_bodies[arm] = []
        if kind == "suction":
            add_suction(spec, side)
            ee_bodies[arm] = [f"suction_{side}_link"]

    # 夹爪内部互不碰撞(每只手 9 个 body 两两排除;碰撞位已经让它们互不碰,这里再保险一次)
    for arm, names in GRIPPER_BODIES.items():
        if ee[arm] != "omnipicker":
            continue
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                e = spec.add_exclude()
                e.bodyname1 = names[i]
                e.bodyname2 = names[j]
        for name in names:                               # 指头接触用椭圆锥 + 扭转摩擦
            for g in spec.body(name).geoms:
                g.condim = 4
                g.friction = [1.2, 0.01, 0.0001]

    # 3. 接触
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0

    # 4. 相机 + 灯
    add_head_cameras(spec)
    light = spec.worldbody.add_light()
    light.pos = [0.8, 0.0, 2.5]
    light.dir = [0, 0, -1]
    light.diffuse = [0.7, 0.7, 0.7]
    light.castshadow = True

    # 5. 桌子、物体
    table = scene["table"]
    add_table(spec, table)
    objects = []
    for k, obj in enumerate(scene["objects"]):
        add_object(spec, obj, table["top_z"])
        objects.append({"index": k + 1, "name": obj["name"], "labels": [s.lower() for s in obj["labels"]],
                        "fixed": bool(obj.get("fixed", False))})

    # 吸盘的焊接约束(默认关)
    weld_names = {}
    for arm, kind in ee.items():
        if kind != "suction":
            continue
        weld_names[arm] = {}
        for o in objects:
            if o["fixed"]:
                continue
            eq = spec.add_equality()
            eq.type = mujoco.mjtEq.mjEQ_WELD
            eq.objtype = mujoco.mjtObj.mjOBJ_BODY
            eq.name = f"suction_{SIDE[arm]}__{o['name']}"
            eq.name1 = f"suction_{SIDE[arm]}_link"
            eq.name2 = o["name"]
            eq.active = False
            eq.solref = SUCTION_WELD_SOLREF
            weld_names[arm][o["name"]] = eq.name

    model = spec.compile()
    geom_object = np.zeros(model.ngeom, dtype=np.int32)
    for o in objects:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, o["name"])
        o["body_id"] = body_id
        geom_object[model.geom_bodyid == body_id] = o["index"]
    suction_welds = {}
    for arm, names in weld_names.items():
        suction_welds[arm] = {obj: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, name) for obj, name in names.items()}
    info = {"objects": objects, "geom_object": geom_object, "initial_pose": scene.get("initial_pose", "init"),
            "gripper_bodies": ee_bodies, "end_effectors": ee,
            "suction_welds": suction_welds}          # {arm: {物体名: equality id}},只有装吸盘的手
    return model, info


def main():
    scene_path = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).parent / "scenes/tabletop.yaml")
    left_ee = sys.argv[2] if len(sys.argv) > 2 else "omnipicker"
    right_ee = sys.argv[3] if len(sys.argv) > 3 else "omnipicker"
    model, info = build_model(scene_path, left_ee, right_ee)
    print(f"末端 {info['end_effectors']}  nq={model.nq} nu={model.nu} nbody={model.nbody} ngeom={model.ngeom}")
    for o in info["objects"]:
        print(f"  {o['index']:2d} {o['name']:12s} {o['labels']}{'  (固定)' if o['fixed'] else ''}")


if __name__ == "__main__":
    main()
