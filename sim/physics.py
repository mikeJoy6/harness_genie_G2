"""G2 仿真的物理进程:MuJoCo 4(Python 3.12,~/mujoco_sim/.venv),不碰 ROS。
ROS 那一侧由 sim/ros_bridge.py(系统 Python 3.10 + rclpy)负责,两者用 Unix 套接字通信(sim/link.py)。
为什么分开:MuJoCo 4 要 Python ≥3.11,ROS Humble 的 rclpy 只有 3.10。

不要直接运行,用 sim/launch/sim.launch.py 启动,见 sim/README.md。
单独跑(调试,没有 ROS 也能跑):MUJOCO_GL=egl ~/mujoco_sim/.venv/bin/python sim/physics.py --viewer

发给桥的(都是 link.send 的消息):
  hello    连上时一次:物体编号表、两只手的末端
  state    100Hz:22 个关节(腰 5、头 3、两臂 14,不含夹爪,和真机 /gdk/joint_state 一样)、两个末端位姿和六维力、头部位姿
  camera   15Hz:头部相机 JPEG + 16 位深度(毫米)+ 16 位真值分割图(每个像素是哪个物体,0 = 不是物体),同一帧
  photo    1Hz:拍照相机 1920x1536 JPEG
  suction  10Hz:装吸盘的手:开没开、吸住了哪个物体
桥发来的:
  targets          关节 / 夹爪目标位置(来自 /gdk/joint_control)
  reset_objects    物体放回初始位置(机器人不动)
  suction          {"arm", "on"}:开 / 关吸盘

六维力:末端(夹爪 / 吸盘)上所有接触力 + 末端自重 + 吸住的物体重量,换到 arm_*_end_link 系、对 end_link 原点取矩。
  真机 GDK 已减掉手臂重力、但末端重量会算成外力,harness 每次用前静止取零点,所以这里也带上末端重量。
吸盘:开着时,吸盘一碰到能动的物体就用焊接约束把它粘住(scene.py 预先建好、默认关);关掉就松开。
"""
import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
import link  # noqa: E402
from scene import HEAD_CAMERA, PHOTO_CAMERA, SIDE, build_model  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
# 分割图第二通道是对象类型。必须先转成 int:numpy 数组直接和 MuJoCo 的枚举比较会逐像素调用 Python,一帧 100ms
GEOM_TYPE = int(mujoco.mjtObj.mjOBJ_GEOM)

# 真机 /gdk/joint_state 里的 22 个关节(顺序照真机)
GDK_JOINTS = ([f"idx0{i}_body_joint{i}" for i in range(1, 6)]
              + [f"idx1{i}_head_joint{i}" for i in range(1, 4)]
              + [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
              + [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)])
GRIPPER_JOINTS = {"left": "idx31_gripper_l_inner_joint1", "right": "idx71_gripper_r_inner_joint1"}
LINK7 = {"left": "arm_l_link7", "right": "arm_r_link7"}
# URDF:arm_*_link7 → arm_*_end_link 是固定关节,沿 link7 的 x 走 0.09m,再转个向(两只手一样)
LINK7_TO_END_POS = np.array([0.09, 0.0, 0.0])
LINK7_TO_END_ROT = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])

PHYSICS_DT = 0.001
STATE_EVERY = 10          # 100Hz 发状态
CAMERA_EVERY = 66         # 约 15Hz 发头部相机
PHOTO_EVERY = 1000        # 1Hz 发拍照相机
SUCTION_EVERY = 100       # 10Hz 发吸盘状态
VIEWER_EVERY = 33         # 约 30Hz 刷新 MuJoCo 窗口
MAX_DEPTH_M = 6.0         # 更远的像素当“没测到”(真机深度相机也有量程),写 0


def rot_to_quat_xyzw(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))     # wxyz
    return [float(q[1]), float(q[2]), float(q[3]), float(q[0])]


def floats(values):
    return [float(v) for v in values]


class Physics:
    def __init__(self, scene_path, left_ee="omnipicker", right_ee="omnipicker", realtime=True, viewer=False):
        self.model, self.info = build_model(scene_path, left_ee, right_ee)
        self.data = mujoco.MjData(self.model)
        self.realtime = realtime
        m = self.model

        # 名字 → 下标
        self.qpos_adr = {}
        self.dof_adr = {}
        gripper_joints = [GRIPPER_JOINTS[arm] for arm, kind in self.info["end_effectors"].items() if kind == "omnipicker"]
        for name in GDK_JOINTS + gripper_joints:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j < 0:
                raise RuntimeError(f"模型里没有关节 {name}")
            self.qpos_adr[name] = m.jnt_qposadr[j]
            self.dof_adr[name] = m.jnt_dofadr[j]
        self.position_actuator = {}
        for name in self.qpos_adr:
            a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "position_" + name)
            if a < 0:
                raise RuntimeError(f"模型里没有位置驱动器 position_{name}")
            self.position_actuator[name] = a
        self.link7_body = {arm: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, LINK7[arm]) for arm in LINK7}
        self.head_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "head_link3")
        self.ee_body_ids = {}
        self.ee_mass = {}
        for arm, names in self.info["gripper_bodies"].items():
            ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in names]
            self.ee_body_ids[arm] = set(ids)
            self.ee_mass[arm] = float(sum(m.body_mass[i] for i in ids))
        self.object_by_body = {o["body_id"]: o for o in self.info["objects"] if not o["fixed"]}

        # 初始姿态:configs/poses.yaml 里的位姿,夹爪闭合
        poses = yaml.safe_load((ROOT / "configs/poses.yaml").read_text(encoding="utf-8"))
        pose = poses[self.info["initial_pose"]]["joints"]
        self.lock = threading.Lock()
        self.targets = {}
        for name in self.qpos_adr:
            value = float(pose.get(name, 0.0))
            self.targets[name] = value
            self.data.qpos[self.qpos_adr[name]] = value
        mujoco.mj_forward(m, self.data)
        self.initial_qpos = self.data.qpos.copy()
        self.commands_received = 0
        self.reset_requested = False
        self.suction_on = {arm: False for arm in self.info["suction_welds"]}
        self.suction_attached = {arm: None for arm in self.info["suction_welds"]}    # 吸住的物体名

        # 渲染(必须在主线程建,EGL 上下文跟着线程走)。彩色 / 深度 / 分割各一个渲染器、模式固定:
        #   同一个渲染器来回切模式一帧要 ~100ms,分开后三张一共 ~8ms
        c = HEAD_CAMERA
        self.color_renderer = mujoco.Renderer(m, height=c["height"], width=c["width"])
        self.depth_renderer = mujoco.Renderer(m, height=c["height"], width=c["width"])
        self.depth_renderer.enable_depth_rendering()
        self.seg_renderer = mujoco.Renderer(m, height=c["height"], width=c["width"])
        self.seg_renderer.enable_segmentation_rendering()
        self.photo_renderer = mujoco.Renderer(m, height=PHOTO_CAMERA["height"], width=PHOTO_CAMERA["width"])

        # MuJoCo 窗口(看物理效果;关掉窗口仿真照常跑)。窗口走 GLFW,离屏渲染走 EGL,互不影响
        self.viewer = None
        if viewer:
            self.viewer = mujoco.viewer.launch_passive(m, self.data, show_left_ui=False, show_right_ui=False)

        # 和 ROS 桥的连接:后台线程等桥连上、收命令;发送只在主循环里做
        self.conn = None
        self.listener = link.listen()
        threading.Thread(target=self.serve, daemon=True).start()

    # ---------------- 和桥的连接(后台线程) ----------------
    def serve(self):
        while True:
            conn = self.listener.accept()
            hello = {"type": "hello", "end_effectors": self.info["end_effectors"],
                     "objects": [{"index": o["index"], "name": o["name"], "labels": o["labels"]}
                                 for o in self.info["objects"]]}
            try:
                link.send(conn, hello)
            except OSError:
                continue
            self.conn = conn
            print("[physics] ROS 桥已连上", flush=True)
            try:
                while True:
                    header, _ = link.recv(conn)
                    self.handle(header)
            except (EOFError, OSError):
                print("[physics] ROS 桥断开,等它重连", flush=True)
                self.conn = None

    def handle(self, msg):
        kind = msg.get("type")
        with self.lock:
            if kind == "targets":
                for name, value in msg["targets"].items():
                    if name in self.targets and np.isfinite(value):
                        self.targets[name] = float(value)
                self.commands_received += 1
            elif kind == "reset_objects":
                self.reset_requested = True
            elif kind == "suction":
                arm = msg.get("arm")
                if arm in self.suction_on:
                    self.suction_on[arm] = bool(msg.get("on"))

    def send(self, header, blobs=()):
        conn = self.conn
        if conn is None:
            return
        try:
            link.send(conn, header, blobs)
        except OSError:
            self.conn = None

    # ---------------- 主循环 ----------------
    def run(self):
        m, d = self.model, self.data
        step = 0
        start = time.monotonic()
        last_report = start
        print(f"[physics] MuJoCo {mujoco.__version__},末端 {self.info['end_effectors']},"
              f"{len(self.info['objects'])} 个物体,等 ROS 桥连 {link.SOCKET_PATH}", flush=True)
        while True:
            with self.lock:
                for name, value in self.targets.items():
                    d.ctrl[self.position_actuator[name]] = value
                if self.reset_requested:
                    self.reset_objects()
                    self.reset_requested = False
                self.update_suction()
            mujoco.mj_step(m, d)
            step += 1
            if step % STATE_EVERY == 0:
                self.send(self.state_message())
            if step % CAMERA_EVERY == 0 and self.conn is not None:
                self.send_head_camera()
            if step % PHOTO_EVERY == 0 and self.conn is not None:
                self.send_photo()
            if step % SUCTION_EVERY == 0 and self.suction_on:
                self.send({"type": "suction", "arms": {arm: {"on": self.suction_on[arm],
                                                              "attached": self.suction_attached[arm]}
                                                        for arm in self.suction_on}})
            if self.viewer is not None and step % VIEWER_EVERY == 0:
                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    self.viewer = None          # 窗口被关了,仿真继续
            if self.realtime:
                ahead = start + step * PHYSICS_DT - time.monotonic()
                if ahead > 0:
                    time.sleep(ahead)
            now = time.monotonic()
            if now - last_report > 10.0:
                lag = now - (start + step * PHYSICS_DT)
                print(f"[physics] 仿真 {step * PHYSICS_DT:.0f}s,落后真实时间 {lag:.2f}s,"
                      f"收到关节指令 {self.commands_received} 次{'' if self.conn else ',ROS 桥没连'}", flush=True)
                last_report = now

    def reset_objects(self):
        """物体放回初始位置、速度清零,吸住的松开;机器人关节不动。"""
        m, d = self.model, self.data
        for arm in self.suction_attached:
            self.release(arm)
        for o in self.info["objects"]:
            for j in range(m.njnt):
                if m.jnt_bodyid[j] == o["body_id"] and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                    a = m.jnt_qposadr[j]
                    d.qpos[a:a + 7] = self.initial_qpos[a:a + 7]
                    v = m.jnt_dofadr[j]
                    d.qvel[v:v + 6] = 0.0
        mujoco.mj_forward(m, d)

    # ---------------- 吸盘 ----------------
    def update_suction(self):
        for arm, on in self.suction_on.items():
            if not on:
                if self.suction_attached[arm] is not None:
                    self.release(arm)
                continue
            if self.suction_attached[arm] is None:
                touching = self.touching_object(arm)
                if touching is not None:
                    self.attach(arm, touching)

    def touching_object(self, arm):
        """吸盘贴着(5mm 以内,scene.py 的 SUCTION_GRAB_DISTANCE)的能动物体(第一个),没有返回 None。"""
        m, d = self.model, self.data
        ids = self.ee_body_ids[arm]
        for i in range(d.ncon):
            c = d.contact[i]
            b1 = m.geom_bodyid[c.geom1]
            b2 = m.geom_bodyid[c.geom2]
            if b1 in ids and b2 in self.object_by_body:
                return self.object_by_body[b2]
            if b2 in ids and b1 in self.object_by_body:
                return self.object_by_body[b1]
        return None

    def attach(self, arm, obj):
        """把物体按当前的相对位姿焊在吸盘上。"""
        m, d = self.model, self.data
        eq = self.info["suction_welds"][arm][obj["name"]]
        cup = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"suction_{SIDE[arm]}_link")
        R = d.xmat[cup].reshape(3, 3)
        m.eq_data[eq, 3:6] = R.T @ (d.xpos[obj["body_id"]] - d.xpos[cup])
        q_inv = np.zeros(4)
        q_rel = np.zeros(4)
        mujoco.mju_negQuat(q_inv, d.xquat[cup])
        mujoco.mju_mulQuat(q_rel, q_inv, d.xquat[obj["body_id"]])
        m.eq_data[eq, 6:10] = q_rel
        d.eq_active[eq] = 1
        self.suction_attached[arm] = obj["name"]
        print(f"[physics] {arm} 吸盘吸住 {obj['name']}", flush=True)

    def release(self, arm):
        name = self.suction_attached[arm]
        if name is None:
            return
        self.data.eq_active[self.info["suction_welds"][arm][name]] = 0
        self.suction_attached[arm] = None
        print(f"[physics] {arm} 吸盘松开 {name}", flush=True)

    # ---------------- 状态 ----------------
    def end_pose(self, arm):
        """arm_*_end_link 在 base_link(= 世界)里的 (位置, 旋转)。"""
        b = self.link7_body[arm]
        R7 = self.data.xmat[b].reshape(3, 3)
        p = self.data.xpos[b] + R7 @ LINK7_TO_END_POS
        return p, R7 @ LINK7_TO_END_ROT

    def end_wrench(self, arm):
        """末端受的外力(接触 + 末端自重 + 吸住的物体重量),end_link 系,对 end_link 原点取矩。"""
        m, d = self.model, self.data
        p_end, R_end = self.end_pose(arm)
        mass = self.ee_mass.get(arm, 0.0)
        held = self.suction_attached.get(arm)
        if held is not None:
            body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, held)
            mass += float(m.body_mass[body])
        force = np.array([0.0, 0.0, -9.81 * mass])
        torque = np.zeros(3)
        ids = self.ee_body_ids.get(arm, set())
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1 = m.geom_bodyid[c.geom1]
            b2 = m.geom_bodyid[c.geom2]
            if (b1 in ids) == (b2 in ids):          # 不涉及末端,或末端自己碰自己
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            frame = c.frame.reshape(3, 3)           # 行:法向、两个切向
            f_world = frame.T @ f6[:3]              # geom1 施加给 geom2 的力
            if b1 in ids:
                f_world = -f_world
            force += f_world
            torque += np.cross(c.pos - p_end, f_world)
        return R_end.T @ force, R_end.T @ torque

    def state_message(self):
        d = self.data
        ends = {}
        for arm in ("left", "right"):
            p, R = self.end_pose(arm)
            f, t = self.end_wrench(arm)
            ends[arm] = {"pos": floats(p), "quat_xyzw": rot_to_quat_xyzw(R), "force": floats(f), "torque": floats(t)}
        h = self.head_body
        return {"type": "state", "names": GDK_JOINTS,
                "position": [float(d.qpos[self.qpos_adr[n]]) for n in GDK_JOINTS],
                "velocity": [float(d.qvel[self.dof_adr[n]]) for n in GDK_JOINTS],
                "effort": [float(d.qfrc_actuator[self.dof_adr[n]]) for n in GDK_JOINTS],
                "ends": ends, "head": {"pos": floats(d.xpos[h]), "quat_xyzw": rot_to_quat_xyzw(d.xmat[h])}}

    def send_head_camera(self):
        images = []
        for r in (self.color_renderer, self.depth_renderer, self.seg_renderer):
            r.update_scene(self.data, camera="head_depth")
            images.append(r.render())
        rgb, depth, seg = images
        ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return
        depth_mm = np.round(depth * 1000.0)
        depth_mm[(depth <= 0) | (depth > MAX_DEPTH_M)] = 0
        geom_id = seg[:, :, 0]
        is_geom = (seg[:, :, 1] == GEOM_TYPE) & (geom_id >= 0)
        objects = np.zeros(geom_id.shape, dtype=np.uint16)
        objects[is_geom] = self.info["geom_object"][geom_id[is_geom]]
        h, w = depth.shape
        self.send({"type": "camera", "width": w, "height": h},
                  [jpg.tobytes(), depth_mm.astype("<u2").tobytes(), objects.astype("<u2").tobytes()])

    def send_photo(self):
        r = self.photo_renderer
        r.update_scene(self.data, camera="head_photo")
        ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            self.send({"type": "photo"}, [jpg.tobytes()])


def main():
    parser = argparse.ArgumentParser(description="G2 MuJoCo 仿真的物理进程(ROS 由 sim/ros_bridge.py 负责)")
    parser.add_argument("--scene", default=str(Path(__file__).parent / "scenes/tabletop.yaml"))
    parser.add_argument("--left-ee", default="omnipicker", help="左手末端:none / suction / omnipicker(和 sim.launch.py 一致)")
    parser.add_argument("--right-ee", default="omnipicker", help="右手末端:none / suction / omnipicker")
    parser.add_argument("--no-realtime", action="store_true", help="不按真实时间走,能多快多快(调试用)")
    parser.add_argument("--viewer", action="store_true", help="开 MuJoCo 窗口看物理效果")
    args = parser.parse_args()
    sim = Physics(args.scene, args.left_ee, args.right_ee, realtime=not args.no_realtime, viewer=args.viewer)
    try:
        sim.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
