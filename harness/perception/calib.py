"""calib.py —— 头部 RGBD 标定（内置值 + 从 GDK 重读）。record_dataset 和 scene3d 共用。"""
import json
import time

# 标定：2026-11-03 用 GDK 从真机读出（agibot_gdk Camera.get_camera_intrinsic / TF.get_tf_from_sensor）。
#   约定 XToY = X 在 Y 里的位姿（见 genie_robot_agibot_g2_perception/config/head_camera.yaml 的验证）。
#   启动时若能 import agibot_gdk（先 source ~/.cache/agibot/app/env.sh）就重读一次覆盖这里的值。
CALIB = {
    "image_size": [640, 400],
    "color": {"fx": 304.5283508301, "fy": 304.4833374023, "cx": 318.1716003418, "cy": 204.41746521,
              "distortion_k1k2p1p2k3k4k5k6": [-0.0299324486, 0.0326635875, -0.000521665, -0.0001703959,
                                              -0.0113731129, 0.0, 0.0, 0.0]},
    "depth": {"fx": 309.3593139648, "fy": 309.3593139648, "cx": 319.0, "cy": 202.5,
              "distortion_k1k2p1p2k3k4k5k6": [0.0] * 8, "unit": "mm", "invalid": 0},
    "depth_in_color_xyz_qxyzw": [-0.023767, -0.000476537, 3.96624e-05, -0.000163058, -0.00101275, 0.0101925, 0.999948],
    "color_in_head_link3_xyz_qxyzw": [0.109538, 0.0245318, 0.0793004, 0.601425, -0.591243, 0.383415, -0.376447],
    "note": "XToY = X 在 Y 里的位姿；光学系 x 右 y 下 z 前；深度未对齐到彩色",
    "source": "built-in (GDK 2026-11-03)",
}


def read_gdk_calib():
    """能 import agibot_gdk 就从真机重读标定；失败返回 None。"""
    try:
        import agibot_gdk as g
    except ImportError:
        return None
    try:
        if g.gdk_init() != g.GDKRes.kSuccess:
            return None
        cam, tf = g.Camera(), g.TF()
        time.sleep(2.0)
        out = json.loads(json.dumps(CALIB))
        for key, ct in (("color", g.CameraType.kHeadColor), ("depth", g.CameraType.kHeadDepth)):
            k = cam.get_camera_intrinsic(ct)
            out[key].update(fx=k.intrinsic[0], fy=k.intrinsic[1], cx=k.intrinsic[2], cy=k.intrinsic[3],
                            distortion_k1k2p1p2k3k4k5k6=list(k.distortion))
        for key, et in (("depth_in_color_xyz_qxyzw", g.SensorExtrinsicType.kHeadDepthToHeadColor),
                        ("color_in_head_link3_xyz_qxyzw", g.SensorExtrinsicType.kHeadRGBDToHeadLink3)):
            t = tf.get_tf_from_sensor(et)
            out[key] = [t.translation.x, t.translation.y, t.translation.z,
                        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        out["source"] = f"GDK {time.strftime('%Y-%m-%d %H:%M:%S')}"
        return out
    except Exception as e:                       # 读不到就用内置值
        print(f"GDK 重读标定失败，用内置值：{e}", flush=True)
        return None
