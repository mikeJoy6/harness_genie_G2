"""harness/touch_keep_out.py:touch_object 最后一段的几何检查(纯 numpy,不需要 ROS)。

    python3 -m pytest tests/test_touch_keep_out.py -q
"""
import numpy as np

from harness.touch_keep_out import gripper_hits, outside_object, rests_on_surface

RES = 0.03
MOUSE = np.array([0.7971, -0.173, 0.7221])      # 2026-11-14 真机撞桌面那次的鼠标(可见顶面中位点)
TABLE_Z = 0.697                                  # 桌面那层格的中心高度


def grid(x0, x1, y0, y1, z):
    xs = np.arange(x0, x1, RES)
    ys = np.arange(y0, y1, RES)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    return np.stack([X.ravel(), Y.ravel(), np.full(X.size, z)], axis=1)


TABLE = grid(0.45, 1.10, -0.60, 0.20, TABLE_Z)
MOUSE_CELLS = grid(MOUSE[0] - 0.045, MOUSE[0] + 0.05, MOUSE[1] - 0.03, MOUSE[1] + 0.035, MOUSE[2] - 0.01)
SCENE = np.concatenate([TABLE, MOUSE_CELLS])
RADIUS, LENGTH, OBJECT_RADIUS = 0.04, 0.13, 0.08


def line(start, end, n=30):
    return np.linspace(start, end, n)


def test_horizontal_push_at_mouse_height_hits_table():
    """那次的最后一段:夹爪尖在鼠标顶面高度,从后面 30cm 水平推过来 —— 一路贴着桌面。"""
    blocked = outside_object(SCENE, MOUSE, OBJECT_RADIUS)
    tips = line(MOUSE + [-0.30, -0.03, 0.0], MOUSE)
    axis = (MOUSE - tips[0]) / np.linalg.norm(MOUSE - tips[0])
    assert gripper_hits(tips, [axis] * len(tips), blocked, RADIUS, LENGTH) > 0


def test_straight_down_onto_mouse_is_clear():
    """从上往下 30cm 到鼠标顶面:只碰到鼠标和它正下方的台面(都在放过的范围里)。"""
    blocked = outside_object(SCENE, MOUSE, OBJECT_RADIUS)
    tips = line(MOUSE + [0, 0, 0.30], MOUSE)
    axis = np.array([0.0, 0.0, -1.0])
    assert gripper_hits(tips, [axis] * len(tips), blocked, RADIUS, LENGTH) == 0


def test_down_with_15deg_tilt_is_still_clear():
    """朝下差 15°(容差上限)时夹爪尾巴歪出去,也不能擦到桌面。"""
    blocked = outside_object(SCENE, MOUSE, OBJECT_RADIUS)
    t = np.radians(15)
    axis = np.array([np.sin(t), 0.0, -np.cos(t)])
    tips = line(MOUSE - 0.30 * axis, MOUSE)
    assert gripper_hits(tips, [axis] * len(tips), blocked, RADIUS, LENGTH) == 0


def test_mouse_on_table_rests_on_surface():
    assert rests_on_surface(SCENE, MOUSE)


def test_hand_in_air_does_not_rest_on_surface():
    """举在空中的手(只有手掌那块格),四周下方是空的:允许从侧面碰(击掌)。"""
    palm = MOUSE + [0, 0, 0.40]
    hand = grid(palm[0] - 0.015, palm[0] + 0.02, palm[1] - 0.045, palm[1] + 0.05, palm[2])
    hand = np.concatenate([hand, hand + [0, 0, -RES], hand + [0, 0, RES]])
    assert not rests_on_surface(np.concatenate([hand, TABLE]), palm)


def test_empty_cloud():
    assert gripper_hits([MOUSE], [[0, 0, -1.0]], np.empty((0, 3)), RADIUS, LENGTH) == 0
    assert not rests_on_surface(np.empty((0, 3)), MOUSE)


def test_force_change_is_magnitude_of_force_part():
    from harness.backends.wrench_gdk import force_change_n
    zero = [0.0, 0.0, 9.4, 0.1, 0.0, 0.0]                       # 夹爪重量(零点)
    assert force_change_n([0.0, 0.0, 9.4, 3.0, 0.0, 0.0], zero) == 0.0      # 只看力,不看力矩
    assert abs(force_change_n([3.0, 0.0, 13.4, 0, 0, 0], zero) - 5.0) < 1e-9
