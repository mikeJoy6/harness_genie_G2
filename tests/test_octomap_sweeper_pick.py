"""octomap_sweeper.pick_targets(代码在 ~/agi@g2_robot 的 perception 包里):哪些点云块要清。
空中的小块(不落地、没和大块连着)不管在不在视野里都清;桌子、桌上的东西、人、举在空中的手留着。

    source /opt/ros/humble/setup.bash
    python3 -m pytest tests/test_octomap_sweeper_pick.py -q
"""
import os
import sys

import numpy as np
import pytest

PKG = os.path.expanduser("~/agi@g2_robot/src/genie_robot_agibot_g2_perception")
if not os.path.isdir(PKG):
    pytest.skip(f"没有 {PKG}", allow_module_level=True)
pytest.importorskip("rclpy")
pytest.importorskip("moveit_msgs")
sys.path.insert(0, PKG)

from genie_robot_agibot_g2_perception.octomap_sweeper import pick_targets  # noqa: E402
from genie_robot_agibot_g2_perception.voxel_util import keys_of  # noqa: E402

RES = 0.03


def box(x, y, z, nx, ny, nz):
    """从 (x,y,z) 起 nx×ny×nz 个格的格中心(base_link,米)。"""
    i = np.stack(np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij"), -1).reshape(-1, 3)
    return np.floor(np.array([x, y, z]) / RES) * RES + (i + 0.5) * RES


SCENE = {
    "table": box(0.6, -0.4, 0.70, 20, 27, 1),      # 桌面,桌腿被挡住
    "mouse": box(0.81, -0.15, 0.75, 2, 2, 1),       # 桌上的鼠标,和桌面隔一格
    "person": box(1.5, 0.3, -0.03, 10, 10, 54),     # 站在地上的人
    "streak": box(0.5, 0.0, 1.2, 1, 1, 8),          # 空中一个假点 + 背面补全 25cm
    "hand": box(0.9, 0.0, 1.1, 8, 3, 3),            # 空中的手(身体没拍到)+ 背面补全
    "speck": box(0.3, -0.5, 0.2, 1, 1, 1),          # 低处单格
}


def cleared():
    centers = np.concatenate(list(SCENE.values()))
    uk, small, _, _ = pick_targets(centers, RES, 4, 40, 0.15, 2)
    result = {}
    for name, cells in SCENE.items():
        keys = np.unique(keys_of(cells, RES))
        result[name] = small[np.searchsorted(uk, keys)]
    return result


def test_floating_streak_and_speck_are_cleared():
    c = cleared()
    assert c["streak"].all() and c["speck"].all()


def test_real_things_are_kept():
    c = cleared()
    for name in ("table", "mouse", "person", "hand"):
        assert not c[name].any(), name
