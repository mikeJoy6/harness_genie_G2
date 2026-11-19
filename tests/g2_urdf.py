"""测试用:按末端展开 G2 的 URDF(和 hardware.launch.py / sim.launch.py 同一份 genie.urdf.xacro)。

    from g2_urdf import g2_urdf
    urdf = g2_urdf("suction", "suction")   # 两只手吸盘 = 以前的 G2_t2_crsB_ee.urdf(TCP 在法兰前 0.128m)
    urdf = g2_urdf("none", "none")         # 只有机身 = G2_t2_crsB.urdf

agi@g2_robot 没编译(install 里没有这两个包)时返回 None,测试自己 skip。
"""
import os
from pathlib import Path

INSTALL = Path.home() / "agi@g2_robot/install"
PACKAGES = ("genie_robot_description", "genie_moveit_config")


def g2_urdf(left_ee="suction", right_ee="suction"):
    xacro_path = INSTALL / "genie_moveit_config/share/genie_moveit_config/config/genie.urdf.xacro"
    if not xacro_path.exists():
        return None
    try:
        import xacro
    except ImportError:
        return None
    # xacro 里的 $(find genie_robot_description) 靠 AMENT_PREFIX_PATH 找包;没 source 工作空间时补上
    prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    for pkg in PACKAGES:
        prefix = str(INSTALL / pkg)
        if prefix not in prefixes:
            prefixes.insert(0, prefix)
    os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join(p for p in prefixes if p)
    doc = xacro.process_file(str(xacro_path), mappings={"left_ee": left_ee, "right_ee": right_ee})
    return doc.toxml()
