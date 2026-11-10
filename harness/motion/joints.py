"""joints.py —— 全身关节分组 = hardware.launch 起的 4 个 JTC（genie_moveit_config/config/ros2_controllers.yaml）。"""
GROUPS = {
    "body": ("body_group_controller",
             ["idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3", "idx04_body_joint4", "idx05_body_joint5"]),
    "head": ("head_group_controller", ["idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"]),
    "left_arm": ("left_arm_group_controller",
                 [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]),
    "right_arm": ("right_arm_group_controller",
                  [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]),
}
ALL_GROUPS = tuple(GROUPS)


def parse_groups(text):
    """"body,head" → ["body", "head"]；空 / "all" → 全部。不认识的组名抛 ValueError。"""
    if not text or text == "all":
        return list(ALL_GROUPS)
    groups = [g.strip() for g in text.split(",") if g.strip()]
    bad = [g for g in groups if g not in GROUPS]
    if bad:
        raise ValueError(f"不认识的关节组 {bad}，可选 {list(ALL_GROUPS)}")
    return groups
