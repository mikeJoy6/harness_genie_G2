"""打通 ROS:一步一步读出机器人的数据,每一步都打印出来。

    python3 learning/stage2_ros/check_ros.py                    全部 5 步
    python3 learning/stage2_ros/check_ros.py --step 3           只跑第 3 步
    python3 learning/stage2_ros/check_ros.py --link arm_r_end_link   换一个 link

只读,不会让机器人动。
"""
import argparse
import math

import numpy as np

import config
import kinematics
from ros_io import RosIO


def step1_joints(ros):
    print("\n[1] 关节角(/joint_states)")
    print(f"    一共 {len(ros.joints())} 个关节,数据是 {ros.joints_age_s():.2f} 秒前的")
    for group, names in config.JOINT_GROUPS.items():
        values = ros.joints(names)
        degs = "  ".join(f"{math.degrees(v):7.2f}" for v in values.values())
        print(f"    {group:<10}(度) {degs}")


def step2_urdf(ros, link):
    print("\n[2] 机器人结构(/robot_description)")
    robot = kinematics.load_urdf(ros.urdf_xml())
    print(f"    URDF 长 {len(ros.urdf_xml())} 个字符,{len(robot.links)} 个 link,{len(robot.joints)} 个 joint")
    print(f"    {config.BASE_FRAME} → {link} 经过:")
    for j in kinematics.chain_joints(robot, config.BASE_FRAME, link):
        print(f"      {j.name:<24} {j.type}")


def step3_tf(ros, link):
    print(f"\n[3] TF:{link} 在 {config.BASE_FRAME} 下的位置")
    xyz, quat = ros.lookup(config.BASE_FRAME, link)
    print(f"    xyz  = {np.round(xyz, 4).tolist()} 米")
    print(f"    四元数 = {np.round(quat, 4).tolist()}  (x, y, z, w)")


def step4_fk_vs_tf(ros, link):
    print("\n[4] 自己算 FK,和 TF 对比")
    robot = kinematics.load_urdf(ros.urdf_xml())
    # 两个数据要尽量同一时刻取,否则手臂在动时会有差
    joints = ros.joints()
    xyz, quat = ros.lookup(config.BASE_FRAME, link)
    T = kinematics.fk(robot, config.BASE_FRAME, link, joints)
    pos_err_mm = np.linalg.norm(T[:3, 3] - xyz) * 1000
    rot_err = kinematics.rotation_diff_deg(T[:3, :3], kinematics.quat_to_matrix(quat))
    print(f"    自己算 xyz = {np.round(T[:3, 3], 4).tolist()}")
    print(f"    TF     xyz = {np.round(xyz, 4).tolist()}")
    print(f"    差 {pos_err_mm:.2f} mm,{rot_err:.3f}°  (手臂在动时会有几毫米,静止时应接近 0)")


def step5_collision(ros):
    print("\n[5] MoveIt 碰撞检查(/check_state_validity)")
    now = ros.check_collision()
    if not now["available"]:
        print(f"    {now['error']}")
        return
    print(f"    当前姿态:{'不碰撞' if now['valid'] else '碰撞 ' + str(now['contacts'])}")
    # 假设左臂第 4 关节(肘)再弯 90 度,只问 MoveIt,不动机器人
    elbow = config.JOINT_GROUPS["left_arm"][3]
    what_if = {elbow: ros.joints([elbow])[elbow] + math.radians(90)}
    res = ros.check_collision(what_if)
    print(f"    假设 {elbow} 再转 +90°:{'不碰撞' if res['valid'] else '碰撞 ' + str(res['contacts'])}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--step", type=int, choices=range(1, 6), help="只跑某一步")
    parser.add_argument("--link", default=config.LEFT_END, help="第 2~4 步查哪个 link")
    args = parser.parse_args()

    with RosIO() as ros:
        ros.wait_ready()
        steps = {
            1: lambda: step1_joints(ros),
            2: lambda: step2_urdf(ros, args.link),
            3: lambda: step3_tf(ros, args.link),
            4: lambda: step4_fk_vs_tf(ros, args.link),
            5: lambda: step5_collision(ros),
        }
        for n in ([args.step] if args.step else steps):
            steps[n]()


if __name__ == "__main__":
    main()
