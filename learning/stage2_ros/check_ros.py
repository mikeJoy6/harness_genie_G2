"""打通 ROS:一步一步读出机器人的数据,每一步都打印出来。

    python3 learning/stage2_ros/check_ros.py                    全部 5 步
    python3 learning/stage2_ros/check_ros.py --step 3           只跑第 3 步
    python3 learning/stage2_ros/check_ros.py --link arm_r_end_link   换一个 link

只读,不会让机器人动。
"""
import argparse
import math

import numpy as np

import g2_config
import g2_kinematics
from ros_io import RosIO


def step1_joints(robot_io):
    print("\n[1] 关节角(/joint_states)")
    print(f"    一共 {len(robot_io.joints())} 个关节,数据是 {robot_io.joints_age_s():.2f} 秒前的")
    for group, names in g2_config.JOINT_GROUPS.items():
        values = robot_io.joints(names)
        degs = "  ".join(f"{math.degrees(v):7.2f}" for v in values.values())
        print(f"    {group:<10}(度) {degs}")


def step2_urdf(robot_io, link):
    print("\n[2] 机器人结构(/robot_description)")
    robot = g2_kinematics.load_urdf(robot_io.urdf_xml())
    print(f"    URDF 长 {len(robot_io.urdf_xml())} 个字符,{len(robot.links)} 个 link,{len(robot.joints)} 个 joint")
    print(f"    {g2_config.BASE_FRAME} → {link} 经过:")
    for j in g2_kinematics.chain_joints(robot, g2_config.BASE_FRAME, link):
        print(f"      {j.name:<24} {j.type}")


def step3_tf(robot_io, link):
    print(f"\n[3] TF:{link} 在 {g2_config.BASE_FRAME} 下的位置")
    xyz, quat = robot_io.lookup(g2_config.BASE_FRAME, link)
    print(f"    xyz  = {np.round(xyz, 4).tolist()} 米")
    print(f"    四元数 = {np.round(quat, 4).tolist()}  (x, y, z, w)")


def step4_fk_vs_tf(robot_io, link):
    print("\n[4] 自己算 FK,和 TF 对比")
    robot = g2_kinematics.load_urdf(robot_io.urdf_xml())
    # 两个数据要尽量同一时刻取,否则手臂在动时会有差
    joints = robot_io.joints()
    xyz, quat = robot_io.lookup(g2_config.BASE_FRAME, link)
    T = g2_kinematics.fk(robot, g2_config.BASE_FRAME, link, joints)
    pos_err_mm = np.linalg.norm(T[:3, 3] - xyz) * 1000
    rot_err = g2_kinematics.rotation_diff_deg(T[:3, :3], g2_kinematics.quat_to_matrix(quat))
    print(f"    自己算 xyz = {np.round(T[:3, 3], 4).tolist()}")
    print(f"    TF     xyz = {np.round(xyz, 4).tolist()}")
    print(f"    差 {pos_err_mm:.2f} mm,{rot_err:.3f}°  (手臂在动时会有几毫米,静止时应接近 0)")


def step5_collision(robot_io):
    print("\n[5] MoveIt 碰撞检查(/check_state_validity)")
    now = robot_io.check_collision()
    if not now["available"]:
        print(f"    {now['error']}")
        return
    print(f"    当前姿态:{'不碰撞' if now['valid'] else '碰撞 ' + str(now['contacts'])}")
    # 假设左臂第 4 关节(肘)再弯 90 度,只问 MoveIt,不动机器人
    elbow = g2_config.JOINT_GROUPS["left_arm"][3]
    what_if = {elbow: robot_io.joints([elbow])[elbow] + math.radians(90)}
    res = robot_io.check_collision(what_if)
    print(f"    假设 {elbow} 再转 +90°:{'不碰撞' if res['valid'] else '碰撞 ' + str(res['contacts'])}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--step", type=int, choices=range(1, 6), help="只跑某一步")
    parser.add_argument("--link", default=g2_config.LEFT_END, help="第 2~4 步查哪个 link")
    args = parser.parse_args()

    robot_io = RosIO()                 # 连接 ROS:建节点、订阅话题、后台开始收数据
    try:
        robot_io.wait_ready()          # 等 /joint_states 和 /robot_description 都收到

        # --step 不填时 args.step 是 None,5 步全跑;填了就只跑那一步
        if args.step is None or args.step == 1:
            step1_joints(robot_io)
        if args.step is None or args.step == 2:
            step2_urdf(robot_io, args.link)
        if args.step is None or args.step == 3:
            step3_tf(robot_io, args.link)
        if args.step is None or args.step == 4:
            step4_fk_vs_tf(robot_io, args.link)
        if args.step is None or args.step == 5:
            step5_collision(robot_io)
    finally:
        robot_io.close()               # 不管上面是否出错,最后都断开 ROS


if __name__ == "__main__":
    main()
