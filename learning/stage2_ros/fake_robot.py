"""假机器人:没有真机时,发出和真机一样的话题,让其他脚本有数据可读。

它做两件事:
    1. 启动 robot_state_publisher(读 URDF → 发 /robot_description 和 /tf)
    2. 自己发 /joint_states:左臂缓慢摆动,其他关节为 0

必须在单独的 ROS_DOMAIN_ID 里运行,和真机隔开:
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=77
    python3 learning/stage2_ros/fake_robot.py
没有 move_group,所以碰撞检查会显示"不可用",这是正常的。
"""
import math
import os
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from sensor_msgs.msg import JointState

import g2_config
from g2_kinematics import load_urdf


def refuse_robot_domain():
    """假数据绝不能发到真机的网络里:真机的 MoveIt 会把假关节角当真。"""
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    if domain == g2_config.ROBOT_DOMAIN_ID or os.environ.get("DEV_IP"):
        sys.exit(f"当前 ROS_DOMAIN_ID={domain}(或已 source env_g2.sh),这是真机的网络。"
                 f"新开一个终端,只 source /opt/ros/humble/setup.bash,再 export ROS_DOMAIN_ID=77")


def main():
    refuse_robot_domain()
    urdf_xml = g2_config.URDF_FILE.read_text(encoding="utf-8")
    joint_names = [j.name for j in load_urdf(urdf_xml).joints if j.type != "fixed"]

    # robot_state_publisher 是 ROS 自带的程序:给它 URDF,它就根据 /joint_states 算出并发布所有 link 的 TF。
    # URDF 里有注释和特殊字符,不能直接写在命令行上,所以先写进一个参数文件
    params = {"robot_state_publisher": {"ros__parameters": {"robot_description": urdf_xml}}}
    params_file = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump(params, params_file)
    params_file.close()
    rsp = subprocess.Popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher",
                            "--ros-args", "--params-file", params_file.name])

    rclpy.init()
    node = rclpy.create_node("learning_fake_robot")
    pub = node.create_publisher(JointState, g2_config.JOINT_STATES_TOPIC, 10)
    left_arm = g2_config.JOINT_GROUPS["left_arm"]
    print(f"假机器人已启动(ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}),"
          f"{len(joint_names)} 个关节,Ctrl+C 退出")

    t0 = time.monotonic()
    try:
        while rclpy.ok():
            t = time.monotonic() - t0
            positions = {name: 0.0 for name in joint_names}
            for i, name in enumerate(left_arm):        # 左臂每个关节以不同相位慢慢摆 ±0.3 rad
                positions[name] = 0.3 * math.sin(0.5 * t + i)
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = list(positions)
            msg.position = list(positions.values())
            pub.publish(msg)
            time.sleep(0.02)                           # 50 Hz
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        rsp.terminate()                                # 只关掉自己启动的那个进程
        rsp.wait(timeout=5)
        os.remove(params_file.name)


if __name__ == "__main__":
    main()
