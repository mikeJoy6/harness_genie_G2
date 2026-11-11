"""所有名字和数字都放在这里。换机器人、换话题名,只改这个文件。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]          # 项目根目录(.env 在这里)

# 离线时用的 URDF 文件(fake_robot.py 和 kinematics.py 自测用)。真机上 URDF 从话题里收
URDF_FILE = Path.home() / "agi@g2_robot/src/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf"

# ---- ROS 接口的名字 ----
JOINT_STATES_TOPIC = "/joint_states"                # 关节角,一秒几百条
URDF_TOPIC = "/robot_description"                   # URDF 原文,启动时发一次
VALIDITY_SERVICE = "/check_state_validity"          # MoveIt 的碰撞检查服务(move_group 提供)

# ---- 坐标系 ----
BASE_FRAME = "base_link"                            # 机器人底座,所有坐标都相对它
LEFT_END = "arm_l_end_link"
RIGHT_END = "arm_r_end_link"

# 允许大模型查询的 link(给它一个固定列表,比让它自己猜名字可靠)
LINKS_FOR_LLM = [LEFT_END, RIGHT_END, "head_link3", "body_link5", "arm_l_link4", "arm_r_link4"]

# ---- 关节分组(G2 的关节名规律:idx 后面第一位数字表示部位) ----
JOINT_GROUPS = {
    "waist": [f"idx0{i}_body_joint{i}" for i in range(1, 6)],
    "head": [f"idx1{i}_head_joint{i}" for i in range(1, 4)],
    "left_arm": [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)],
    "right_arm": [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)],
}

# ---- 超时(秒) ----
READY_TIMEOUT_S = 5.0          # 等 /joint_states 和 URDF
TF_TIMEOUT_S = 1.0             # 等一次 TF
SERVICE_TIMEOUT_S = 2.0        # 等 MoveIt 回答

# 真机所在的 ROS_DOMAIN_ID(env_g2.sh 设的)。fake_robot.py 拒绝在这个 domain 里发假数据
ROBOT_DOMAIN_ID = "4"
