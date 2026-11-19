"""G2 仿真:MuJoCo(冒充 GDK)+ 真机同一套 ros2_control 插件 / 控制器 / MoveIt / 头部点云 → octomap + 末端控制面板。

    source ~/robot_harness/sim/sim_env.sh
    ros2 launch ~/robot_harness/sim/launch/sim.launch.py               # 带 RViz
    ros2 launch ~/robot_harness/sim/launch/sim.launch.py use_rviz:=false
    ros2 launch ~/robot_harness/sim/launch/sim.launch.py viewer:=false     # 不开 MuJoCo 窗口
    ros2 launch ~/robot_harness/sim/launch/sim.launch.py panel:=false      # 不开末端控制面板
    ros2 launch ~/robot_harness/sim/launch/sim.launch.py scene:=/path/to/scene.yaml
    ros2 launch ~/robot_harness/sim/launch/sim.launch.py right_ee:=suction left_ee:=suction   # 末端,和真机 hardware.launch.py 同样的参数

照着 agi@g2_robot 的 hardware.launch.py 写的(末端参数 left_ee / right_ee 也一样:none / suction / omnipicker),区别:
  · 先起仿真:物理进程 sim/physics.py(MuJoCo 4,Python 3.12)+ ROS 桥 sim/ros_bridge.py(Python 3.10),
    3 秒后再起 ros2_control:真机插件激活时要已经收到 /gdk/joint_state,否则那一组先不发命令。
  · 不起雷达。
  · 不 include hardware.launch.py 本身(那是真机启动文件)。
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch

SIM_DIR = Path(__file__).resolve().parents[1]
SIM_PYTHON = Path.home() / "mujoco_sim/.venv/bin/python"     # 装了 MuJoCo 4 的 Python 3.12(项目外,uv 建的)


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") == "4":
        raise RuntimeError("ROS_DOMAIN_ID=4 是真机的域。先 source ~/robot_harness/sim/sim_env.sh(会设成 77)")
    if not SIM_PYTHON.exists():
        raise RuntimeError(f"没找到 {SIM_PYTHON},先按 sim/README.md 装 MuJoCo")

    genie_moveit_config = get_package_share_directory("genie_moveit_config")
    xacro_path = os.path.join(genie_moveit_config, "config", "genie.urdf.xacro")
    controllers_yaml = os.path.join(genie_moveit_config, "config", "ros2_controllers.yaml")
    initial_positions_yaml = os.path.join(genie_moveit_config, "config", "initial_positions.yaml")

    left_ee = LaunchConfiguration("left_ee")
    right_ee = LaunchConfiguration("right_ee")
    # use_real_hardware:=true → 用真机插件 GenieG2ArmHardware,它收发的 /gdk/* 话题由仿真冒充
    robot_desc = ParameterValue(
        Command(["xacro ", xacro_path, " use_real_hardware:=true", " initial_positions_file:=", initial_positions_yaml,
                 " control_head:=true control_body:=true", " left_ee:=", left_ee, " right_ee:=", right_ee]),
        value_type=str)

    # 物理进程:MuJoCo 模型按同样的末端参数拼(sim/scene.py)。它不碰 ROS,不需要 ROS 的环境变量
    physics_cmd = [str(SIM_PYTHON), str(SIM_DIR / "physics.py"), "--scene", LaunchConfiguration("scene"),
                   "--left-ee", left_ee, "--right-ee", right_ee]
    physics = ExecuteProcess(cmd=physics_cmd, output="screen", additional_env={"MUJOCO_GL": "egl"},
                             condition=UnlessCondition(LaunchConfiguration("viewer")))
    physics_with_viewer = ExecuteProcess(cmd=physics_cmd + ["--viewer"], output="screen",
                                         additional_env={"MUJOCO_GL": "egl"},
                                         condition=IfCondition(LaunchConfiguration("viewer")))
    # ROS 桥:系统 Python 3.10 + rclpy,冒充 GDK 话题
    bridge = ExecuteProcess(cmd=["python3", str(SIM_DIR / "ros_bridge.py")], output="screen")
    # 末端控制面板(夹爪开合 / 吸盘吸放 / 物体复位),走和真机一样的接口
    panel = ExecuteProcess(cmd=["python3", str(SIM_DIR / "panel.py")], output="screen",
                           condition=IfCondition(LaunchConfiguration("panel")))

    control_node = Node(package="controller_manager", executable="ros2_control_node",
                        parameters=[{"robot_description": robot_desc}, controllers_yaml], output="screen")
    rsp_node = Node(package="robot_state_publisher", executable="robot_state_publisher",
                    parameters=[{"robot_description": robot_desc}])
    # 夹爪状态(和真机 hardware.launch.py 一样):把 /gdk/joint_control 里夹爪的最后一次指令发到 /joint_states
    gripper_state_node = Node(package="genie_robot_agibot_g2_hardware", executable="gripper_joint_state.py",
                              output="screen")
    spawner = Node(package="controller_manager", executable="spawner",
                   arguments=["joint_state_broadcaster", "left_arm_group_controller", "right_arm_group_controller",
                              "head_group_controller", "body_group_controller",
                              "--controller-manager", "/controller_manager", "--service-call-timeout", "5"])

    def moveit_nodes(context):
        ee = {"left_ee": left_ee.perform(context), "right_ee": right_ee.perform(context)}
        cfg = (MoveItConfigsBuilder("genie", package_name="genie_moveit_config")
               .robot_description(mappings=ee)
               .robot_description_semantic(file_path="config/genie.srdf", mappings=ee)
               .to_moveit_configs())
        out = [generate_move_group_launch(cfg)]
        if LaunchConfiguration("use_rviz").perform(context).lower() in ("true", "1"):
            out.append(generate_moveit_rviz_launch(cfg))
        return out

    # 头部深度点云 → MoveIt octomap(和真机同一套节点:head_depth_cloud、shadow_cloud、octomap_sweeper)
    head_cloud = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("genie_robot_agibot_g2_perception"), "launch",
                     "head_depth_cloud.launch.py")))

    robot_stack = [control_node, rsp_node, gripper_state_node,
                   RegisterEventHandler(OnProcessStart(target_action=control_node, on_start=[spawner])),
                   OpaqueFunction(function=moveit_nodes), head_cloud]

    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value=str(SIM_DIR / "scenes/tabletop.yaml")),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("viewer", default_value="true", description="true: 开 MuJoCo 窗口看物理效果"),
        DeclareLaunchArgument("panel", default_value="true", description="true: 开末端控制面板"),
        DeclareLaunchArgument("left_ee", default_value="omnipicker", description="左手末端:none / suction / omnipicker"),
        DeclareLaunchArgument("right_ee", default_value="omnipicker", description="右手末端:none / suction / omnipicker"),
        physics,
        physics_with_viewer,
        bridge,
        TimerAction(period=3.0, actions=robot_stack + [panel]),
    ])
