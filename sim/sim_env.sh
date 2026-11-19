#!/bin/bash
# 仿真用的 ROS 环境。必须 source:  source ~/robot_harness/sim/sim_env.sh
#
# 和真机 env_g2.sh 的区别(故意的):
#   · 不 source GDK 的 ros_env.sh:那个会设 ROS_DOMAIN_ID=4 + GDK 的 DDS 网络配置,连到真机。
#   · ROS_DOMAIN_ID=77 + ROS_LOCALHOST_ONLY=1:仿真的话题只在本机、只在 77 号域,
#     仿真里发的 /gdk/joint_control 绝对到不了真机。
#   · gdk_msgs 只是消息定义,照常用(和真机同一份)。

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "请用 source 执行: source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source ~/.cache/agibot/app/gdk/examples/ros/install/local_setup.bash
source ~/agi@g2_robot/install/setup.bash

export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export MUJOCO_GL=egl                      # 仿真离屏渲染用 GPU,不开窗口

echo ">>> G2 仿真环境已加载 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, 只在本机)"
