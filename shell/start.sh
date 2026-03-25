#!/bin/bash

# Session 名称
SESSION="ego_planner_session"

# ================= 配置路径 =================
MAIN_WS=~/first_task_ws
PX4_PATH=/home/jetson/Libraries/PX4-Autopilot

# 清理旧环境
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

# ====================================================
# 窗口 0: 基础仿真 (Sim + Core) - 保持你原来的不变
# ====================================================
tmux new-session -d -s $SESSION -n "sim_core"
tmux send-keys -t $SESSION:0.0 'roscore' C-m
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "sleep 3; \
source ${MAIN_WS}/devel/setup.bash; \
source ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
roslaunch tutorial_gazebo sim.launch" C-m
tmux select-layout -t $SESSION:0 tiled

# ====================================================
# 窗口 1: 感知与点云拉伸处理
# ====================================================
tmux new-window -t $SESSION:1 -n "perception"
# 1.0: 跑你以前的 PCL 处理脚本，输出 /projected_accumulated_cloud
tmux send-keys -t $SESSION:1.0 "sleep 8; source ${MAIN_WS}/devel/setup.bash; cd ${MAIN_WS}/src/pcl_detection2/shell; bash pcl_detection.sh" C-m

# 1.1: 跑你刚刚写的“点云拉伸节点”，转成 3D 发给 Ego-Planner
tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1.1 "sleep 10; source ${MAIN_WS}/devel/setup.bash; rosrun uav_navigation cloud_extruder" C-m
tmux select-layout -t $SESSION:1 tiled

# ====================================================
# 窗口 2: 导航大脑与飞控执行
# ====================================================
tmux new-window -t $SESSION:2 -n "nav_and_control"

# Pane 2.0: 启动整合包（包含：Ego + 翻译官 + C++状态机主控）
# 现在运行这一个 launch，飞机就会尝试解锁、起飞并跑完流程
tmux send-keys -t $SESSION:2.0 "sleep 12; source ${MAIN_WS}/devel/setup.bash; roslaunch uav_navigation ego_nav.launch" C-m

# Pane 2.1: 建议改为监控 ego_controller 的状态，看看导航到哪一步了
tmux split-window -v -t $SESSION:2
tmux send-keys -t $SESSION:2.1 "sleep 15; rostopic echo /ego_controller/status" C-m

# ====================================================
# 窗口 3: 监控窗口
# ====================================================
tmux new-window -t $SESSION:3 -n "monitor"
tmux send-keys -t $SESSION:3.0 "sleep 5; rostopic echo /mavros/local_position/pose" C-m

# 附加到会话
tmux select-window -t $SESSION:2
tmux attach-session -t $SESSION