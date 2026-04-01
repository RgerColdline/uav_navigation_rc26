#!/bin/zsh

# 【中文编码修复】强制设置 UTF-8 环境变量，并导出给 tmux 继承
export LANG=zh_CN.UTF-8
export LC_ALL=zh_CN.UTF-8
export ROS_LOG_DIR=$HOME/.ros/log

# 【关键修复】tmux 默认会清空环境变量，需要配置 tmux 保留 UTF-8 支持
# 在 tmux.conf 中设置或启动时传入
tmux set-option -gq default-command "/bin/zsh" 2>/dev/null

# Session 名称
SESSION="ego_planner_session"

# ================= 配置路径 =================
MAIN_WS=~/raicom_ws
PX4_PATH=/home/jetson/Libraries/PX4-Autopilot

# 清理旧环境
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

echo "=============================================="
echo "   UAV Navigation 启动脚本"
echo "=============================================="
echo "工作空间：$MAIN_WS"
echo "PX4 路径：$PX4_PATH"
echo "=============================================="
echo ""

# ====================================================
# 窗口 0: 基础仿真 (Sim + Core)
# ====================================================
tmux new-session -d -s $SESSION -n "sim_core"
# Pane 0.0: roscore
tmux send-keys -t $SESSION:0.0 'roscore' C-m

# Pane 0.1: Gazebo 仿真加载
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "sleep 3; \
export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
source ${MAIN_WS}/devel/setup.zsh; \
source ${PX4_PATH}/Tools/setup_gazebo.zsh ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
echo '============================================'; \
echo '  启动 Gazebo 仿真'; \
echo '  首次启动需要加载 LiDAR 模型 (约 2-3 分钟)'; \
echo '  请等待日志显示 [ros topic name:/livox/lidar]'; \
echo '============================================'; \
roslaunch tutorial_gazebo sim.launch" C-m
tmux select-layout -t $SESSION:0 tiled

# ====================================================
# 窗口 1: 感知与点云拉伸处理 (【调试阶段暂时封印】)
# ====================================================
tmux new-window -t $SESSION:1 -n "perception"
# Pane 1.0: 跑你以前的 PCL 处理脚本，输出 /projected_accumulated_cloud
tmux send-keys -t $SESSION:1.0 "sleep 8; source ${MAIN_WS}/devel/setup.zsh; cd ${MAIN_WS}/src/pcl_detection2/shell; zsh pcl_detection.sh" C-m

# Pane 1.1: 跑点云拉伸节点，转成 3D 发给 Ego-Planner
tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1.1 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
echo '等待 Gazebo 启动...'; sleep 17; \
source ${MAIN_WS}/devel/setup.zsh; rosrun uav_navigation cloud_extruder" C-m
tmux select-layout -t $SESSION:1 tiled

# ====================================================
# 窗口 2: 导航大脑与飞控执行 (这里是主战场)
# ====================================================
tmux new-window -t $SESSION:2 -n "nav_and_control"

# Pane 2.0: 监控飞机的精准坐标 (继承你原来优秀的监控习惯)
tmux send-keys -t $SESSION:2.0 "sleep 5; rostopic echo /mavros/local_position/pose" C-m

# Pane 2.1: 启动终极任务！包含 Ego + 翻译官 + 你等待按1的主控
tmux split-window -h -t $SESSION:2
tmux send-keys -t $SESSION:2.1 "sleep 12; source ~/ego_ws/devel/setup.zsh; source ${MAIN_WS}/devel/setup.zsh; roslaunch uav_navigation ego_nav.launch" C-m
tmux select-layout -t $SESSION:2 tiled

# ====================================================
# 窗口 3: 状态监控面板
# ====================================================
tmux new-window -t $SESSION:3 -n "status_monitor"
# Pane 3.0: 监控 Ego-Controller 的当前到达状态 (0:待机, 1:飞, 2:到)
tmux send-keys -t $SESSION:3.0 "sleep 15; rostopic echo /ego_controller/status" C-m

# 附加到会话，并跳转到主控页面等待你输入 1
tmux select-window -t $SESSION:2
tmux select-pane -t $SESSION:2.1
tmux attach-session -t $SESSION
