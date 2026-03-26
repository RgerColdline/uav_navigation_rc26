#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>

// ================= 全局变量与配置 =================
ros::Publisher mavros_cmd_pub;
ros::Publisher ego_goal_pub;
ros::Publisher nav_status_pub;

nav_msgs::Odometry current_odom;
quadrotor_msgs::PositionCommand current_traj_cmd;
geometry_msgs::PoseStamped current_goal;

// 节点状态枚举
enum NavState
{
    IDLE = 0,    // 待机，原地悬停
    FLYING = 1,  // 正在执行避障飞行
    ARRIVED = 2, // 已到达目标点
    FAILED = 3   // 规划失败（选做，需读取ego_planner日志或超时判断）
};
NavState nav_state = IDLE;

// 控制参数
double kp_pos = 1.5;        // 位置追踪比例系数
double arrive_radius = 0.2; // 到达判定半径 (米)

// ================= 回调函数区 =================

// 1. 接收底层里程计
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_odom = *msg;
}

// 2. 接收主控状态机发来的目标点
void fsmGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_goal = *msg;
    ROS_INFO("[Ego_Controller] 收到新目标点: x=%.2f, y=%.2f", msg->pose.position.x, msg->pose.position.y);

    // 将目标点转发给 Ego-Planner 触发黑盒规划
    ego_goal_pub.publish(current_goal);

    // 切换状态为正在飞行
    nav_state = FLYING;
}

// 3. 接收 Ego-Planner 的平滑轨迹指令 (核心控制逻辑)
void trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    if (nav_state != FLYING)
        return; // 如果不是飞行状态，不执行轨迹

    current_traj_cmd = *msg;
    mavros_msgs::PositionTarget setpoint;

    // --- 绝杀优化的前馈控制器 ---
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // 启用 X,Y 速度控制，启用 Z 位置控制 (定高飞行必备)
    setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                         mavros_msgs::PositionTarget::IGNORE_PY |
                         mavros_msgs::PositionTarget::IGNORE_VZ |
                         mavros_msgs::PositionTarget::IGNORE_AFX |
                         mavros_msgs::PositionTarget::IGNORE_AFY |
                         mavros_msgs::PositionTarget::IGNORE_AFZ |
                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // 计算位置误差
    double err_x = current_traj_cmd.position.x - current_odom.pose.pose.position.x;
    double err_y = current_traj_cmd.position.y - current_odom.pose.pose.position.y;

    // 速度前馈 + P 控制
    setpoint.velocity.x = current_traj_cmd.velocity.x + kp_pos * err_x;
    setpoint.velocity.y = current_traj_cmd.velocity.y + kp_pos * err_y;

    // Z 轴和偏航角直接发送位置期望
    setpoint.position.z = current_traj_cmd.position.z;
    setpoint.yaw = current_traj_cmd.yaw;

    mavros_cmd_pub.publish(setpoint);
}

// ================= 主循环 =================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_controller_node");
    ros::NodeHandle nh("~");

    // 订阅与发布
    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomCallback);
    ros::Subscriber fsm_goal_sub = nh.subscribe("/fsm/ego_goal", 1, fsmGoalCallback);
    ros::Subscriber traj_sub = nh.subscribe("/position_cmd", 10, trajCmdCallback);

    mavros_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    // 修复：目标点话题必须与 EGO-Planner 的 flight_type=1 匹配
    ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 1);
    nav_status_pub = nh.advertise<std_msgs::Int8>("/ego_controller/status", 10);

    ros::Rate rate(20); // 20Hz 主循环
    while (ros::ok())
    {
        ros::spinOnce();

        std_msgs::Int8 status_msg;

        // 状态机逻辑：判断是否到达
        if (nav_state == FLYING)
        {
            double dist = sqrt(pow(current_goal.pose.position.x - current_odom.pose.pose.position.x, 2) +
                               pow(current_goal.pose.position.y - current_odom.pose.pose.position.y, 2));

            // 如果距离小于阈值，认为到达
            if (dist < arrive_radius)
            {
                ROS_INFO("[Ego_Controller] 已到达目标点！");
                nav_state = ARRIVED;

                // 到达后，发送一个当前位置的定点指令让飞机刹车悬停
                mavros_msgs::PositionTarget hover_cmd;
                hover_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                hover_cmd.type_mask = 0b101111111000; // 纯位置控制掩码
                hover_cmd.position.x = current_odom.pose.pose.position.x;
                hover_cmd.position.y = current_odom.pose.pose.position.y;
                hover_cmd.position.z = current_odom.pose.pose.position.z;
                mavros_cmd_pub.publish(hover_cmd);
            }
        }

        // 持续向主控汇报当前状态
        status_msg.data = nav_state;
        nav_status_pub.publish(status_msg);

        rate.sleep();
    }
    return 0;
}