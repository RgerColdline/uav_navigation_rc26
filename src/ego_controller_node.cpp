#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <algorithm>

ros::Publisher mavros_cmd_pub;
ros::Publisher ego_goal_pub;
ros::Publisher nav_status_pub;

nav_msgs::Odometry current_odom;
quadrotor_msgs::PositionCommand current_traj_cmd;
geometry_msgs::PoseStamped current_goal;

enum NavState
{
    IDLE = 0,
    FLYING = 1,
    ARRIVED = 2
};
NavState nav_state = IDLE;

// 控制与安全参数
double kp_pos = 0.8;        // 【修改】降低 P 参数，使其更柔和
double max_v = 1.0;         // 【新增】物理速度绝对限幅 (m/s)
double arrive_radius = 0.2; // 到达判定半径 (m)
bool has_odom = false;
bool has_traj = false;

// 1. 接收里程计
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_odom = *msg;
    has_odom = true;
}

// 2. 接收主控目标点
void fsmGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_goal = *msg;
    ROS_INFO("\n===========================================");
    ROS_INFO("[Ego_Controller] 收到长官命令！前往新目标: X=%.2f, Y=%.2f", msg->pose.position.x, msg->pose.position.y);
    ROS_INFO("===========================================\n");

    ego_goal_pub.publish(current_goal); // 转发给黑盒 Ego-Planner
    nav_state = FLYING;
    has_traj = false; // 重置轨迹接收标志
}

// 3. 接收 Ego 轨迹指令
void trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    current_traj_cmd = *msg;
    has_traj = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_controller_node");
    ros::NodeHandle nh("~");

    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomCallback);
    ros::Subscriber fsm_goal_sub = nh.subscribe("/fsm/ego_goal", 1, fsmGoalCallback);
    ros::Subscriber traj_sub = nh.subscribe("/position_cmd", 10, trajCmdCallback);

    mavros_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    // 修复：目标点话题必须与 EGO-Planner 的 flight_type=1 匹配
    ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 1);
    nav_status_pub = nh.advertise<std_msgs::Int8>("/ego_controller/status", 10);

    ros::Rate rate(30); // 【重要】强制 30Hz 循环发布，绝不能断开 MAVROS

    while (ros::ok())
    {
        ros::spinOnce();
        if (!has_odom)
        {
            rate.sleep();
            continue;
        }

        mavros_msgs::PositionTarget setpoint;
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        if (nav_state == FLYING)
        {
            double dist = sqrt(pow(current_goal.pose.position.x - current_odom.pose.pose.position.x, 2) +
                               pow(current_goal.pose.position.y - current_odom.pose.pose.position.y, 2));

            ROS_INFO_THROTTLE(1.0, "[Ego_Controller] 正在飞行... 距离目标点: %.2f 米", dist);

            if (dist < arrive_radius)
            {
                ROS_INFO("[Ego_Controller] 成功到达目标点附近！刹车！");
                nav_state = ARRIVED;
            }
            else if (has_traj)
            {
                // --- 丝滑且安全的轨迹追踪 ---
                // 掩码：控制 VX, VY, PZ, YAW
                setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_VZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

                double err_x = current_traj_cmd.position.x - current_odom.pose.pose.position.x;
                double err_y = current_traj_cmd.position.y - current_odom.pose.pose.position.y;

                double cmd_vx = current_traj_cmd.velocity.x + kp_pos * err_x;
                double cmd_vy = current_traj_cmd.velocity.y + kp_pos * err_y;

                // 【绝杀修复】绝对速度限幅，防止前馈爆炸导致翻车！
                setpoint.velocity.x = std::max(-max_v, std::min(max_v, cmd_vx));
                setpoint.velocity.y = std::max(-max_v, std::min(max_v, cmd_vy));
                setpoint.position.z = current_traj_cmd.position.z;
                setpoint.yaw = current_traj_cmd.yaw;

                mavros_cmd_pub.publish(setpoint);
            }
            else
            {
                // 【绝杀修复】还没收到轨迹时，强制原地悬停，防止掉 Offboard
                ROS_WARN_THROTTLE(0.5, "[Ego_Controller] 等待 Ego-Planner 吐出轨迹，安全悬停中...");
                setpoint.type_mask = 0b101111111000; // 纯位置掩码
                setpoint.position.x = current_odom.pose.pose.position.x;
                setpoint.position.y = current_odom.pose.pose.position.y;
                setpoint.position.z = current_goal.pose.position.z;
                setpoint.yaw = current_odom.pose.pose.orientation.z; // 粗略保持yaw
                mavros_cmd_pub.publish(setpoint);
            }
        }
        else
        {
            // 如果处于 IDLE 或 ARRIVED 状态，本节点闭嘴，不发任何指令
            // 此时由主控 FSM 接管发悬停指令
        }

        std_msgs::Int8 status_msg;
        status_msg.data = nav_state;
        nav_status_pub.publish(status_msg);

        rate.sleep();
    }
    return 0;
}