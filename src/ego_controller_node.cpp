#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <algorithm>

ros::Publisher mavros_cmd_pub;
// 【破案修复】对小脑的三个命门同时开火！
ros::Publisher ego_goal_pub_1; // /goal
ros::Publisher ego_goal_pub_2; // /uav1/goal
ros::Publisher ego_goal_pub_3; // /ego_planner/goal

ros::Publisher nav_status_pub;

nav_msgs::Odometry current_odom;
quadrotor_msgs::PositionCommand current_traj_cmd;
geometry_msgs::PoseStamped current_goal;

enum NavState { IDLE = 0, FLYING = 1, ARRIVED = 2 };
NavState nav_state = IDLE;

double arrive_radius = 0.2;
bool has_odom = false;
bool has_traj = false;

double hover_x = 0.0;
double hover_y = 0.0;
double hover_z = 0.0;
double hover_yaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_odom = *msg;
    has_odom = true;
}

void fsmGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_goal = *msg;
    ROS_INFO("\n===========================================");
    ROS_INFO("[Ego执行器] 收到长官命令！前往新目标: X=%.2f, Y=%.2f", msg->pose.position.x, msg->pose.position.y);
    ROS_INFO("===========================================\n");

    hover_x = current_odom.pose.pose.position.x;
    hover_y = current_odom.pose.pose.position.y;
    hover_z = current_goal.pose.position.z;
    hover_yaw = current_odom.pose.pose.orientation.z;

    // 【破案修复】三管齐下发送目标点！绝对命中！
    ego_goal_pub_1.publish(current_goal);
    ego_goal_pub_2.publish(current_goal);
    ego_goal_pub_3.publish(current_goal);

    nav_state = FLYING;
    has_traj = false;
}

void trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
    current_traj_cmd = *msg;
    has_traj = true; // 终于收到小脑轨迹！
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ego_controller_node");
    ros::NodeHandle nh("~");

    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomCallback);
    ros::Subscriber fsm_goal_sub = nh.subscribe("/fsm/ego_goal", 1, fsmGoalCallback);
    ros::Subscriber traj_sub = nh.subscribe("/position_cmd", 10, trajCmdCallback);

    mavros_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    nav_status_pub = nh.advertise<std_msgs::Int8>("/ego_controller/status", 10);
    
    // 【破案修复】注册这三个真命天女话题！
    ego_goal_pub_1 = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    ego_goal_pub_2 = nh.advertise<geometry_msgs::PoseStamped>("/uav1/goal", 1);
    ego_goal_pub_3 = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 1);

    ros::Rate rate(30);

    while (ros::ok()) {
        ros::spinOnce();
        if (!has_odom) { rate.sleep(); continue; }

        mavros_msgs::PositionTarget setpoint;
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 1
        setpoint.type_mask = 3040; 

        if (nav_state == FLYING) {
            double dist = sqrt(pow(current_goal.pose.position.x - current_odom.pose.pose.position.x, 2) +
                               pow(current_goal.pose.position.y - current_odom.pose.pose.position.y, 2));

            if (dist < arrive_radius) {
                ROS_INFO("[Ego执行器] 成功到达目标点附近！交还控制权！");
                nav_state = ARRIVED;
            } else if (has_traj) {
                setpoint.position.x = current_traj_cmd.position.x;
                setpoint.position.y = current_traj_cmd.position.y;
                setpoint.position.z = current_traj_cmd.position.z;
                
                setpoint.velocity.x = current_traj_cmd.velocity.x;
                setpoint.velocity.y = current_traj_cmd.velocity.y;
                setpoint.velocity.z = 0;
                
                setpoint.yaw = current_traj_cmd.yaw;

                ROS_INFO_THROTTLE(1.0, "[Ego执行器] 距目标:%.2fm | 跟踪小脑轨迹中...", dist);
                mavros_cmd_pub.publish(setpoint);
            } else {
                ROS_WARN_THROTTLE(1.0, "[Ego执行器] 等待小脑轨迹，安全定点悬停中...");
                
                setpoint.position.x = hover_x;
                setpoint.position.y = hover_y;
                setpoint.position.z = hover_z;
                
                setpoint.velocity.x = 0; 
                setpoint.velocity.y = 0; 
                setpoint.velocity.z = 0;
                
                setpoint.yaw = hover_yaw;
                
                mavros_cmd_pub.publish(setpoint);
            }
        }
        
        std_msgs::Int8 status_msg;
        status_msg.data = nav_state;
        nav_status_pub.publish(status_msg);

        rate.sleep();
    }
    return 0;
}