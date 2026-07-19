#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h> // 【新增】引入 Bool 消息
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <algorithm>

ros::Publisher mavros_cmd_pub;
ros::Publisher ego_goal_pub_1; 
ros::Publisher ego_goal_pub_2; 
ros::Publisher ego_goal_pub_3; 
ros::Publisher nav_status_pub;
ros::Publisher finish_ego_pub; // 【破案修复】专门控制魔改版 traj_server 的开关！

nav_msgs::Odometry current_odom;
quadrotor_msgs::PositionCommand current_traj_cmd;
geometry_msgs::PoseStamped current_goal;

enum NavState { IDLE = 0, FLYING = 1, ARRIVED = 2 };
NavState nav_state = IDLE;

double arrive_radius = 0.3;
bool has_odom = false;
bool has_traj = false;
ros::Time last_traj_time;

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
    // 【修复】从四元数正确解算yaw角，而非错误地取orientation.z
    double qw = current_odom.pose.pose.orientation.w;
    double qx = current_odom.pose.pose.orientation.x;
    double qy = current_odom.pose.pose.orientation.y;
    double qz = current_odom.pose.pose.orientation.z;
    hover_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    ego_goal_pub_1.publish(current_goal);
    ego_goal_pub_2.publish(current_goal);
    ego_goal_pub_3.publish(current_goal);

    // ==============================================================
    // 【破案修复】给魔改版的 traj_server 发送解锁信号！让它开始吐轨迹！
    // ==============================================================
    std_msgs::Bool finish_flag;
    finish_flag.data = false;
    finish_ego_pub.publish(finish_flag);

    nav_state = FLYING;
    has_traj = false;
}

void trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
    current_traj_cmd = *msg;
    has_traj = true; 
    last_traj_time = ros::Time::now();
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
    
    ego_goal_pub_1 = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    ego_goal_pub_2 = nh.advertise<geometry_msgs::PoseStamped>("/uav1/goal", 1);
    ego_goal_pub_3 = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 1);

    // 【破案修复】注册这把钥匙话题
    finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 10);

    ros::Rate rate(30);
    ros::Time last_retry_time = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();
        if (!has_odom) { rate.sleep(); continue; }

        // 【绝杀修复】加上 nav_state == FLYING 的前提！
        // 因为到达(ARRIVED)或待机(IDLE)时，脊髓本来就应该闭嘴！
        if (nav_state == FLYING && has_traj && (ros::Time::now() - last_traj_time).toSec() > 0.5)
        {
            ROS_ERROR("[Ego执行器] 警报！/position_cmd 数据流中断！降级为悬停模式！");
            has_traj = false;
            // 锁定在最后已知轨迹位置，防止飘走
            hover_x = current_traj_cmd.position.x;
            hover_y = current_traj_cmd.position.y;
            hover_z = current_traj_cmd.position.z;
            // 【修复】轨迹中断时同步更新 hover_yaw 为当前正确yaw
            double qw = current_odom.pose.pose.orientation.w;
            double qx = current_odom.pose.pose.orientation.x;
            double qy = current_odom.pose.pose.orientation.y;
            double qz = current_odom.pose.pose.orientation.z;
            hover_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        }

        mavros_msgs::PositionTarget setpoint;
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 1
        
        if (nav_state == FLYING) {
            double dist = sqrt(pow(current_goal.pose.position.x - current_odom.pose.pose.position.x, 2) +
                               pow(current_goal.pose.position.y - current_odom.pose.pose.position.y, 2));

            if (dist < arrive_radius) {
                ROS_INFO("[Ego执行器] 成功到达目标点附近！交还控制权！");
                
                // ==============================================================
                // 【破案修复】到达终点，发送 true 给 traj_server，让它闭嘴！
                // ==============================================================
                std_msgs::Bool finish_flag;
                finish_flag.data = true;
                finish_ego_pub.publish(finish_flag);

                nav_state = ARRIVED;
            } else if (has_traj) {
                setpoint.type_mask = 3040; 
                setpoint.position.x = current_traj_cmd.position.x;
                setpoint.position.y = current_traj_cmd.position.y;
                setpoint.position.z = current_traj_cmd.position.z;
                setpoint.velocity.x = current_traj_cmd.velocity.x;
                setpoint.velocity.y = current_traj_cmd.velocity.y;
                setpoint.velocity.z = current_traj_cmd.velocity.z; 
                setpoint.yaw = current_traj_cmd.yaw;

                ROS_INFO_THROTTLE(1.0, "[Ego执行器] 追踪轨迹中 | 距终点:%.2fm | 发送指令: Pos(%.1f,%.1f) Vel(%.1f,%.1f)", 
                                  dist, setpoint.position.x, setpoint.position.y, setpoint.velocity.x, setpoint.velocity.y);
                mavros_cmd_pub.publish(setpoint);
            } else {
                setpoint.type_mask = 0b100111000111; // 速度零悬停（比位置模式更稳）
                setpoint.velocity.x = 0.0;
                setpoint.velocity.y = 0.0;
                setpoint.velocity.z = 0.0;
                setpoint.yaw = hover_yaw;
                mavros_cmd_pub.publish(setpoint);

                if ((ros::Time::now() - last_retry_time).toSec() > 1.5) {
                    ego_goal_pub_1.publish(current_goal);
                    ego_goal_pub_2.publish(current_goal);
                    ego_goal_pub_3.publish(current_goal);
                    
                    // 【破案修复】再次强调解锁信号，防止丢失
                    std_msgs::Bool finish_flag;
                    finish_flag.data = false;
                    finish_ego_pub.publish(finish_flag);
                    
                    ROS_WARN("[Ego执行器] 重新催促魔改版 traj_server 吐轨迹...");
                    last_retry_time = ros::Time::now();
                }
            }
        } else if (nav_state == ARRIVED) {
            // 【兜底悬停】到达后继续发布悬停setpoint，填补main_fsm接管前的交接间隙，
            // 防止offboard指令流中断导致PX4触发failsafe
            mavros_msgs::PositionTarget setpoint;
            setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            setpoint.type_mask = 0b101111111000; // PX,PY,PZ used; V*,A* ignored; YAW_RATE ignored
            setpoint.position.x = current_odom.pose.pose.position.x;
            setpoint.position.y = current_odom.pose.pose.position.y;
            setpoint.position.z = current_goal.pose.position.z;
            double qw = current_odom.pose.pose.orientation.w;
            double qx = current_odom.pose.pose.orientation.x;
            double qy = current_odom.pose.pose.orientation.y;
            double qz = current_odom.pose.pose.orientation.z;
            setpoint.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            mavros_cmd_pub.publish(setpoint);
        }
        std_msgs::Int8 status_msg;
        status_msg.data = nav_state;
        nav_status_pub.publish(status_msg);

        rate.sleep();
    }
    return 0;
}
