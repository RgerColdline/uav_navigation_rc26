#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Int8.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <thread>

// 状态机枚举
enum class MissionState {
    WAIT_FOR_START, IDLE, TAKEOFF, WAIT_FOR_MAP, NAV_RECOG_AREA,
    HOVER_RECOGNIZE, NAV_AIRDROP_AREA, HOVER_AIRDROP, NAV_STRIKE_AREA,
    LASER_STRIKE, RETURN_TO_LAUNCH, LANDING, FINISHED
};

class MissionController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_, odom_sub_, nav_status_sub_;
    ros::Publisher setpoint_pub_, ego_goal_pub_;
    ros::ServiceClient arming_client_, set_mode_client_;

    MissionState current_state_;
    mavros_msgs::State mavros_state_;
    Eigen::Vector3d current_pos_, init_pos_;
    double current_yaw_, init_yaw_;
    Eigen::Vector2d wp_recog_, wp_airdrop_, wp_strike_;
    double takeoff_height_;
    ros::Time state_start_time_;
    int ego_nav_status_;
    bool is_start_pressed_;

public:
    MissionController(ros::NodeHandle &nh) : nh_(nh), current_state_(MissionState::WAIT_FOR_START), ego_nav_status_(0), is_start_pressed_(false) {
        nh_.param<double>("mission/takeoff_height", takeoff_height_, 0.6);
        nh_.param<double>("mission/wp_recog_x", wp_recog_.x(), 5.0);
        nh_.param<double>("mission/wp_recog_y", wp_recog_.y(), 0.0);
        nh_.param<double>("mission/wp_airdrop_x", wp_airdrop_.x(), 10.0);
        nh_.param<double>("mission/wp_airdrop_y", wp_airdrop_.y(), 5.0);
        nh_.param<double>("mission/wp_strike_x", wp_strike_.x(), 15.0);
        nh_.param<double>("mission/wp_strike_y", wp_strike_.y(), -5.0);

        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionController::stateCb, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MissionController::odomCb, this);
        nav_status_sub_ = nh_.subscribe<std_msgs::Int8>("/ego_controller/status", 10, &MissionController::navStatusCb, this);
        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ego_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        std::thread(&MissionController::keyboardThread, this).detach();
    }

    void keyboardThread() {
        int cmd;
        std::cout << "\n\n*************************************************************" << std::endl;
        std::cout << "  [终极指令] 比赛参数加载完毕！等待 Gazebo 和 雷达 加载！" << std::endl;
        std::cout << "  >>>> 请在终端输入 1 并回车，正式授权无人机起飞！ <<<<" << std::endl;
        std::cout << "*************************************************************\n\n" << std::endl;
        while (std::cin >> cmd) {
            if (cmd == 1) {
                is_start_pressed_ = true;
                ROS_INFO("[Boss] 收到起飞指令！开始任务执行流！");
                break;
            }
        }
    }

    void tick() {
        if (!mavros_state_.connected) return;

        switch (current_state_) {
        case MissionState::WAIT_FOR_START:
            if (is_start_pressed_) current_state_ = MissionState::IDLE;
            break;

        case MissionState::IDLE:
            if (setOffboardAndArm()) {
                init_pos_ = current_pos_;
                init_yaw_ = current_yaw_;
                wp_recog_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
                wp_airdrop_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
                wp_strike_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
                current_state_ = MissionState::TAKEOFF;
            }
            break;

        case MissionState::TAKEOFF:
            publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if (std::abs(current_pos_.z() - (init_pos_.z() + takeoff_height_)) < 0.15) {
                current_state_ = MissionState::WAIT_FOR_MAP;
                state_start_time_ = ros::Time::now();
                
                // 【绝杀修复】恢复你老版本的 PCL 启动参数！
                //nh_.setParam("/pcl_enable", true);
                ROS_INFO("[Boss] 到达起飞高度！/pcl_enable 已设为 true，等待点云积累...");
            }
            break;

        case MissionState::WAIT_FOR_MAP:
            publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0) {
                ROS_INFO(">>>[任务一] 地图载入完毕，下发目标点前往识别区！");
                sendEgoGoal(wp_recog_); 
                current_state_ = MissionState::NAV_RECOG_AREA;
            }
            break;

        case MissionState::NAV_RECOG_AREA:
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::HOVER_RECOGNIZE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>>[任务二] 到达识别区！执行定点悬停...");
            }
            break;

        case MissionState::HOVER_RECOGNIZE:
            publishSetpoint(wp_recog_, Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0) {
                sendEgoGoal(wp_airdrop_);
                current_state_ = MissionState::NAV_AIRDROP_AREA;
            }
            break;

        case MissionState::NAV_AIRDROP_AREA:
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::HOVER_AIRDROP;
                state_start_time_ = ros::Time::now();
            }
            break;

        case MissionState::HOVER_AIRDROP:
            publishSetpoint(wp_airdrop_, Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0) {
                sendEgoGoal(wp_strike_);
                current_state_ = MissionState::NAV_STRIKE_AREA;
            }
            break;

        case MissionState::NAV_STRIKE_AREA:
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::LASER_STRIKE;
                state_start_time_ = ros::Time::now();
            }
            break;

        case MissionState::LASER_STRIKE:
            publishSetpoint(wp_strike_, Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0) {
                sendEgoGoal(Eigen::Vector2d(init_pos_.x(), init_pos_.y()));
                current_state_ = MissionState::RETURN_TO_LAUNCH;
            }
            break;

        case MissionState::RETURN_TO_LAUNCH:
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::LANDING;
            }
            break;

        case MissionState::LANDING:
            publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), current_pos_.z() - 0.2, init_yaw_);
            if (current_pos_.z() < init_pos_.z() + 0.1) current_state_ = MissionState::FINISHED;
            break;

        case MissionState::FINISHED:
            publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + 0.0, init_yaw_);
            break;
        }
    }

private:
    void sendEgoGoal(const Eigen::Vector2d &target) {
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "world";
        goal_msg.pose.position.x = target.x();
        goal_msg.pose.position.y = target.y();
        goal_msg.pose.position.z = init_pos_.z() + takeoff_height_;
        goal_msg.pose.orientation.w = 1.0;
        ego_goal_pub_.publish(goal_msg);
        ego_nav_status_ = 1;
    }

    void publishSetpoint(const Eigen::Vector2d &xy, const Eigen::Vector2d &vel_xy, double z, double yaw) {
        mavros_msgs::PositionTarget msg;
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 1
        msg.type_mask = 3040; 
        msg.position.x = xy.x();
        msg.position.y = xy.y();
        msg.position.z = z;  
        msg.velocity.x = vel_xy.x();
        msg.velocity.y = vel_xy.y();
        msg.velocity.z = 0;
        msg.yaw = yaw;
        setpoint_pub_.publish(msg);
    }

    bool setOffboardAndArm() {
        static ros::Time last_req = ros::Time::now();
        static bool has_sent_setpoint = false;
        if (!has_sent_setpoint) {
            publishSetpoint(Eigen::Vector2d(current_pos_.x(), current_pos_.y()), Eigen::Vector2d(0, 0), current_pos_.z(), current_yaw_);
            if (ros::Time::now() - last_req > ros::Duration(1.0)) has_sent_setpoint = true;
            return false;
        }
        if (mavros_state_.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(1.0))) {
            mavros_msgs::SetMode srv;
            srv.request.custom_mode = "OFFBOARD";
            set_mode_client_.call(srv);
            last_req = ros::Time::now();
        } else if (!mavros_state_.armed && (ros::Time::now() - last_req > ros::Duration(0.5))) {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            arming_client_.call(srv);
            last_req = ros::Time::now();
        }
        return mavros_state_.armed;
    }

    void stateCb(const mavros_msgs::State::ConstPtr &msg) { mavros_state_ = *msg; }
    void navStatusCb(const std_msgs::Int8::ConstPtr &msg) { ego_nav_status_ = msg->data; }
    void odomCb(const nav_msgs::Odometry::ConstPtr &msg) {
        current_pos_.x() = msg->pose.pose.position.x;
        current_pos_.y() = msg->pose.pose.position.y;
        current_pos_.z() = msg->pose.pose.position.z;
        tf::Quaternion q; tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        double r, p; tf::Matrix3x3(q).getRPY(r, p, current_yaw_);
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, ""); 

    // 【绝杀修复】最靠前的硬阻塞！不按 1，连 ROS 都不初始化！绝对安静！
    int cmd = 0;
    std::cout << "\n\n*************************************************************" << std::endl;
    std::cout << "  [终极指令] 比赛参数加载完毕！等待 Gazebo 和 雷达 加载！" << std::endl;
    std::cout << "  >>>> 请在终端输入 1 并回车，正式授权无人机起飞！ <<<<" << std::endl;
    std::cout << "*************************************************************\n\n" << std::endl;

    while (true)
    {
        std::cin >> cmd;
        if (cmd == 1)
        {
            std::cout << "[Boss] 收到起飞指令！唤醒 ROS 系统！\n" << std::endl;
            break;
        }
        else
        {
            std::cout << "无效指令，请输入 1 确认起飞。" << std::endl;
        }
    }

    // 按下 1 后，才开始启动 ROS 系统，开始疯狂刷屏和订阅
    ros::init(argc, argv, "main_fsm_node");
    ros::NodeHandle nh("~");

    MissionController boss(nh);
    ros::Rate rate(20.0);

    while (ros::ok())
    {
        ros::spinOnce();
        boss.tick();
        rate.sleep();
    }
    return 0;
}