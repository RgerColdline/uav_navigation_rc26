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

// 状态机枚举（完全对齐比赛任务）
enum class MissionState
{
    IDLE,
    TAKEOFF,
    WAIT_FOR_MAP,
    NAV_RECOG_AREA,
    HOVER_RECOGNIZE,
    NAV_AIRDROP_AREA,
    HOVER_AIRDROP,
    NAV_STRIKE_AREA,
    LASER_STRIKE,
    RETURN_TO_LAUNCH,
    LANDING,
    FINISHED
};

class MissionController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber nav_status_sub_; // 监听 ego_controller 的状态

    ros::Publisher setpoint_pub_; // 直接发给飞控 (用于起飞/悬停)
    ros::Publisher ego_goal_pub_; // 发给 ego_controller (用于避障导航)

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    MissionState current_state_;
    mavros_msgs::State mavros_state_;

    Eigen::Vector3d current_pos_;
    Eigen::Vector3d init_pos_;
    double current_yaw_;
    double init_yaw_;

    // 比赛目标点
    Eigen::Vector2d wp_recog_, wp_airdrop_, wp_strike_;
    double takeoff_height_;

    // 时间与状态标志
    ros::Time state_start_time_;
    int ego_nav_status_; // 0:待机, 1:飞行中, 2:已到达

public:
    MissionController(ros::NodeHandle &nh) : nh_(nh), current_state_(MissionState::IDLE), ego_nav_status_(0)
    {
        // 1. 读取参数
        nh_.param("mission/wp_recog_x", wp_recog_.x(), 5.0);
        nh_.param("mission/wp_recog_y", wp_recog_.y(), 0.0);
        nh_.param("mission/wp_airdrop_x", wp_airdrop_.x(), 10.0);
        nh_.param("mission/wp_airdrop_y", wp_airdrop_.y(), 5.0);
        nh_.param("mission/wp_strike_x", wp_strike_.x(), 15.0);
        nh_.param("mission/wp_strike_y", wp_strike_.y(), -5.0);
        nh_.param("mission/takeoff_height", takeoff_height_, 0.6); // 配合你的 2D 压扁高度

        // 2. 初始化 ROS 接口
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionController::stateCb, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MissionController::odomCb, this);
        nav_status_sub_ = nh_.subscribe<std_msgs::Int8>("/ego_controller/status", 10, &MissionController::navStatusCb, this);

        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ego_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ROS_INFO("[Boss] 任务指挥官已就绪！采用解耦架构，Ego-Planner 作为外部导航引擎。");
    }

    // 核心 Tick 函数 (建议在 main 中以 20Hz 运行)
    void tick()
    {
        if (!mavros_state_.connected)
            return;

        switch (current_state_)
        {
        case MissionState::IDLE:
            ROS_INFO_THROTTLE(1.0, "[IDLE] 尝试解锁和切换 OFFBOARD...");
            if (setOffboardAndArm())
            {
                init_pos_ = current_pos_;
                init_yaw_ = current_yaw_;

                // 将相对坐标转换为绝对坐标
                wp_recog_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
                wp_airdrop_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
                wp_strike_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());

                current_state_ = MissionState::TAKEOFF;
                ROS_INFO(">>> [任务开始] 起飞点已锁定，开始爬升！");
            }
            break;

        case MissionState::TAKEOFF:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), init_pos_.z() + takeoff_height_);
            if (std::abs(current_pos_.z() - (init_pos_.z() + takeoff_height_)) < 0.15)
            {
                current_state_ = MissionState::WAIT_FOR_MAP;
                state_start_time_ = ros::Time::now();
                ROS_INFO("[Boss] 到达起飞高度！原位悬停等待点云雷达积累...");
            }
            break;

        case MissionState::WAIT_FOR_MAP:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0)
            {
                ROS_INFO(">>> [任务一] 地图载入完毕，下发目标点前往识别区！");
                sendEgoGoal(wp_recog_); // 向 ego_controller 下发目标
                current_state_ = MissionState::NAV_RECOG_AREA;
            }
            break;

        // =================================================================
        // [绝杀架构] 导航状态下，本 FSM "闭嘴"，停止向 Mavros 发送指令，
        // 全权交由 ego_controller_node 接管飞行！我们只负责监听是否到达。
        // =================================================================
        case MissionState::NAV_RECOG_AREA:
            if (ego_nav_status_ == 2)
            { // 2 表示 ego_controller 汇报已到达
                current_state_ = MissionState::HOVER_RECOGNIZE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>>[任务二] 到达识别区，接管飞控，开始 3 秒悬停...");
            }
            break;

        case MissionState::HOVER_RECOGNIZE:
            publishHoverSetpoint(wp_recog_.x(), wp_recog_.y(), init_pos_.z() + takeoff_height_);
            // TODO: 这里将来可以加入视觉识别成功的回调判断
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0)
            {
                ROS_INFO(">>>[任务三] 识别完毕！下发目标点前往投放区...");
                sendEgoGoal(wp_airdrop_);
                current_state_ = MissionState::NAV_AIRDROP_AREA;
            }
            break;

        case MissionState::NAV_AIRDROP_AREA:
            if (ego_nav_status_ == 2)
            {
                current_state_ = MissionState::HOVER_AIRDROP;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>>[任务四] 到达投放区上方，接管飞控，准备投物...");
            }
            break;

        case MissionState::HOVER_AIRDROP:
            publishHoverSetpoint(wp_airdrop_.x(), wp_airdrop_.y(), init_pos_.z() + takeoff_height_);
            // TODO: 这里将来加入调用舵机投掷的 Service
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0)
            {
                ROS_INFO(">>>[任务五] 投掷完毕！前往靶标攻击阵位...");
                sendEgoGoal(wp_strike_);
                current_state_ = MissionState::NAV_STRIKE_AREA;
            }
            break;

        case MissionState::NAV_STRIKE_AREA:
            if (ego_nav_status_ == 2)
            {
                current_state_ = MissionState::LASER_STRIKE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>> [任务五] 抵达攻击阵位，接管飞控，准备激光打击！");
            }
            break;

        case MissionState::LASER_STRIKE:
            publishHoverSetpoint(wp_strike_.x(), wp_strike_.y(), init_pos_.z() + takeoff_height_);
            // TODO: 视觉伺服微调并点亮激光
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0)
            {
                ROS_INFO(">>> [任务六] 攻击成功！下发目标点，全速返航！");
                sendEgoGoal(Eigen::Vector2d(init_pos_.x(), init_pos_.y()));
                current_state_ = MissionState::RETURN_TO_LAUNCH;
            }
            break;

        case MissionState::RETURN_TO_LAUNCH:
            if (ego_nav_status_ == 2)
            {
                current_state_ = MissionState::LANDING;
                ROS_INFO(">>> [任务六] 回到起飞点上空，开始降落...");
            }
            break;

        case MissionState::LANDING:
            // 缓慢降低高度
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), current_pos_.z() - 0.2);
            if (current_pos_.z() < init_pos_.z() + 0.1)
            {
                current_state_ = MissionState::FINISHED;
                ROS_INFO(">>> 比赛完美收官！");
                // 可选：发送上锁指令
            }
            break;

        case MissionState::FINISHED:
            // 停在地上了，发个推力为0的指令或者直接不管了
            break;
        }
    }

private:
    // ==========================================================
    // 助手函数：向 ego_controller 发送导航目标
    // ==========================================================
    void sendEgoGoal(const Eigen::Vector2d &target)
    {
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "world";
        goal_msg.pose.position.x = target.x();
        goal_msg.pose.position.y = target.y();
        goal_msg.pose.position.z = init_pos_.z() + takeoff_height_;
        goal_msg.pose.orientation.w = 1.0;
        ego_goal_pub_.publish(goal_msg);

        // 重置状态，防止秒触发
        ego_nav_status_ = 1;
    }

    // ==========================================================
    // 助手函数：纯原位悬停控制器（只在起飞、降落、任务区停留时使用）
    // ==========================================================
    void publishHoverSetpoint(double x, double y, double z)
    {
        mavros_msgs::PositionTarget msg;
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // 掩码：只控制位置 X, Y, Z 和 Yaw (忽略速度和加速度)
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        msg.position.x = x;
        msg.position.y = y;
        msg.position.z = z;
        msg.yaw = init_yaw_; // 保持起飞时的机头朝向

        setpoint_pub_.publish(msg);
    }

    // ==========================================================
    // 助手函数：解锁与切换 Offboard
    // ==========================================================
    bool setOffboardAndArm()
    {
        static ros::Time last_req = ros::Time::now();
        static bool has_sent_setpoint = false;

        // PX4 强制要求：切 Offboard 前必须有连续设点流
        if (!has_sent_setpoint)
        {
            publishHoverSetpoint(current_pos_.x(), current_pos_.y(), current_pos_.z());
            if (ros::Time::now() - last_req > ros::Duration(1.0))
            {
                has_sent_setpoint = true;
            }
            return false;
        }

        if (mavros_state_.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(1.0)))
        {
            mavros_msgs::SetMode srv;
            srv.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(srv) && srv.response.mode_sent)
            {
                ROS_INFO("OFFBOARD 切换成功！");
            }
            last_req = ros::Time::now();
        }
        else if (!mavros_state_.armed && (ros::Time::now() - last_req > ros::Duration(0.5)))
        {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            if (arming_client_.call(srv) && srv.response.success)
            {
                ROS_INFO("解锁成功！");
            }
            last_req = ros::Time::now();
        }
        return mavros_state_.armed;
    }

    // 回调函数
    void stateCb(const mavros_msgs::State::ConstPtr &msg) { mavros_state_ = *msg; }
    void navStatusCb(const std_msgs::Int8::ConstPtr &msg) { ego_nav_status_ = msg->data; }
    void odomCb(const nav_msgs::Odometry::ConstPtr &msg)
    {
        current_pos_.x() = msg->pose.pose.position.x;
        current_pos_.y() = msg->pose.pose.position.y;
        current_pos_.z() = msg->pose.pose.position.z;
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        double r, p;
        tf::Matrix3x3(q).getRPY(r, p, current_yaw_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_fsm_node");
    ros::NodeHandle nh("~");

    MissionController boss(nh);

    // 主循环频率，必须大于 2Hz 以维持 Offboard
    ros::Rate rate(20.0);

    while (ros::ok())
    {
        ros::spinOnce();
        boss.tick();
        rate.sleep();
    }
    return 0;
}