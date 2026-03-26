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
        // 1. 【完美替换】使用你指定的读取参数方式
        nh_.param("mission/wp_recog_x", wp_recog_.x(), 5.0);
        nh_.param("mission/wp_recog_y", wp_recog_.y(), 0.0);
        nh_.param("mission/wp_airdrop_x", wp_airdrop_.x(), 10.0);
        nh_.param("mission/wp_airdrop_y", wp_airdrop_.y(), 5.0);
        nh_.param("mission/wp_strike_x", wp_strike_.x(), 15.0);
        nh_.param("mission/wp_strike_y", wp_strike_.y(), -5.0);
        nh_.param("mission/takeoff_height", takeoff_height_, 1.2);

        // 2. 初始化 ROS 接口
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionController::stateCb, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MissionController::odomCb, this);
        nav_status_sub_ = nh_.subscribe<std_msgs::Int8>("/ego_controller/status", 10, &MissionController::navStatusCb, this);

        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ego_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ROS_INFO("[Boss] 任务指挥官已就绪！各项比赛参数读取无误。");
    }

    // 核心 Tick 函数
    void tick()
    {
        if (!mavros_state_.connected)
            return;

        switch (current_state_)
        {
        case MissionState::IDLE:
            ROS_WARN_THROTTLE(0.5, "[IDLE] 尝试解锁和切换 OFFBOARD...");
            // 【完全继承老代码逻辑】
            if (setOffboardAndArm())
            {
                init_pos_ = current_pos_;
                init_yaw_ = current_yaw_;

                wp_recog_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
                wp_airdrop_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
                wp_strike_ += Eigen::Vector2d(init_pos_.x(), init_pos_.y());

                current_state_ = MissionState::TAKEOFF;
                ROS_INFO(">>>[任务开始] 起飞点已锁定，开始爬升！");
            }
            break;

        case MissionState::TAKEOFF:
            // 【完全继承老代码逻辑】高度就是 z + takeoff_height
            publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if (std::abs(current_pos_.z() - (init_pos_.z() + takeoff_height_)) < 0.15)
            {
                current_state_ = MissionState::WAIT_FOR_MAP;
                state_start_time_ = ros::Time::now();
                ROS_INFO("[Boss] 到达起飞高度！原位悬停等待点云雷达积累...");
            }
            break;

        case MissionState::WAIT_FOR_MAP:
            publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0)
            {
                ROS_INFO(">>> [任务一] 地图载入完毕，下发目标点前往识别区！");
                sendEgoGoal(wp_recog_); // 向 ego_controller 下发目标
                current_state_ = MissionState::NAV_RECOG_AREA;
            }
            break;

        case MissionState::NAV_RECOG_AREA:
            ROS_INFO_THROTTLE(1.0, "[FSM-Boss] 任务一：正在前往识别区，等待小脑汇报... 状态码: %d", ego_nav_status_);
            if (ego_nav_status_ == 2)
            {
                current_state_ = MissionState::HOVER_RECOGNIZE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>>[任务二] 到达识别区！FSM 接管飞控，执行定点悬停...");
            }
            break;

        case MissionState::HOVER_RECOGNIZE:
            publishSetpoint(wp_recog_, Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0)
            {
                ROS_INFO(">>>[任务三] 识别完毕！下发目标点前往投放区...");
                sendEgoGoal(wp_airdrop_);
                current_state_ = MissionState::NAV_AIRDROP_AREA;
            }
            break;

        case MissionState::NAV_AIRDROP_AREA:
            ROS_INFO_THROTTLE(1.0, "[FSM-Boss] 任务三：正在前往投放区，等待小脑汇报... 状态码: %d", ego_nav_status_);
            if (ego_nav_status_ == 2)
            {
                current_state_ = MissionState::HOVER_AIRDROP;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>>[任务四] 到达投放区上方，接管飞控，准备投物...");
            }
            break;

        case MissionState::HOVER_AIRDROP:
            publishSetpoint(wp_airdrop_, Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0)
            {
                ROS_INFO(">>>[任务五] 投掷完毕！前往靶标攻击阵位...");
                sendEgoGoal(wp_strike_);
                current_state_ = MissionState::NAV_STRIKE_AREA;
            }
            break;

        case MissionState::NAV_STRIKE_AREA:
            ROS_INFO_THROTTLE(1.0, "[FSM-Boss] 任务五：正在前往攻击阵位... 状态码: %d", ego_nav_status_);
            if (ego_nav_status_ == 2)
            {
                current_state_ = MissionState::LASER_STRIKE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>> [任务五] 抵达攻击阵位，接管飞控，准备激光打击！");
            }
            break;

        case MissionState::LASER_STRIKE:
            publishSetpoint(wp_strike_, Eigen::Vector2d(0, 0), init_pos_.z() + takeoff_height_, init_yaw_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0)
            {
                ROS_INFO(">>>[任务六] 攻击成功！下发目标点，全速返航！");
                sendEgoGoal(Eigen::Vector2d(init_pos_.x(), init_pos_.y()));
                current_state_ = MissionState::RETURN_TO_LAUNCH;
            }
            break;

        case MissionState::RETURN_TO_LAUNCH:
            ROS_INFO_THROTTLE(1.0, "[FSM-Boss] 任务六：正在返航... 状态码: %d", ego_nav_status_);
            if (ego_nav_status_ == 2)
            {
                current_state_ = MissionState::LANDING;
                ROS_INFO(">>> [任务六] 回到起飞点上空，开始降落...");
            }
            break;

        case MissionState::LANDING:
            // 缓慢降低高度
            publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), current_pos_.z() - 0.2, init_yaw_);
            if (current_pos_.z() < init_pos_.z() + 0.1)
            {
                current_state_ = MissionState::FINISHED;
                ROS_INFO(">>> 比赛完美收官！停桨！");
            }
            break;

        case MissionState::FINISHED:
            publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + 0.0, init_yaw_);
            break;
        }
    }

private:
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
        ego_nav_status_ = 1;
    }

    // ==========================================================
    // 【完美继承老代码】你的 publishSetpoint，不动一分一毫！
    // ==========================================================
    void publishSetpoint(const Eigen::Vector2d &xy, const Eigen::Vector2d &vel_xy, double z, double yaw)
    {
        mavros_msgs::PositionTarget msg;
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 即 1
        msg.type_mask = 3040;                                                // 核心！同时监听位置和速度！

        // 【致命 Bug 终极修复】ENU 到 NED 的十字翻转！
        msg.position.x = xy.y();
        msg.position.y = xy.x();
        msg.position.z = -z;

        msg.velocity.x = vel_xy.y();
        msg.velocity.y = vel_xy.x();
        msg.velocity.z = 0;

        double yaw_ned = M_PI / 2.0 - yaw;
        while (yaw_ned > M_PI)
            yaw_ned -= 2.0 * M_PI;
        while (yaw_ned < -M_PI)
            yaw_ned += 2.0 * M_PI;
        msg.yaw = yaw_ned;

        setpoint_pub_.publish(msg);
    }

    // ==========================================================
    // 【完美继承老代码】你的 setOffboardAndArm，包含 1.5s 的预热！
    // ==========================================================
    bool setOffboardAndArm()
    {
        static ros::Time last_mode_req = ros::Time::now();
        static ros::Time last_arm_req = ros::Time::now();
        static bool has_sent_setpoint = false;

        ROS_WARN_THROTTLE(0.5, "[setOffboardAndArm] mode=%s, armed=%d",
                          mavros_state_.mode.c_str(), mavros_state_.armed);

        if (!has_sent_setpoint)
        {
            publishSetpoint(Eigen::Vector2d(current_pos_.x(), current_pos_.y()), Eigen::Vector2d(0, 0),
                            current_pos_.z(), current_yaw_);
            ROS_INFO_THROTTLE(0.5, "[setOffboardAndArm] 发送设定点以准备 OFFBOARD...");

            if (ros::Time::now() - last_mode_req > ros::Duration(1.5))
            {
                has_sent_setpoint = true;
            }
            return false;
        }

        if (mavros_state_.mode != "OFFBOARD" && (ros::Time::now() - last_mode_req > ros::Duration(1.0)))
        {
            mavros_msgs::SetMode srv;
            srv.request.custom_mode = "OFFBOARD";
            ROS_INFO("[setOffboardAndArm] 请求切换到 OFFBOARD 模式...");

            if (set_mode_client_.call(srv) && srv.response.mode_sent)
            {
                ROS_INFO("[setOffboardAndArm] OFFBOARD 切换成功！");
            }
            last_mode_req = ros::Time::now();
        }

        if (!mavros_state_.armed && (ros::Time::now() - last_arm_req > ros::Duration(0.5)))
        {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            ROS_INFO("[setOffboardAndArm] 请求解锁...");

            if (arming_client_.call(srv) && srv.response.success)
            {
                ROS_INFO("[setOffboardAndArm] 解锁成功！");
            }
            last_arm_req = ros::Time::now();
        }

        return mavros_state_.armed;
    }

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
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "main_fsm_node");
    ros::NodeHandle nh("~");

    // 【绝杀修复】阻断式等待：加载完参数后，程序死等键盘输入，绝对不刷屏！
    int cmd = 0;
    std::cout << "\n\n*************************************************************" << std::endl;
    std::cout << "  [终极指令] 比赛参数加载完毕！等待 Gazebo 和 雷达 加载！" << std::endl;
    std::cout << "  >>>> 请在终端输入 1 并回车，正式授权无人机起飞！ <<<<" << std::endl;
    std::cout << "*************************************************************\n\n"
              << std::endl;

    while (ros::ok())
    {
        std::cin >> cmd;
        if (cmd == 1)
        {
            ROS_INFO("[Boss] 收到起飞指令！开始任务执行流！");
            break;
        }
        else
        {
            ROS_WARN("无效指令，请输入 1 确认起飞。");
        }
    }

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