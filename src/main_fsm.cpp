#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <thread>
#include <iostream>

// 状态机枚举
enum class MissionState
{
    WAIT_FOR_START, // 【新增】等待键盘按1启动
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
    ros::Subscriber nav_status_sub_;

    ros::Publisher setpoint_pub_;
    ros::Publisher ego_goal_pub_;
    ros::Publisher pcl_enable_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    MissionState current_state_;
    mavros_msgs::State mavros_state_;

    Eigen::Vector3d current_pos_;
    Eigen::Vector3d init_pos_;
    double current_yaw_;
    double init_yaw_;

    // 比赛目标点 (相对坐标)
    Eigen::Vector2d wp_recog_, wp_airdrop_, wp_strike_;
    double takeoff_height_;

    // 时间与状态标志
    ros::Time state_start_time_;
    int ego_nav_status_; // 0:待机, 1:飞行中, 2:已到达

    // 【新增】启动标志位
    bool is_start_pressed_;

public:
    MissionController(ros::NodeHandle &nh) : nh_(nh), current_state_(MissionState::WAIT_FOR_START), ego_nav_status_(0), is_start_pressed_(false)
    {
        // 1. 读取参数 (使用 vector 格式)
        std::vector<double> wp_recog_vec, wp_airdrop_vec, wp_strike_vec;

        if (nh_.getParam("mission/wp_recognition", wp_recog_vec) && wp_recog_vec.size() >= 2) {
            wp_recog_ = Eigen::Vector2d(wp_recog_vec[0], wp_recog_vec[1]);
        } else {
            wp_recog_ = Eigen::Vector2d(5.0, 0.0);
            ROS_WARN("wp_recognition not found, using default [5.0, 0.0]");
        }

        if (nh_.getParam("mission/wp_airdrop", wp_airdrop_vec) && wp_airdrop_vec.size() >= 2) {
            wp_airdrop_ = Eigen::Vector2d(wp_airdrop_vec[0], wp_airdrop_vec[1]);
        } else {
            wp_airdrop_ = Eigen::Vector2d(10.0, 5.0);
            ROS_WARN("wp_airdrop not found, using default [10.0, 5.0]");
        }

        if (nh_.getParam("mission/wp_strike", wp_strike_vec) && wp_strike_vec.size() >= 2) {
            wp_strike_ = Eigen::Vector2d(wp_strike_vec[0], wp_strike_vec[1]);
        } else {
            wp_strike_ = Eigen::Vector2d(15.0, -5.0);
            ROS_WARN("wp_strike not found, using default [15.0, -5.0]");
        }

        nh_.param("mission/takeoff_height", takeoff_height_, 0.6);

        // 2. 初始化 ROS 接口
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionController::stateCb, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MissionController::odomCb, this);
        nav_status_sub_ = nh_.subscribe<std_msgs::Int8>("/ego_controller/status", 10, &MissionController::navStatusCb, this);

        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ego_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
        pcl_enable_pub_ = nh_.advertise<std_msgs::Bool>("/pcl_enable", 1);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ROS_INFO("[Boss] 任务指挥官已就绪！采用解耦架构，Ego-Planner 作为外部导航引擎。");

        // 【新增】开启监听键盘的独立线程，绝不阻塞 ROS
        std::thread(&MissionController::keyboardThread, this).detach();
    }

    // 【新增】键盘监听线程
    void keyboardThread()
    {
        int cmd;
        std::cout << "\n\n*************************************************************" << std::endl;
        std::cout << "  [终极指令] 系统初始化完成！等待 Gazebo 和 雷达 加载！" << std::endl;
        std::cout << "  >>>> 请在终端输入 1 并回车，授权无人机起飞！ <<<<" << std::endl;
        std::cout << "*************************************************************\n\n"
                  << std::endl;
        while (std::cin >> cmd)
        {
            if (cmd == 1)
            {
                is_start_pressed_ = true;
                ROS_INFO("[Boss] 收到起飞指令！开始任务执行流！");
                break;
            }
            else
            {
                ROS_WARN("无效指令，请输入 1 确认起飞。");
            }
        }
    }

    // 核心 Tick 函数 (建议在 main 中以 20Hz 运行)
    void tick()
    {
        // 检查键盘输入
        char key = readKey();
        if (key == '1' && current_state_ == MissionState::WAITING_START) {
            start_command_received_ = true;
            current_state_ = MissionState::IDLE;
            ROS_INFO(" ");
            ROS_INFO("============================================");
            ROS_INFO(">>> Mission Start Command Received!        ");
            ROS_INFO("============================================");
            ROS_INFO(" ");
        }

        if (!mavros_state_.connected) {
            ROS_WARN_THROTTLE(2.0, "[FSM] FCU not connected! Waiting...");
            return;
        }

        // 【修复】FCU 连接后只显示一次中文提示信息
        static bool mission_info_shown = false;
        if (!mission_info_shown && current_state_ == MissionState::WAITING_START) {
            mission_info_shown = true;
            ROS_INFO(" ");
            ROS_INFO("============================================");
            ROS_INFO("     [Boss] Mission Controller Ready!       ");
            ROS_INFO("============================================");
            ROS_INFO(">>> Press [1] to START MISSION!");
            printWaypoints();
            ROS_INFO("============================================");
            ROS_INFO(" ");
        }

        switch (current_state_)
        {
        case MissionState::WAIT_FOR_START:
            // 【新增】只有按下1，才进入 IDLE 开始索要控制权
            if (is_start_pressed_)
            {
                current_state_ = MissionState::IDLE;
            }
            break;

        case MissionState::IDLE:
            ROS_INFO_THROTTLE(0.5, "[IDLE] Unlocking and switching to OFFBOARD...");
            if (setOffboardAndArm())
            {
                init_pos_ = current_pos_;
                init_yaw_ = current_yaw_;
                current_state_ = MissionState::TAKEOFF;
                ROS_INFO(" ");
                ROS_INFO(">>> [MISSION START] Takeoff point locked, climbing!");
                ROS_INFO("    Takeoff position: [%.2f, %.2f, %.2f]", init_pos_.x(), init_pos_.y(), init_pos_.z());
                ROS_INFO("    Takeoff yaw: %.2f deg", init_yaw_ * 180.0 / M_PI);
                ROS_INFO(" ");
            }
            break;

        case MissionState::TAKEOFF:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), init_pos_.z() + takeoff_height_);
            if (std::abs(current_pos_.z() - (init_pos_.z() + takeoff_height_)) < 0.15)
            {
                current_state_ = MissionState::WAIT_FOR_MAP;
                state_start_time_ = ros::Time::now();
                ROS_INFO(" ");
                ROS_INFO("[Boss] Reached takeoff height! Hovering to wait for map...");
                ROS_INFO("    Current Z: %.2f, Target Z: %.2f", current_pos_.z(), init_pos_.z() + takeoff_height_);
                ROS_INFO(" ");

                // 【修复 1】在这里按 1 后才启动 PCL
                if (!pcl_enabled_) {
                    std_msgs::Bool pcl_msg;
                    pcl_msg.data = true;
                    pcl_enable_pub_.publish(pcl_msg);
                    nh_.setParam("/pcl_enable", true);
                    pcl_enabled_ = true;
                    ROS_INFO("[Boss] PCL Enabled!");
                    buildWalls();
                }
            }
            break;

        case MissionState::WAIT_FOR_MAP:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0)
            {
                ROS_INFO(" ");
                ROS_INFO(">>> [Task 1] Map loaded! Navigate to recognition area!");
                ROS_INFO("    目标点：[%.2f, %.2f]", wp_recog_.x(), wp_recog_.y());
                ROS_INFO(" ");
                sendEgoGoal(wp_recog_);
                current_state_ = MissionState::NAV_RECOG_AREA;
            }
            break;

        case MissionState::NAV_RECOG_AREA:
            ROS_INFO_THROTTLE(1.0, "[NAV_RECOG] 飞向识别区... 距离=%.2f米", 
                sqrt(pow(wp_recog_.x() - current_pos_.x(), 2) + pow(wp_recog_.y() - current_pos_.y(), 2)));
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::HOVER_RECOGNIZE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(" ");
                ROS_INFO(">>> [Task 2] Arrived at recognition area, hovering 3s...");
                ROS_INFO(" ");
            }
            break;

        case MissionState::HOVER_RECOGNIZE:
            publishHoverSetpoint(wp_recog_.x(), wp_recog_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0)
            {
                ROS_INFO(" ");
                ROS_INFO(">>> [Task 3] Recognition done! Navigate to airdrop area!");
                ROS_INFO("    目标点：[%.2f, %.2f]", wp_airdrop_.x(), wp_airdrop_.y());
                ROS_INFO(" ");
                sendEgoGoal(wp_airdrop_);
                current_state_ = MissionState::NAV_AIRDROP_AREA;
            }
            break;

        case MissionState::NAV_AIRDROP_AREA:
            ROS_INFO_THROTTLE(1.0, "[NAV_AIR] 飞向空投区... 距离=%.2f米", 
                sqrt(pow(wp_airdrop_.x() - current_pos_.x(), 2) + pow(wp_airdrop_.y() - current_pos_.y(), 2)));
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::HOVER_AIRDROP;
                state_start_time_ = ros::Time::now();
                ROS_INFO(" ");
                ROS_INFO(">>> [Task 4] Arrived at airdrop area, hovering 3s...");
                ROS_INFO(" ");
            }
            break;

        case MissionState::HOVER_AIRDROP:
            publishHoverSetpoint(wp_airdrop_.x(), wp_airdrop_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0)
            {
                ROS_INFO(" ");
                ROS_INFO(">>> [Task 5] Airdrop done! Navigate to strike area!");
                ROS_INFO("    目标点：[%.2f, %.2f]", wp_strike_.x(), wp_strike_.y());
                ROS_INFO(" ");
                sendEgoGoal(wp_strike_);
                current_state_ = MissionState::NAV_STRIKE_AREA;
            }
            break;

        case MissionState::NAV_STRIKE_AREA:
            ROS_INFO_THROTTLE(1.0, "[NAV_STRIKE] 飞向打击区... 距离=%.2f米", 
                sqrt(pow(wp_strike_.x() - current_pos_.x(), 2) + pow(wp_strike_.y() - current_pos_.y(), 2)));
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::LASER_STRIKE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(" ");
                ROS_INFO(">>> [Task 6] Arrived at strike area, hovering 2s...");
                ROS_INFO(" ");
            }
            break;

        case MissionState::LASER_STRIKE:
            publishHoverSetpoint(wp_strike_.x(), wp_strike_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0)
            {
                ROS_INFO(" ");
                ROS_INFO(">>> [Task 7] Strike done! Return to launch!");
                ROS_INFO("    返航目标：[%.2f, %.2f]", init_pos_.x(), init_pos_.y());
                ROS_INFO(" ");
                sendEgoGoal(Eigen::Vector2d(init_pos_.x(), init_pos_.y()));
                current_state_ = MissionState::RETURN_TO_LAUNCH;
            }
            break;

        case MissionState::RETURN_TO_LAUNCH:
            ROS_INFO_THROTTLE(1.0, "[FSM-Boss] 任务六：正在返航，等待小脑(Ego)汇报... 当前状态码: %d", ego_nav_status_);
            if (ego_nav_status_ == 2)
            {
                current_state_ = MissionState::LANDING;
                ROS_INFO(" ");
                ROS_INFO(">>> [Task 8] Back at launch point, landing...");
                ROS_INFO(" ");
            }
            break;

        case MissionState::LANDING:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), current_pos_.z() - 0.2);
            ROS_INFO_THROTTLE(0.5, "[LANDING] 下降中... 当前 Z=%.2f", current_pos_.z());
            if (current_pos_.z() < init_pos_.z() + 0.1)
            {
                current_state_ = MissionState::FINISHED;
                ROS_INFO(" ");
                ROS_INFO("============================================");
                ROS_INFO(">>> Mission Complete! 任务完成！            ");
                ROS_INFO("============================================");
                ROS_INFO(" ");
            }
            break;

        case MissionState::FINISHED:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), init_pos_.z());
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
        
        ROS_INFO("[FSM] 发送 EGO 目标点：x=%.2f, y=%.2f, z=%.2f", 
                 goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z);
        
        ego_goal_pub_.publish(goal_msg);
        ego_nav_status_ = 1;
    }

    // ==========================================================
    // 助手函数：纯原位悬停控制器（只在起飞、降落、任务区停留时使用）
    // ==========================================================
    void publishHoverSetpoint(double x_enu, double y_enu, double z_enu)
    {
        mavros_msgs::PositionTarget msg;
        // 【尊重原著】跟 ego_controller 严格保持一致的坐标系！
        msg.coordinate_frame = 1;

        // 掩码：只控制位置 X, Y, Z 和 Yaw (忽略速度和加速度)
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        // 【史诗级纠偏】同样必须执行 ENU -> NED 转换！
        msg.position.x = y_enu;  // x_ned = y_enu
        msg.position.y = x_enu;  // y_ned = x_enu
        msg.position.z = -z_enu; // z_ned = -z_enu

        double yaw_ned = M_PI / 2.0 - init_yaw_;
        while (yaw_ned > M_PI)
            yaw_ned -= 2.0 * M_PI;
        while (yaw_ned < -M_PI)
            yaw_ned += 2.0 * M_PI;
        msg.yaw = yaw_ned;

        setpoint_pub_.publish(msg);
    }

    bool setOffboardAndArm()
    {
        static ros::Time last_req = ros::Time::now();
        static bool has_sent_setpoint = false;

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
                ROS_INFO("OFFBOARD mode activated!");
            }
            last_req = ros::Time::now();
        }
        else if (!mavros_state_.armed && (ros::Time::now() - last_req > ros::Duration(0.5)))
        {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            if (arming_client_.call(srv) && srv.response.success)
            {
                ROS_INFO("Armed!");
            }
            last_req = ros::Time::now();
        }
        return mavros_state_.armed;
    }

    void buildWalls() {
        // 构建边界墙，防止飞出地图
        ROS_INFO("[Boss] Building boundary walls...");
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
    setlocale(LC_ALL, ""); // 【修复】允许终端输出中文，防乱码！
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
    
    boss.restoreKeyboard();
    return 0;
}
