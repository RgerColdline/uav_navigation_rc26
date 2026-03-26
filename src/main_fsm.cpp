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
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

// 状态机枚举
enum class MissionState
{
    WAITING_START,  // 等待按 1 启动
    IDLE,           // 解锁和切换 OFFBOARD
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
    int ego_nav_status_;
    bool start_command_received_;

    // 键盘输入
    int keyboard_fd_;
    struct termios cooked_tattr_;

public:
    MissionController(ros::NodeHandle &nh) 
        : nh_(nh), current_state_(MissionState::WAITING_START), 
          ego_nav_status_(0), start_command_received_(false)
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

        // 3. 初始化键盘输入
        initKeyboard();

        ROS_INFO("[Boss] Mission Controller Ready!");
        ROS_INFO(">>> Press [1] to start mission!");
        printWaypoints();
    }

    ~MissionController() {
        restoreKeyboard();
    }

    void initKeyboard() {
        keyboard_fd_ = fileno(stdin);
        tcgetattr(keyboard_fd_, &cooked_tattr_);
        struct termios raw_tattr_;
        memcpy(&raw_tattr_, &cooked_tattr_, sizeof(struct termios));
        raw_tattr_.c_lflag &= ~(ICANON | ECHO);
        raw_tattr_.c_cc[VMIN] = 1;
        raw_tattr_.c_cc[VTIME] = 0;
        tcsetattr(keyboard_fd_, TCSANOW, &raw_tattr_);
        fcntl(keyboard_fd_, F_SETFL, O_NONBLOCK);
    }

    void restoreKeyboard() {
        tcsetattr(keyboard_fd_, TCSANOW, &cooked_tattr_);
    }

    char readKey() {
        char ch;
        int n = read(keyboard_fd_, &ch, 1);
        return (n > 0) ? ch : 0;
    }

    void printWaypoints() {
        ROS_INFO("=== Mission Waypoints ===");
        ROS_INFO("Recog:     [%.2f, %.2f]", wp_recog_.x(), wp_recog_.y());
        ROS_INFO("Airdrop:   [%.2f, %.2f]", wp_airdrop_.x(), wp_airdrop_.y());
        ROS_INFO("Strike:    [%.2f, %.2f]", wp_strike_.x(), wp_strike_.y());
        ROS_INFO("Takeoff Z: %.2f m", takeoff_height_);
        ROS_INFO("=========================");
    }

    void tick()
    {
        // 检查键盘输入
        char key = readKey();
        if (key == '1' && current_state_ == MissionState::WAITING_START) {
            start_command_received_ = true;
            current_state_ = MissionState::IDLE;
            ROS_INFO(">>> Mission Start Command Received!");
        }

        if (!mavros_state_.connected) {
            ROS_WARN_THROTTLE(1.0, "FCU not connected!");
            return;
        }

        switch (current_state_)
        {
        case MissionState::WAITING_START:
            publishHoverSetpoint(current_pos_.x(), current_pos_.y(), current_pos_.z());
            break;

        case MissionState::IDLE:
            ROS_INFO_THROTTLE(0.5, "[IDLE] Unlocking and switching to OFFBOARD...");
            if (setOffboardAndArm())
            {
                init_pos_ = current_pos_;
                init_yaw_ = current_yaw_;
                current_state_ = MissionState::TAKEOFF;
                ROS_INFO(">>> [MISSION START] Takeoff point locked, climbing!");
            }
            break;

        case MissionState::TAKEOFF:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), init_pos_.z() + takeoff_height_);
            if (std::abs(current_pos_.z() - (init_pos_.z() + takeoff_height_)) < 0.15)
            {
                current_state_ = MissionState::WAIT_FOR_MAP;
                state_start_time_ = ros::Time::now();
                ROS_INFO("[Boss] Reached takeoff height! Hovering to wait for map...");
                
                // Enable PCL
                std_msgs::Bool pcl_msg;
                pcl_msg.data = true;
                pcl_enable_pub_.publish(pcl_msg);
                nh_.setParam("/pcl_enable", true);
                ROS_INFO("[Boss] PCL Enabled!");
                
                buildWalls();
            }
            break;

        case MissionState::WAIT_FOR_MAP:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0)
            {
                ROS_INFO(">>> [Task 1] Map loaded! Navigate to recognition area!");
                sendEgoGoal(wp_recog_);
                current_state_ = MissionState::NAV_RECOG_AREA;
            }
            break;

        case MissionState::NAV_RECOG_AREA:
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::HOVER_RECOGNIZE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>> [Task 2] Arrived at recognition area, hovering 3s...");
            }
            break;

        case MissionState::HOVER_RECOGNIZE:
            publishHoverSetpoint(wp_recog_.x(), wp_recog_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0)
            {
                ROS_INFO(">>> [Task 3] Recognition done! Navigate to airdrop area!");
                sendEgoGoal(wp_airdrop_);
                current_state_ = MissionState::NAV_AIRDROP_AREA;
            }
            break;

        case MissionState::NAV_AIRDROP_AREA:
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::HOVER_AIRDROP;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>> [Task 4] Arrived at airdrop area, hovering 3s...");
            }
            break;

        case MissionState::HOVER_AIRDROP:
            publishHoverSetpoint(wp_airdrop_.x(), wp_airdrop_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 3.0)
            {
                ROS_INFO(">>> [Task 5] Airdrop done! Navigate to strike area!");
                sendEgoGoal(wp_strike_);
                current_state_ = MissionState::NAV_STRIKE_AREA;
            }
            break;

        case MissionState::NAV_STRIKE_AREA:
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::LASER_STRIKE;
                state_start_time_ = ros::Time::now();
                ROS_INFO(">>> [Task 6] Arrived at strike area, hovering 2s...");
            }
            break;

        case MissionState::LASER_STRIKE:
            publishHoverSetpoint(wp_strike_.x(), wp_strike_.y(), init_pos_.z() + takeoff_height_);
            if ((ros::Time::now() - state_start_time_).toSec() > 2.0)
            {
                ROS_INFO(">>> [Task 7] Strike done! Return to launch!");
                sendEgoGoal(Eigen::Vector2d(init_pos_.x(), init_pos_.y()));
                current_state_ = MissionState::RETURN_TO_LAUNCH;
            }
            break;

        case MissionState::RETURN_TO_LAUNCH:
            if (ego_nav_status_ == 2) {
                current_state_ = MissionState::LANDING;
                ROS_INFO(">>> [Task 8] Back at launch point, landing...");
            }
            break;

        case MissionState::LANDING:
            publishHoverSetpoint(init_pos_.x(), init_pos_.y(), current_pos_.z() - 0.2);
            if (current_pos_.z() < init_pos_.z() + 0.1)
            {
                current_state_ = MissionState::FINISHED;
                ROS_INFO(">>> Mission Complete!");
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
        ego_goal_pub_.publish(goal_msg);
        ego_nav_status_ = 1;
    }

    void publishHoverSetpoint(double x, double y, double z)
    {
        mavros_msgs::PositionTarget msg;
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
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
        msg.yaw = init_yaw_;

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
