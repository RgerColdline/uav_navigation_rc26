#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher cloud_pub;

float x_min = -3.35, x_max = 0.65;   // 场地 X 轴前后边界 (米)
float y_min = -3.25, y_max = 0.75;   // 场地 Y 轴左右边界 (米)
float z_min = 0.0, z_max = 4.0;      // 围墙高度范围
float step                 = 0.2;    // 围墙点云的间距密度
float wall_intensity       = 255.0;  // 赋予围墙最高强度值，防止被误杀！

// 【新增】定义一个全局的强度过滤阈值
double intensity_threshold = 10.0;

void loadParam(const ros::NodeHandle &nh) {
    nh.param<double>("intensity_threshold", intensity_threshold, 10.0);
    nh.param<float>("wall/x_min", x_min, -3.35f);
    nh.param<float>("wall/x_max", x_max, 0.65f);
    nh.param<float>("wall/y_min", y_min, -3.25f);
    nh.param<float>("wall/y_max", y_max, 0.75f);
    nh.param<float>("wall/z_min", z_min, 0.0f);
    nh.param<float>("wall/z_max", z_max, 4.0f);
    nh.param<float>("wall/step", step, 0.2f);
    nh.param<float>("wall/intensity", wall_intensity, 255.0f);
    ROS_INFO("点云强度过滤阈值: %.2f", intensity_threshold);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // 【核心修改 1】将 PointXYZ 改为 PointXYZI，提取并保留强度(Intensity)信息
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    pcl::PointCloud<pcl::PointXYZI> output_cloud;

    // 【优化逻辑】即使 input_cloud 是空的（比如雷达这帧没扫到东西），
    // 我们也不要直接 return！我们要继续往下走，把“虚拟围墙”发给小脑！
    if (!input_cloud.empty()) {
        // 处理真实的雷达障碍物
        for (const auto &pt : input_cloud.points) {
            // 过滤无穷远无效点
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;

            // 【防误报】过滤掉离飞机极近的点（半径0.3m内），防止把螺旋桨/起落架当成墙！
            if (pt.x * pt.x + pt.y * pt.y < 0.09) continue;

            // ==============================================================
            // 【新增】强度过滤！如果该点的反射强度低于阈值，视为噪点，直接抛弃！
            // ==============================================================
            if (pt.intensity < intensity_threshold) continue;

            // 【修复闪烁】间距从 0.1 放大到 0.2，极大减轻点云运算负担
            for (float z = 0.0; z <= 2.0; z += 0.2) {
                pcl::PointXYZI new_pt;
                new_pt.x         = pt.x;
                new_pt.y         = pt.y;
                new_pt.z         = z;
                new_pt.intensity = pt.intensity;  // 继承原生的高强度值

                output_cloud.points.push_back(new_pt);
            }
        }
    }

    // 沿 X 轴画南北两堵墙
    for (float x = x_min; x <= x_max; x += step) {
        for (float z = z_min; z <= z_max; z += step) {
            pcl::PointXYZI pt_south, pt_north;
            pt_south.x         = x;
            pt_south.y         = y_min;
            pt_south.z         = z;
            pt_south.intensity = wall_intensity;
            pt_north.x         = x;
            pt_north.y         = y_max;
            pt_north.z         = z;
            pt_north.intensity = wall_intensity;
            output_cloud.points.push_back(pt_south);
            output_cloud.points.push_back(pt_north);
        }
    }

    // 沿 Y 轴画东西两堵墙
    for (float y = y_min; y <= y_max; y += step) {
        for (float z = z_min; z <= z_max; z += step) {
            pcl::PointXYZI pt_west, pt_east;
            pt_west.x         = x_min;
            pt_west.y         = y;
            pt_west.z         = z;
            pt_west.intensity = wall_intensity;
            pt_east.x         = x_max;
            pt_east.y         = y;
            pt_east.z         = z;
            pt_east.intensity = wall_intensity;
            output_cloud.points.push_back(pt_west);
            output_cloud.points.push_back(pt_east);
        }
    }

    // 如果连围墙都没生成（理论上不可能），就不发布
    if (output_cloud.empty()) return;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);

    // 【修复闪烁】强行绑定 world 坐标系，绕过 TF 变换带来的跳变延迟
    output_msg.header.frame_id = "world";
    output_msg.header.stamp    = msg->header.stamp;  // 保持时间戳对齐
    // output_msg.header.stamp    = ros::Time::now();  // 保持时间戳对齐

    cloud_pub.publish(output_msg);
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cloud_extruder");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");  // 用于读取私有参数

    loadParam(nh_private);

    ROS_INFO("[点云拉伸与过滤节点] 启动成功！当前过滤低强度噪点的阈值为: %.2f",
             intensity_threshold);

    ros::Subscriber sub =
        nh.subscribe("/pcl_detection2/projected_accumulated_cloud", 1, cloudCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_pcl", 1);

    ros::spin();
    return 0;
}