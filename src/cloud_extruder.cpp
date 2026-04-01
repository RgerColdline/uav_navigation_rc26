#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher cloud_pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    // 【修复闪烁】防空包过滤！如果 PCL 这帧没处理出来障碍物，宁愿不发，也不能发空包清空 Ego 的视野
    if (input_cloud.empty())
        return;

    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    // 假设定高 0.6m，拉伸范围 0.0 到 2.0 米
    for (const auto &pt : input_cloud.points)
    {
        // 过滤无穷远无效点
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
            continue;

        // 【修复闪烁】间距从 0.1 放大到 0.2，极大减轻点云运算负担，防止延迟卡顿
        for (float z = 0.0; z <= 2.0; z += 0.2)
        {
            pcl::PointXYZ new_pt;
            new_pt.x = pt.x;
            new_pt.y = pt.y;
            new_pt.z = z;
            output_cloud.points.push_back(new_pt);
        }
    }

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);

    // 【修复闪烁】强行绑定 world 坐标系，绕过 TF 变换带来的跳变延迟
    output_msg.header.frame_id = "world";
    output_msg.header.stamp = ros::Time::now();

    cloud_pub.publish(output_msg);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cloud_extruder");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/projected_accumulated_cloud", 1, cloudCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_pcl", 1);

    ros::spin();
    return 0;
}
