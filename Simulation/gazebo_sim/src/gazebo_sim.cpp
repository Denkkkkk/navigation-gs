/**
 * @file robotSimulator.cpp
 * @author 李东权 (1327165187@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-02-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using namespace std;
const double PI = 3.1415926;

class RobotSimulator : public rclcpp::Node
{
public:
    RobotSimulator() : Node("robotSimulator")
    {
        // 参数声明
        this->declare_parameter<std::string>("hub_id", "");
        hub_id = this->get_parameter("hub_id").as_string();
        robotFrame = "denk/base_footprint";
        // TF2 相关初始化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅者和发布者
        subScan = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "velodyne_points", 5,
            std::bind(&RobotSimulator::scanHandler, this, std::placeholders::_1));
            
        subScan2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "velodyne_points2", 5,
            std::bind(&RobotSimulator::scan2Handler, this, std::placeholders::_1));
            
        pubVehicleOdom = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 5);
        pubScan = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);

        // 初始化消息头
        registered_points.header.frame_id = "denk/map";
        odomData.header.frame_id = "denk/map";
        odomData.child_frame_id = robotFrame;

        // 定时器，替代原来的循环
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&RobotSimulator::timerCallback, this));
    }

private:
    // ROS2 组件
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subScan;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subScan2;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubVehicleOdom;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScan;

    // 数据存储
    nav_msgs::msg::Odometry odomData;
    std::string hub_id;
    std::string robotFrame;
    sensor_msgs::msg::PointCloud2 registered_points;
    sensor_msgs::msg::PointCloud2 velodyne_points1;
    sensor_msgs::msg::PointCloud2 velodyne_points2;

    void timerCallback()
    {
        // 合并点云
        pcl::PointCloud<pcl::PointXYZI> pcl_points1;
        pcl::PointCloud<pcl::PointXYZI> pcl_points2;
        pcl::PointCloud<pcl::PointXYZI> pcl_registered;
        
        pcl::fromROSMsg(velodyne_points1, pcl_points1);
        // 先剔除NaN点
        pcl::PointCloud<pcl::PointXYZI> pcl_points1_filtered;
        std::vector<int> indices1, indices2;
        for (const auto& point : pcl_points1) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) && std::isfinite(point.intensity)) {
                pcl_points1_filtered.push_back(point);
            }
        }
        pcl_registered = pcl_points1_filtered;
        // pcl::fromROSMsg(velodyne_points2, pcl_points2);
        // for (long unsigned int i = 0; i < pcl_points1.points.size(); i++)
        // {
        //     pcl_registered.push_back(pcl_points1.points[i]);
        // }
        // for (long unsigned int i = 0; i < pcl_points2.points.size(); i++)
        // {
        //     pcl_registered.push_back(pcl_points2.points[i]);
        // }

        pcl::toROSMsg(pcl_registered, registered_points);
        registered_points.header.stamp = this->now();
        registered_points.header.frame_id = "denk/map";
        pubScan->publish(registered_points);

        // 获取 TF 变换并发布里程计
        try
        {
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform("denk/map", robotFrame, tf2::TimePointZero);

            // 从四元数获取欧拉角
            tf2::Quaternion quat;
            tf2::fromMsg(transform.transform.rotation, quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

            // 设置里程计数据
            odomData.pose.pose.orientation = transform.transform.rotation;
            odomData.pose.pose.position.x = transform.transform.translation.x;
            odomData.pose.pose.position.y = transform.transform.translation.y;
            odomData.pose.pose.position.z = transform.transform.translation.z;
            odomData.header.stamp = this->now();
            
            pubVehicleOdom->publish(odomData);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "robotSimulator: %s", ex.what());
        }
    }

    void scanHandler(const sensor_msgs::msg::PointCloud2::SharedPtr scanIn)
    {
        try
        {
            // 转换点云坐标系
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform("denk/map", scanIn->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*scanIn, velodyne_points1, transform);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "scanHandler: %s", ex.what());
        }
    }

    void scan2Handler(const sensor_msgs::msg::PointCloud2::SharedPtr scanIn)
    {
        try
        {
            // 转换点云坐标系
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform("denk/map", scanIn->header.frame_id, scanIn->header.stamp);
            
            tf2::doTransform(*scanIn, velodyne_points2, transform);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "scan2Handler: %s", ex.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}