/**
 * @file map_to_odom.hpp
 * @brief manage map to odom transformation
 *
 */
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "Eigen/Core"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class MapToOdom : public rclcpp::Node
{
private:
    // 订阅者和发布者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subReLocal;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subInitOdom;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subStop;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubVehicleToMapPose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubOdomToMapPose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubvehicleToOdom;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubRelocal;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubGoalPoint;

    // TF2 相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 参数
    double defaultX = 0;
    double defaultY = 0;
    double defaultYaw = 0;
    geometry_msgs::msg::Quaternion geoQuat_odom;
    int get_relocal_num = 0;
    bool tranf_odom = false;
    geometry_msgs::msg::PoseStamped wantVehicleToMap;
    geometry_msgs::msg::PoseStamped countOdomToMap;
    geometry_msgs::msg::PoseStamped vehicleToOdom;
    bool safetyStop;

public:
    MapToOdom();
    ~MapToOdom() = default;
    
    geometry_msgs::msg::PoseStamped map_to_odom_trans;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    void reLocalizationCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr vTm_msg);
    void initOdomCallBack(const std_msgs::msg::Bool::SharedPtr initOdom_msg);
    void stopCallBack(const std_msgs::msg::Bool::SharedPtr stop);
};