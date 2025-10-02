#include "map_to_odom.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

MapToOdom::MapToOdom() : Node("map_to_odom")
{
    // 参数声明和获取
    this->declare_parameter<double>("vehicleX", 0.0);
    this->declare_parameter<double>("vehicleY", 0.0);
    this->declare_parameter<double>("vehicleYaw", 0.0);
    
    defaultX = this->get_parameter("vehicleX").as_double();
    defaultY = this->get_parameter("vehicleY").as_double();
    defaultYaw = this->get_parameter("vehicleYaw").as_double();

    // TF2 初始化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 发布者
    pubVehicleToMapPose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vehicleToMapPose", 5);
    pubOdomToMapPose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/odomToMapPose", 5);
    pubvehicleToOdom = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vehicleToOdom", 5);
    pubRelocal = this->create_publisher<geometry_msgs::msg::PoseStamped>("/need_reloc", 1);
    pubGoalPoint = this->create_publisher<geometry_msgs::msg::PointStamped>("/move_base_simple/goal", 5);

    // 订阅者
    subReLocal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/relocalization", 2,
        std::bind(&MapToOdom::reLocalizationCallBack, this, std::placeholders::_1));
        
    subInitOdom = this->create_subscription<std_msgs::msg::Bool>(
        "/init_odom", 2,
        std::bind(&MapToOdom::initOdomCallBack, this, std::placeholders::_1));
       
    subStop = this->create_subscription<std_msgs::msg::Bool>(
        "/stop", 5,
        std::bind(&MapToOdom::stopCallBack, this, std::placeholders::_1));

    // 初始化变换
    map_to_odom_trans.pose.position.x = defaultX;
    map_to_odom_trans.pose.position.y = defaultY;
    map_to_odom_trans.pose.position.z = 0;
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, defaultYaw);
    map_to_odom_trans.pose.orientation = tf2::toMsg(quat);

    RCLCPP_INFO(this->get_logger(), "MapToOdom node initialized");
}

void MapToOdom::stopCallBack(const std_msgs::msg::Bool::SharedPtr stop)
{
    safetyStop = stop->data;
}

void MapToOdom::initOdomCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        map_to_odom_trans.pose.position.x = defaultX;
        map_to_odom_trans.pose.position.y = defaultY;
        map_to_odom_trans.pose.position.z = 0;
        
        tf2::Quaternion quat;
        quat.setRPY(0, 0, defaultYaw);
        map_to_odom_trans.pose.orientation = tf2::toMsg(quat);
    }
}

/**
 * @brief 已知vehicle到odom的变换，和vehicle到map的变换，计算odom到map的变换
 */
void MapToOdom::reLocalizationCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr vTm_msg)
{
    geometry_msgs::msg::Quaternion geoQuat = vTm_msg->pose.orientation;
    if (fabs((geoQuat.x * geoQuat.x + geoQuat.y * geoQuat.y + geoQuat.z * geoQuat.z + geoQuat.w * geoQuat.w) - 1) > 0.01)
    {
        RCLCPP_WARN(this->get_logger(), "map_to_odom.cpp: received incorrect orientation!%f", 
                   fabs((geoQuat.x * geoQuat.x + geoQuat.y * geoQuat.y + geoQuat.z * geoQuat.z + geoQuat.w * geoQuat.w) - 1));
        return;
    }

    // 监听vehicle到odom的坐标变换
    Eigen::Isometry3d vehicle_to_odom = Eigen::Isometry3d::Identity();
    try
    {
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

        // 从变换中获取欧拉角
        tf2::Quaternion quat;
        tf2::fromMsg(transform.transform.rotation, quat);
        double roll_vTo, pitch_vTo, yaw_vTo;
        tf2::Matrix3x3(quat).getEulerYPR(yaw_vTo, pitch_vTo, roll_vTo);
        
        Eigen::Matrix3d vehicle_to_odom_rotation;
        vehicle_to_odom_rotation = Eigen::AngleAxisd(yaw_vTo, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(pitch_vTo, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(roll_vTo, Eigen::Vector3d::UnitX());
        vehicle_to_odom.rotate(vehicle_to_odom_rotation);
        vehicle_to_odom.pretranslate(Eigen::Vector3d(transform.transform.translation.x,
                                                     transform.transform.translation.y,
                                                     transform.transform.translation.z));

        // 获取想要的vehicle到map的变换
        Eigen::Isometry3d vehicle_to_map = Eigen::Isometry3d::Identity();
        tf2::Quaternion vTm_quat;
        tf2::fromMsg(geoQuat, vTm_quat);
        double roll_vTm, pitch_vTm, yaw_vTm;
        tf2::Matrix3x3(vTm_quat).getEulerYPR(yaw_vTm, pitch_vTm, roll_vTm);
        
        Eigen::Matrix3d vehicle_to_map_rotation;
        vehicle_to_map_rotation = Eigen::AngleAxisd(yaw_vTm, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(pitch_vTm, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(roll_vTm, Eigen::Vector3d::UnitX());
        vehicle_to_map.rotate(vehicle_to_map_rotation);
        vehicle_to_map.pretranslate(Eigen::Vector3d(vTm_msg->pose.position.x,
                                                    vTm_msg->pose.position.y,
                                                    transform.transform.translation.z)); // 保持z轴不变

        // 计算odom到map的坐标变换
        Eigen::Isometry3d odom_to_map = vehicle_to_map * vehicle_to_odom.inverse();
        Eigen::Vector3d translationVector = odom_to_map.translation();
        map_to_odom_trans.pose.position.x = translationVector.x();
        map_to_odom_trans.pose.position.y = translationVector.y();
        map_to_odom_trans.pose.position.z = translationVector.z();
        Eigen::Quaterniond quat_eigen = Eigen::Quaterniond(odom_to_map.rotation());
        map_to_odom_trans.pose.orientation.x = quat_eigen.x();
        map_to_odom_trans.pose.orientation.y = quat_eigen.y();
        map_to_odom_trans.pose.orientation.z = quat_eigen.z();
        map_to_odom_trans.pose.orientation.w = quat_eigen.w();

        // 发布想要的vehicle到map的坐标变换
        wantVehicleToMap.header.stamp = this->now();
        wantVehicleToMap.header.frame_id = "map";
        wantVehicleToMap.pose = vTm_msg->pose;
        pubVehicleToMapPose->publish(wantVehicleToMap);

        // 发布订阅到的vehicle到odom的坐标变换
        tf2::Quaternion quat1;
        quat1.setRPY(roll_vTo, pitch_vTo, yaw_vTo);
        vehicleToOdom.header.stamp = this->now();
        vehicleToOdom.header.frame_id = "map";
        vehicleToOdom.pose.position.x = transform.transform.translation.x;
        vehicleToOdom.pose.position.y = transform.transform.translation.y;
        vehicleToOdom.pose.position.z = transform.transform.translation.z;
        vehicleToOdom.pose.orientation = tf2::toMsg(quat1);
        pubvehicleToOdom->publish(vehicleToOdom);

        // 发布计算得到的odom到map的坐标变换
        countOdomToMap.header.stamp = this->now();
        countOdomToMap.header.frame_id = "map";
        countOdomToMap.pose = map_to_odom_trans.pose;
        pubOdomToMapPose->publish(countOdomToMap);

        // 发布重定位后的当前目标点
        auto goal_point = geometry_msgs::msg::PointStamped();
        goal_point.header.frame_id = "map";
        goal_point.header.stamp = this->now();
        goal_point.point.x = vTm_msg->pose.position.x;
        goal_point.point.y = vTm_msg->pose.position.y;
        goal_point.point.z = vTm_msg->pose.position.z;
        pubGoalPoint->publish(goal_point);

        RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Relocalization completed successfully");

    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "map_to_odom.cpp: %s", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
        return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapToOdom>();
    
    // 使用定时器替代原来的循环
    rclcpp::Rate rate(100);
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        // 发布 TF 变换
        geometry_msgs::msg::TransformStamped odomTrans;
        odomTrans.header.stamp = node->now();
        odomTrans.header.frame_id = "map";
        odomTrans.child_frame_id = "odom";
        
        odomTrans.transform.translation.x = node->map_to_odom_trans.pose.position.x;
        odomTrans.transform.translation.y = node->map_to_odom_trans.pose.position.y;
        odomTrans.transform.translation.z = node->map_to_odom_trans.pose.position.z;
        odomTrans.transform.rotation = node->map_to_odom_trans.pose.orientation;
        
        node->tf_broadcaster_->sendTransform(odomTrans);
        
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}