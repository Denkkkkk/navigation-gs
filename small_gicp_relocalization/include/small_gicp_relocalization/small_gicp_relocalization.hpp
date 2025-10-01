// Copyright 2025 Weilin Zhu
// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <deque>
#include <algorithm>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"

#include "small_gicp_relocalization/srv/get_string.hpp"
#include "small_gicp_relocalization/srv/set_string.hpp"
#include "small_gicp_relocalization/debouncer.hpp"

namespace small_gicp_relocalization
{

enum class RelocalStatus
{
  DISABLED,
  SUCCESSFUL,
  FAILED
};

enum class MapStatus
{
    EMPTY,
    FAILED,
    LOADING,
    LOADED
};

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
    explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
    // initializations
    void loadParameters();
    void createServices();
    void createTimers();
    void createSubscriptions();
    bool loadGlobalMap(const std::string & file_name);
    void resetResults();

    // callbacks
    void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void performRegistration();
    void publishTransform();

    // service callbacks
    void initialPoseCallback(
        const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
        std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response);
    void toggleRelocalizationCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void queryStatusCallback(
        const std::shared_ptr<small_gicp_relocalization::srv::GetString::Request> request,
        std::shared_ptr<small_gicp_relocalization::srv::GetString::Response> response);
    void queryMapLoadedCallback(
        const std::shared_ptr<small_gicp_relocalization::srv::GetString::Request> request,
        std::shared_ptr<small_gicp_relocalization::srv::GetString::Response> response);
    void setPCDServiceCallback(
        const std::shared_ptr<small_gicp_relocalization::srv::SetString::Request> request,
        std::shared_ptr<small_gicp_relocalization::srv::SetString::Response> response);
    void getPCDServiceCallback(
        const std::shared_ptr<small_gicp_relocalization::srv::GetString::Request> request,
        std::shared_ptr<small_gicp_relocalization::srv::GetString::Response> response);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;

    rclcpp::CallbackGroup::SharedPtr registration_callback_group_;
    rclcpp::CallbackGroup::SharedPtr transform_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

    std::mutex accumulated_cloud_mutex_;
    std::atomic<bool> registration_in_progress_{false};
    std::mutex success_queue_mutex_;
    small_gicp_relocalization::Debouncer<bool> success_queue_{5};
    std::atomic<RelocalStatus> relocalization_status_{RelocalStatus::DISABLED};
    std::atomic<MapStatus> loading_map_{MapStatus::EMPTY};

    int num_threads_;
    int num_neighbors_;
    int points_missing_times_;
    float global_leaf_size_;
    float registered_leaf_size_;
    float max_dist_sq_;

    bool default_enable_;

    std::string map_frame_;
    std::string odom_frame_;
    std::string prior_pcd_file_;
    std::string base_frame_;
    std::string robot_base_frame_;
    std::string lidar_frame_;
    std::string current_scan_frame_id_;
    rclcpp::Time last_scan_time_;
    Eigen::Isometry3d result_t_;
    Eigen::Isometry3d previous_result_t_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
    std::shared_ptr<
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
        register_;

    rclcpp::TimerBase::SharedPtr transform_timer_;
    rclcpp::TimerBase::SharedPtr register_timer_;

    rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_relocalization_service_;
    rclcpp::Service<small_gicp_relocalization::srv::GetString>::SharedPtr query_status_service_;
    rclcpp::Service<small_gicp_relocalization::srv::GetString>::SharedPtr query_map_loaded_service_;
    rclcpp::Service<small_gicp_relocalization::srv::SetString>::SharedPtr set_pcd_service_;
    rclcpp::Service<small_gicp_relocalization::srv::GetString>::SharedPtr get_pcd_service_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_