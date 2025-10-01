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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

using std::placeholders::_1;
using std::placeholders::_2;

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
	loadParameters();

	points_missing_times_ = 0;

	result_t_.translation() = Eigen::Vector3d::Zero();
	result_t_.linear() = Eigen::Matrix3d::Identity();
	previous_result_t_ = result_t_;

	accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	register_ = std::make_shared<
		small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

	loadGlobalMap(prior_pcd_file_);

    relocalization_status_ = default_enable_ ? RelocalStatus::FAILED : RelocalStatus::DISABLED;

	resetResults();
	
	createSubscriptions();
	createServices();
	createTimers();
}

void SmallGicpRelocalizationNode::loadParameters()
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("robot_base_frame", "base_footprint");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("default_enable", false);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("default_enable", default_enable_);
}

void SmallGicpRelocalizationNode::createServices()
{
	set_initial_pose_service_ = this->create_service<nav2_msgs::srv::SetInitialPose>(
		"relocalization/set_initial_pose",
		std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, _1, _2));

	toggle_relocalization_service_ = this->create_service<std_srvs::srv::SetBool>(
		"relocalization/toggle_relocalization", 
		std::bind(&SmallGicpRelocalizationNode::toggleRelocalizationCallback, this, _1, _2));

	query_status_service_ = this->create_service<small_gicp_relocalization::srv::GetString>(
		"relocalization/query_status", 
		std::bind(&SmallGicpRelocalizationNode::queryStatusCallback, this, _1, _2));
    
    query_map_loaded_service_ = this->create_service<small_gicp_relocalization::srv::GetString>(
        "relocalization/query_map_loaded",
        std::bind(&SmallGicpRelocalizationNode::queryMapLoadedCallback, this, _1, _2));

	set_pcd_service_ = this->create_service<small_gicp_relocalization::srv::SetString>(
		"relocalization/set_pcd_file",
		std::bind(&SmallGicpRelocalizationNode::setPCDServiceCallback, this, _1, _2));

	get_pcd_service_ = this->create_service<small_gicp_relocalization::srv::GetString>(
		"relocalization/get_pcd_file",
		std::bind(&SmallGicpRelocalizationNode::getPCDServiceCallback, this, _1, _2));
}

void SmallGicpRelocalizationNode::createTimers()
{
    registration_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    transform_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // 将耗时的registration timer放在独立的callback group
    register_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SmallGicpRelocalizationNode::performRegistration, this),
        registration_callback_group_);

    // transform timer使用另一个callback group
    transform_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&SmallGicpRelocalizationNode::publishTransform, this),
        transform_callback_group_);
}

void SmallGicpRelocalizationNode::createSubscriptions()
{
    subscription_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = subscription_callback_group_;
    
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "gicp/cloud_in", 10,
        std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1),
        sub_options);
}

bool SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
    if (loading_map_.load() == MapStatus::LOADING) {
        RCLCPP_WARN(this->get_logger(), "Map is already loading.");
        return false;
    }

    loading_map_ = MapStatus::LOADING;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
        loading_map_ = MapStatus::FAILED;
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded global map with %lu points", global_map_->points.size());

    // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
    target_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *global_map_, global_leaf_size_);

    // Estimate covariances of points
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

    // Create KdTree for target
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    
    loading_map_ = MapStatus::LOADED;
    
    return true;
}

void SmallGicpRelocalizationNode::resetResults()
{
    result_t_ = Eigen::Isometry3d::Identity();
    previous_result_t_ = Eigen::Isometry3d::Identity();
    success_queue_mutex_.lock();
    success_queue_.reset();
    success_queue_mutex_.unlock();
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{    
    last_scan_time_ = msg->header.stamp;
    current_scan_frame_id_ = msg->header.frame_id;

    if (relocalization_status_.load() == RelocalStatus::DISABLED) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *scan);
    {
        std::lock_guard<std::mutex> lock(accumulated_cloud_mutex_);
        *accumulated_cloud_ += *scan;
    }
}

void SmallGicpRelocalizationNode::performRegistration()
{
    if (relocalization_status_.load() == RelocalStatus::DISABLED || 
        loading_map_.load() != MapStatus::LOADED ||
        registration_in_progress_.load()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy;

    accumulated_cloud_mutex_.lock();
    if (accumulated_cloud_->empty()) {
        points_missing_times_++;
        if ((points_missing_times_) > 3) {
        RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
        }
        return;
    }
    
    // 复制点云数据
    cloud_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*accumulated_cloud_);
    accumulated_cloud_->clear();
    accumulated_cloud_mutex_.unlock();

    points_missing_times_ = 0;
    registration_in_progress_ = true;

    // 直接使用复制的点云数据进行处理
    source_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *cloud_copy, registered_leaf_size_);  // 使用cloud_copy而不是accumulated_cloud_

    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        source_, small_gicp::KdTreeBuilderOMP(num_threads_));

    if (!source_ || !source_tree_) {
        registration_in_progress_ = false;
        return;
    }

    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = max_dist_sq_;
    register_->optimizer.max_iterations = 10;

    auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

    bool successful = false;
    if (result.converged) {
        success_queue_mutex_.lock();
        successful = success_queue_.push_sample_consecutive(true);
        success_queue_mutex_.unlock();
        result_t_ = previous_result_t_ = result.T_target_source;
    } else {
        success_queue_mutex_.lock();
        successful = success_queue_.push_sample_consecutive(false);
        success_queue_mutex_.unlock();
        RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
    }

    RCLCPP_INFO(this->get_logger(), "GICP registration %s", successful ? "succeeded" : "failed");

    relocalization_status_ = successful ? RelocalStatus::SUCCESSFUL : RelocalStatus::FAILED;

    registration_in_progress_ = false;
}

void SmallGicpRelocalizationNode::publishTransform()
{
    geometry_msgs::msg::TransformStamped transform_stamped;

    // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
    transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id = odom_frame_;

    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    if (result_t_.matrix().isZero()) resetResults();
    
    translation = result_t_.translation();
    rotation = Eigen::Quaterniond(result_t_.rotation());

    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
    const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response)
{
    RCLCPP_INFO(
        this->get_logger(), 
        "Received initial pose: [x: %f, y: %f, z: %f]", 
        request->pose.pose.pose.position.x,
        request->pose.pose.pose.position.y,
        request->pose.pose.pose.position.z
    );

    Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
    map_to_robot_base.translation() <<
        request->pose.pose.pose.position.x,
        request->pose.pose.pose.position.y,
        request->pose.pose.pose.position.z;
    map_to_robot_base.linear() = Eigen::Quaterniond(
        request->pose.pose.pose.orientation.w,
        request->pose.pose.pose.orientation.x,
        request->pose.pose.pose.orientation.y,
        request->pose.pose.pose.orientation.z).toRotationMatrix();

    try {
        auto transform =
        tf_buffer_->lookupTransform(robot_base_frame_, odom_frame_, tf2::TimePointZero);
        Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
        Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

        previous_result_t_ = result_t_ = map_to_odom;
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
        this->get_logger(), "Could not transform initial pose from %s to %s: %s",
        robot_base_frame_.c_str(), odom_frame_.c_str(), ex.what());
    }
}

void SmallGicpRelocalizationNode::toggleRelocalizationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    
    if (request->data) {
        relocalization_status_ = RelocalStatus::FAILED;
        resetResults();
        response->message = "Relocalization enable requested.";
    } else {
        relocalization_status_ = RelocalStatus::DISABLED;
        resetResults();
        response->message = "Relocalization disabled.";
    }
    response->success = true;
}

void SmallGicpRelocalizationNode::queryStatusCallback(
    const std::shared_ptr<small_gicp_relocalization::srv::GetString::Request> request,
    std::shared_ptr<small_gicp_relocalization::srv::GetString::Response> response)
{
    switch (relocalization_status_) 
    {
        case RelocalStatus::DISABLED:
            response->data = "disabled";
            break;
        case RelocalStatus::SUCCESSFUL:
            response->data = "successful";
            break;
        case RelocalStatus::FAILED:
            response->data = "failed";
            break;
    }
}

void SmallGicpRelocalizationNode::queryMapLoadedCallback(
    const std::shared_ptr<small_gicp_relocalization::srv::GetString::Request> request,
    std::shared_ptr<small_gicp_relocalization::srv::GetString::Response> response)
{
    MapStatus current_status = loading_map_.load();
    switch (current_status) 
    {
        case MapStatus::EMPTY:
            response->data = "empty";
            break;
        case MapStatus::FAILED:
            response->data = "failed";
            break;
        case MapStatus::LOADING:
            response->data = "loading";
            break;
        case MapStatus::LOADED:
            response->data = "loaded";
            break;
    }
}

void SmallGicpRelocalizationNode::setPCDServiceCallback(
    const std::shared_ptr<small_gicp_relocalization::srv::SetString::Request> request,
    std::shared_ptr<small_gicp_relocalization::srv::SetString::Response> response)
{
    std::thread([this, request]() {
        this->loadGlobalMap(request->data);
    }).detach();
    response->success = true;
    response->message = "PCD file is loading.";
}

void SmallGicpRelocalizationNode::getPCDServiceCallback(
    const std::shared_ptr<small_gicp_relocalization::srv::GetString::Request> request,
    std::shared_ptr<small_gicp_relocalization::srv::GetString::Response> response)
{
    if (global_map_->empty()) {
        response->data = "";
    } else {
        response->data = prior_pcd_file_;
    }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
