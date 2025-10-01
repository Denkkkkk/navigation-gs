// Copyright (c) 2025 Weilin Zhu
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "angles/angles.h"
#include "neupan_controller/neupan_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT

namespace neupan_controller
{

void NeupanController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_);
  params_ = param_handler_->getParams();

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

  // pub
  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void NeupanController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " neupan_controller::NeupanController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  velocity_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(velocity_data_mutex_);
    latest_velocity_.reset();
  }
}

void NeupanController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "neupan_controller::NeupanController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  auto node = node_.lock();
  if (global_path_pub_->is_activated())
  {
    velocity_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "neupan_cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(velocity_data_mutex_);
        latest_velocity_ = msg;
      });
  }
}

void NeupanController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "neupan_controller::NeupanController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  velocity_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(velocity_data_mutex_);
    latest_velocity_.reset();
  }
}

geometry_msgs::msg::TwistStamped NeupanController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*speed*/,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  // RCLCPP_INFO(logger_, "ready to transformGlobalPlan");

  if (!path_handler_) {
    RCLCPP_ERROR(logger_, "path_handler_ is null!");
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    return cmd_vel;
  }
  
  if (!params_) {
    RCLCPP_ERROR(logger_, "params_ is null!");
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    return cmd_vel;
  }

  auto relevant_global_plan = path_handler_->getRelevantGlobalPlan(
    pose, 3.0, true);
  global_path_pub_->publish(relevant_global_plan);
  // RCLCPP_INFO(logger_, "transformGlobalPlan is done.");
  
  // Get latest velocity data with mutex protection
  geometry_msgs::msg::Twist::SharedPtr current_velocity;
  {
    std::lock_guard<std::mutex> lock(velocity_data_mutex_);
    current_velocity = latest_velocity_;
  }
  double linear_vel = 0.0;
  double angular_vel = 0.0;
  // RCLCPP_DEBUG(logger_, "ready to get velocity data");
  if (current_velocity) {
    // Use received velocity as directly
    linear_vel = current_velocity->linear.x;
    angular_vel = current_velocity->angular.z;
    RCLCPP_DEBUG(logger_, "Received velocity: linear=%.2f, angular=%.2f", 
                 linear_vel, angular_vel);
  } else {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 1000, 
                         "No velocity data received yet");
  }
  // RCLCPP_DEBUG(logger_, "get velocity data is done");

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

bool NeupanController::cancel()
{
  // if false then publish zero velocity
  if (!params_->use_cancel_deceleration) {
    return true;
  }
  cancelling_ = true;
  return finished_cancelling_;
}

void NeupanController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
}

void NeupanController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    params_->desired_linear_vel = params_->base_desired_linear_vel;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      params_->desired_linear_vel = params_->base_desired_linear_vel * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      params_->desired_linear_vel = speed_limit;
    }
  }
}

void NeupanController::reset()
{
  cancelling_ = false;
  finished_cancelling_ = false;
}

}  // namespace neupan_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  neupan_controller::NeupanController,
  nav2_core::Controller)
