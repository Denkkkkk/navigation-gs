#!/usr/bin/env python3

"""
neupan_core is the main class for the neupan_ros package. It is used to run the NeuPAN algorithm in the ROS framework.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
"""

from neupan import neupan
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan, PointCloud2
from math import sin, cos, atan2
import numpy as np
from neupan.util import get_transform
import tf2_ros
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Time
import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from controller_interfaces.srv import CustomControllerQuery, SetGoalPose, SetInitialPath
from rcl_interfaces.msg import SetParametersResult
from typing import Tuple
from enum import Enum
from std_srvs.srv import Trigger

class NeupanStatus(Enum):
    ARRIVED = 1
    FAILED = 2
    NAVIGATING = 3
    READY = 4
    PAUSED = 5

class NeupanCore(Node):
    def __init__(self) -> None:
        super().__init__('neupan_node')

        self._declare_params()

        pan = {'dune_checkpoint': self.dune_checkpoint if self.dune_checkpoint else None}
        self.neupan_planner = neupan.init_from_yaml(self.planner_config_file, pan=pan)
        self.get_logger().info(f'neupan planner initialized with config file {self.planner_config_file}')

        self._sync_params()

        self._create_publishers()
        self._create_subscription()
        self._create_service()
        self.timer = self.create_timer(0.2, self.run, callback_group=MutuallyExclusiveCallbackGroup())

        self.last_path_time = self.get_clock().now()
        self.path_update_interval = 0.001  # 最小间隔 x s

        # 数据初始化
        self.obstacle_points = None
        self.robot_state = None
        self.status = NeupanStatus.READY

        self.obstacle_points_lock = threading.Lock()
        self.neupan_lock = threading.Lock()
        self.robot_state_lock = threading.Lock()

        # 创建定时器
        
    
    def _declare_params(self):

        # configuration parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_downsample', 1)
        self.declare_parameter('scan_range', '0.0 5.0')
        self.declare_parameter('dune_checkpoint', '')
        self.declare_parameter('refresh_initial_path', False)
        self.declare_parameter('flip_angle', False)
        # custom added parameters
        self.declare_parameter('reference_speed', 0.5) # speed control
        # robot_size
        self.declare_parameter('d_min', 0.2)
        self.declare_parameter('d_max', 0.25)
        self.declare_parameter('robot_size', '[0.322, 0.22]')

        self.planner_config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.scan_downsample = self.get_parameter('scan_downsample').get_parameter_value().integer_value
        scan_range_para = self.get_parameter('scan_range').get_parameter_value().string_value
        self.scan_range = np.fromstring(scan_range_para, dtype=np.float32, sep=" ")
        self.dune_checkpoint = self.get_parameter('dune_checkpoint').get_parameter_value().string_value
        self.flip_angle = self.get_parameter('flip_angle').get_parameter_value().bool_value

        if self.planner_config_file == '':
            self.get_logger().error('No planner config file provided! Please set the parameter config_file')
            raise ValueError("No planner config file provided")
    
    def _sync_params(self):
        """同步neupan_planner的参数到parameter server"""
        self.set_parameters([
            Parameter('d_min', Parameter.Type.DOUBLE, self.neupan_planner.pan.nrmp_layer.d_min.item()),
            Parameter('d_max', Parameter.Type.DOUBLE, self.neupan_planner.pan.nrmp_layer.d_max.item()),
        ])

        length = self.neupan_planner.pan.robot.vertices[0, :].max().item() - self.neupan_planner.pan.robot.vertices[0, :].min().item()
        width = self.neupan_planner.pan.robot.vertices[1, :].max().item() - self.neupan_planner.pan.robot.vertices[1, :].min().item()

        self.set_parameters([Parameter('robot_size', Parameter.Type.STRING, f'[{length}, {width}]')])
    
    def _create_publishers(self):
        """创建发布者"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # 创建发布者
        self.vel_pub = self.create_publisher(Twist, 'neupan_cmd_vel', qos)
        
        # 可视化发布者
        self.plan_pub = self.create_publisher(Path, 'visualization/neupan_plan', qos)
        self.ref_state_pub = self.create_publisher(Path, 'visualization/neupan_ref_state', qos)
        self.point_markers_pub_dune = self.create_publisher(MarkerArray, 'visualization/dune_point_markers', qos_best)
        self.robot_marker_pub = self.create_publisher(Marker, 'visualization/robot_marker', qos_best)
        self.point_markers_pub_nrmp = self.create_publisher(MarkerArray, 'visualization/nrmp_point_markers', qos_best)

    def _create_subscription(self):
        # Create QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_best, callback_group=MutuallyExclusiveCallbackGroup())
        self.path_sub = self.create_subscription(Path, 'initial_path', self.path_callback, qos_best)
        self.goal_sub = self.create_subscription(PoseStamped, 'neupan_goal', self.goal_callback, qos)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def _create_service(self):
        # 创建服务
        self.state_srv = self.create_service(CustomControllerQuery, 'neupan_controller_query', self.query_callback)
        self.goal_srv = self.create_service(SetGoalPose, 'neupan_set_goal', self.set_goal_callback)
        self.reset_srv = self.create_service(Trigger, 'neupan_reset', self.reset_callback)
        self.pause_srv = self.create_service(Trigger, 'neupan_pause', self.pause_callback)
        self.resume_srv = self.create_service(Trigger, 'neupan_resume', self.resume_callback)

    def run(self):
        """主循环函数"""
        # update robot state
        success, x, y, yaw = self.get_current_tf()

        if not success: return

        with self.robot_state_lock: self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

        if self.neupan_planner.initial_path is None:
            self.get_logger().warn('waiting for setting goal or path for neupan', throttle_duration_sec=3.0)
            return
        
        with self.obstacle_points_lock: obstacle_points = self.obstacle_points
        
        if obstacle_points is None:
            self.get_logger().warn('No obstacle points, only path tracking task will be performed', 
                                  throttle_duration_sec=1.0)
        
        if self.status == NeupanStatus.NAVIGATING:
        
            # 规划主逻辑
            try:
                with self.robot_state_lock: action, info = self.neupan_planner(self.robot_state, obstacle_points)
            except Exception as e:
                self.get_logger().error(f'Error in neupan planner: {str(e)}')
                return
            
            self.status = NeupanStatus.ARRIVED if info["arrive"] else self.status
            self.status = NeupanStatus.FAILED if info["stop"] else self.status

            if self.status == NeupanStatus.NAVIGATING: self.vel_pub.publish(self.generate_twist_msg(action))
        
        else: self.vel_pub.publish(Twist())  # 停止发布速度指令
        
        # Visualizations
        if self.status == NeupanStatus.NAVIGATING:
            opt_state_list = info.get("opt_state_list", None)
            if opt_state_list is not None: self.plan_pub.publish(self.generate_path_msg(opt_state_list))
            else: self.get_logger().warn('opt_state_list not found in planner info', throttle_duration_sec=1.0)

            ref_state_list = info.get("ref_state_list", None)
            if ref_state_list is not None: self.ref_state_pub.publish(self.generate_path_msg(ref_state_list))
            else: self.get_logger().warn('ref_state_list not found in planner info', throttle_duration_sec=1.0)
        
        self.robot_marker_pub.publish(self.generate_robot_marker_msg())
        self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
        self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
        
        # it's stop only when the robot collides
        if self.status == NeupanStatus.FAILED: self.get_logger().warn(
            'Planning failed, please reset the goal or check the map',
            throttle_duration_sec=3
        )
        elif self.status == NeupanStatus.ARRIVED: self.get_logger().info(
            'arrive at the target', 
            throttle_duration_sec=3
        )
            
    def parameter_callback(self, params):
        """动态参数回调函数"""
        for param in params:
            if param.name == 'reference_speed':

                if not (param.type_ == param.Type.DOUBLE):
                    self.get_logger().error('Invalid parameter type for reference_speed')
                    return SetParametersResult(successful=False)
                
                # 立即更新规划器的参考速度
                self.neupan_planner.set_reference_speed(param.value)
                self.get_logger().info(f'Reference speed updated to: {param.value}')
                
            elif param.name == "d_min":

                if not (param.type_ == param.Type.DOUBLE):
                    self.get_logger().error('Invalid parameter type for d_min')
                    return SetParametersResult(successful=False)
                
                self.neupan_planner.update_adjust_parameters(d_min=param.value)
                self.get_logger().info(f'd_min updated to: {param.value}')
            
            elif param.name == "d_max":

                if not (param.type_ == param.Type.DOUBLE):
                    self.get_logger().error('Invalid parameter type for d_max')
                    return SetParametersResult(successful=False)
                self.neupan_planner.update_adjust_parameters(d_max=param.value)
                self.get_logger().info(f'd_max updated to: {param.value}')

        return SetParametersResult(successful=True)
            
    def scan_callback(self, scan_msg: LaserScan):
        """激光扫描回调函数"""
            
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        points = []
        
        if self.flip_angle: angles = np.flip(angles)
            
        for i in range(len(ranges)):
            distance, angle = ranges[i], angles[i]
            
            if (i % self.scan_downsample == 0 and angle > -np.pi and angle < np.pi and
                distance >= self.scan_range[0] and distance <= self.scan_range[1]):
                points.append(np.array([[distance * cos(angle)], [distance * sin(angle)]]))
                
        if len(points) == 0:
            with self.obstacle_points_lock: self.obstacle_points = None
            return
    
        success, x, y, yaw = self.get_current_tf()

        if not success: return
        
        point_array = np.hstack(points)
        trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
        with self.obstacle_points_lock: self.obstacle_points = rot_matrix @ point_array + trans_matrix

    # from ready / navigating to navigating
    def path_callback(self, path: Path):
        
        if self.status == NeupanStatus.PAUSED: return

        # sanity check: downsample path updates
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_path_time).nanoseconds / 1e9
        if time_diff < self.path_update_interval: return
        self.last_path_time = current_time

        initial_point_list = []

        for i in range(len(path.poses) - 1):
            p1, p2 = path.poses[i], path.poses[i + 1]
            x1, x2 = p1.pose.position.x, p2.pose.position.x
            y1, y2 = p1.pose.position.y, p2.pose.position.y
            theta = atan2(y2 - y1, x2 - x1)
            initial_point_list.append(np.array([[x1], [y1], [theta], [1]]))
        
        initial_point_list.append(np.array([
            [path.poses[-1].pose.position.x], 
            [path.poses[-1].pose.position.y], 
            [initial_point_list[-1][2, 0].item()], [1]
        ]))

        self.get_logger().info("target path update", throttle_duration_sec=0.1)
        self.neupan_planner.set_initial_path(initial_point_list)
        self.neupan_planner.reset()

        if self.status != NeupanStatus.NAVIGATING:
            self.get_logger().info("neupan planner is ready to navigate", throttle_duration_sec=0.1)
            self.status = NeupanStatus.NAVIGATING
    
    # from ready / navigating to navigating
    def goal_callback(self, goal):
        
        if self.status == NeupanStatus.PAUSED: return
        
        self.neupan_planner.reset()
        self.status = NeupanStatus.READY
        
        x, y = goal.pose.position.x, goal.pose.position.y
        theta = self.q2y(goal.pose.orientation)

        goal = np.array([[x], [y], [theta]])

        self.get_logger().info(f"set neupan goal: {[x, y, theta]}")
        self.get_logger().info("using goal instead of reference path", throttle_duration_sec=0.1)
        with self.robot_state_lock: self.neupan_planner.update_initial_path_from_goal(self.robot_state, goal)
        self.neupan_planner.reset()

        self.status = NeupanStatus.NAVIGATING
        
    def query_callback(self, request, response):
        if self.status == NeupanStatus.READY: response.status = "ready"
        elif self.status == NeupanStatus.NAVIGATING: response.status = "navigating"
        elif self.status == NeupanStatus.FAILED: response.status = "failed"
        elif self.status == NeupanStatus.ARRIVED: response.status = "arrived"
        elif self.status == NeupanStatus.PAUSED: response.status = "paused"
        return response
    
    # from ready / navigating to navigating
    def set_goal_callback(self, request, response):
        """设置目标位姿的服务回调函数"""
        
        if self.status == NeupanStatus.PAUSED: return
        
        with self.robot_state_lock: robot_state = self.robot_state
        if robot_state is None:
            self.get_logger().warn("Robot state not available, cannot set goal")
            response.success = False
            response.message = "Robot state not available, cannot set goal"
            return response
        
        self.neupan_planner.reset()
        self.status = NeupanStatus.READY
        
        goal_pose = request.goal_pose
        
        # 检查坐标系
        if goal_pose.header.frame_id and goal_pose.header.frame_id != self.map_frame:
            self.get_logger().warn(f'Goal pose frame_id {goal_pose.header.frame_id} differs from map_frame {self.map_frame}')
        
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y
        theta = self.q2y(goal_pose.pose.orientation)

        goal = np.array([[x], [y], [theta]])
        
        self.get_logger().info(f"Service is setting neupan goal: {[x, y, theta]}")
        self.get_logger().info("Reference path update via service")
        self.neupan_planner.update_initial_path_from_goal(robot_state, goal)
        self.neupan_planner.reset()
        response.success = True
        response.message = f"Goal pose set successfully: [{x:.2f}, {y:.2f}, {theta:.2f}]"

        if self.status == NeupanStatus.READY:
            self.get_logger().info("neupan planner is ready to navigate", throttle_duration_sec=0.1)
            self.status = NeupanStatus.NAVIGATING

        return response
    
    # from all to ready
    def reset_callback(self, request, response):
        """取消当前规划的服务回调函数"""
        self.get_logger().info("Resetting neupan planner.")
        self.neupan_planner.reset()
        self.status = NeupanStatus.READY
        response.success = True
        response.message = "Navigation task canceled successfully"

        return response

    # from navigating to paused
    def pause_callback(self, request, response):
        """暂停导航的服务回调函数"""
        if self.status == NeupanStatus.NAVIGATING:
            self.get_logger().info("Pausing neupan planner.")
            self.status = NeupanStatus.PAUSED
            response.success = True
            response.message = "Navigation paused successfully"
        else:
            response.success = False
            response.message = "Cannot pause, not currently navigating"
        
        return response
    
    # from paused to navigating
    def resume_callback(self, request, response):
        """继续导航的服务回调函数"""
        if self.status == NeupanStatus.PAUSED:
            self.get_logger().info("Resuming neupan planner.")
            self.status = NeupanStatus.NAVIGATING
            response.success = True
            response.message = "Navigation resumed successfully"
        else:
            response.success = False
            response.message = "Cannot resume, not currently paused"
        
        return response

    ##############################################################################################
    ####################### Following Code Doesn't Contain Important Logic #######################
    ##############################################################################################

    @staticmethod
    def generate_twist_msg(vel: np.ndarray) -> Twist:

        action = Twist()

        if vel is None: return action

        action.linear.x, action.angular.z = float(vel[0, 0]), float(vel[1, 0])

        return action
    
    def generate_path_msg(self, path_list):
        """生成路径消息"""
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        
        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            
            ps.pose.position.x = float(point[0, 0])
            ps.pose.position.y = float(point[1, 0])
            ps.pose.orientation = self.y2q(point[2, 0])
            
            path.poses.append(ps)
            
        return path
    
    def generate_dune_points_markers_msg(self):
        marker_array = MarkerArray()
        if self.neupan_planner.dune_points is None:
            self.get_logger().warn('No dune points available', throttle_duration_sec=1.0)
            return marker_array
        else:
            points = self.neupan_planner.dune_points
            for index, point in enumerate(points.T):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 160.0 / 255.0
                marker.color.g = 32.0 / 255.0
                marker.color.b = 240.0 / 255.0
                marker.id = index
                marker.type = 1
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.position.z = 0.3
                marker.pose.orientation = self.y2q(0.0)
                marker_array.markers.append(marker)
            return marker_array

    def generate_nrmp_points_markers_msg(self):
        marker_array = MarkerArray()
        if self.neupan_planner.nrmp_points is None:
            self.get_logger().warn('No NRMP points available', throttle_duration_sec=1.0)
            return marker_array
        else:
            points = self.neupan_planner.nrmp_points
            for index, point in enumerate(points.T):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 128.0 / 255.0
                marker.color.b = 0.0
                marker.id = index
                marker.type = 1
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.position.z = 0.3
                marker.pose.orientation = self.y2q(0.0)
                marker_array.markers.append(marker)
            return marker_array

    def generate_robot_marker_msg(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        # self._logger.info(f"marker time: {marker.header.stamp}")
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.id = 0

        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase
            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = 1.0
            marker.type = 1
            with self.robot_state_lock:
                x = self.robot_state[0, 0]
                y = self.robot_state[1, 0]
                theta = self.robot_state[2, 0]
            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y
            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0.0
            marker.pose.orientation = self.y2q(theta)
        return marker
    
    def get_current_tf(self) -> Tuple[bool, float, float, float]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = self.q2y(transform.transform.rotation)

            return True, x, y, yaw
        
        except Exception as e:
            self.get_logger().info(
                f'waiting for transform from {self.base_frame} to {self.map_frame}: {str(e)}',
                throttle_duration_sec=1.0
            )
            return False, 0.0, 0.0, 0.0
    
    @staticmethod
    def q2y(q: Quaternion) -> float: return atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (pow(q.z, 2) + pow(q.y, 2)))
        
    @staticmethod
    def y2q(yaw: float) -> Quaternion: return Quaternion(x=0.0, y=0.0, z=sin(yaw / 2.0), w=cos(yaw / 2.0))