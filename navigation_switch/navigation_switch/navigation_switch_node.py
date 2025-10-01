#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller_interfaces.srv import NavModeSwitch, GetNavMode

class NavSwitchService(Node):
    def __init__(self):
        super().__init__('navigation_switch_node')
        
        # 当前模式状态
        self.current_mode = 'map'  # 默认有图模式
        
        # 存储最新的速度消息
        self.cmd_vel_smoothed_msg, self.neupan_cmd_vel_msg = None, None
        
        # 订阅速度话题
        self.cmd_vel_smoothed_sub = self.create_subscription(
            Twist, 'cmd_vel_smoothed', self.cmd_vel_smoothed_callback, 10
        )
        self.neupan_cmd_vel_sub = self.create_subscription(
            Twist, 'neupan_cmd_vel', self.neupan_cmd_vel_callback, 10
        )
        
        # 发布速度话题
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建服务
        self.switch_service = self.create_service(
            NavModeSwitch, 'nav_mode_switch', self.nav_mode_switch_callback
        )
        self.current_mode_service = self.create_service(
            GetNavMode, 'get_mode', self.current_mode_callback
        )
        
        # 创建定时器，定期发布速度命令
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10Hz

        self.last_vel_time = rclpy.time.Time()
        
        self.get_logger().info(f'Nav mode switch node started \n Default mode: {self.current_mode}')
    
    def cmd_vel_smoothed_callback(self, msg):
        """接收cmd_vel_smoothed消息"""
        self.cmd_vel_smoothed_msg = msg
        self.last_vel_time = rclpy.time.Time()
        self.get_logger().info('Received cmd_vel_smoothed message', throttle_duration_sec=3)

    def neupan_cmd_vel_callback(self, msg):
        """接收neupan_cmd_vel消息"""
        self.neupan_cmd_vel_msg = msg
        self.last_vel_time = rclpy.time.Time()
        self.get_logger().info('Received neupan_cmd_vel message', throttle_duration_sec=3)
    
    def nav_mode_switch_callback(self, request, response):
        """处理切换模式的服务请求"""
        # 使用data字段作为状态字符串
        mode = request.mode
        self.cmd_vel_smoothed_msg = Twist()  # 重置速度消息
        self.neupan_cmd_vel_msg = Twist()  # 重置速度消息
        
        if mode == "map":
            self.current_mode = "map"
            response.success = True
            response.message = "Switched to map mode (cmd_vel_smoothed)"
            self.get_logger().info('Switched to map mode')
        elif mode == "map-free":
            self.current_mode = "map-free"
            response.success = True
            response.message = "Switched to map-free mode (neupan_cmd_vel)"
            self.get_logger().info('Switched to map-free mode')
        else:
            response.success = False
            response.message = f"Unknown mode: {mode}"
            self.get_logger().warn(f'Unknown mode received: {mode}')
        
        return response
    
    def publish_velocity(self):
        """根据当前模式发布相应的速度命令"""
        cmd_vel = Twist()
        
        if self.current_mode == "map":
            if self.cmd_vel_smoothed_msg is None:
                self.get_logger().warn('No cmd_vel_smoothed message received yet', throttle_duration_sec=3)
                return
            if self.last_vel_time + rclpy.duration.Duration(seconds=1) < rclpy.time.Time():
                self.get_logger().warn('No velocity command received in the last second, publishing zero velocity', throttle_duration_sec=3)
                cmd_vel = Twist()
            else:
                cmd_vel = self.cmd_vel_smoothed_msg
                self.get_logger().debug('Publishing cmd_vel_smoothed velocity', throttle_duration_sec=3)
        elif self.current_mode == "map-free":
            if self.neupan_cmd_vel_msg is None:
                self.get_logger().warn('No neupan_cmd_vel message received yet', throttle_duration_sec=3)
                return
            if self.last_vel_time + rclpy.duration.Duration(seconds=1) < rclpy.time.Time():
                self.get_logger().warn('No velocity command received in the last second, publishing zero velocity', throttle_duration_sec=3)
                cmd_vel = Twist()
            elif self.neupan_cmd_vel_msg is not None:
                cmd_vel = self.neupan_cmd_vel_msg
                self.get_logger().debug('Publishing neupan_cmd_vel velocity', throttle_duration_sec=3)
        
        self.cmd_vel_pub.publish(cmd_vel)

    def current_mode_callback(self, request, response):
        """处理获取当前模式的服务请求"""
        # 返回当前模式信息
        response.success = True
        response.message = f"{self.current_mode}"
        self.get_logger().info(f'Current mode requested: {self.current_mode}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    node = NavSwitchService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()