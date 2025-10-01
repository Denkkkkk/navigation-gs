from typing import List, Tuple
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.client import Client

from ..models.models import Result
from .utils import AsynVar, log_exceptions

class ROSClient(object):
    
    def __init__(self, ros_node_obj: Node, client_name: str):
        self.ros_node = ros_node_obj
        self.client_name = client_name
        self.healthy = AsynVar(True)
        self.service_list = []
        self.action_list = []
    
    async def start(self):
        self.health_check_task = asyncio.create_task(self._health_check_loop())
    
    def _register_services(self, clients: List[Client]):
        """注册服务客户端"""
        self.service_list.extend(clients)
    
    def _register_actions(self, actions: List[Client]):
        """注册动作客户端"""
        self.action_list.extend(actions)
    
    def _wait_for_services(self):
        self.ros_node.get_logger().info(f'[{self.client_name}]: Waiting for services and action...')
        
        for service in self.service_list:
            while not service.wait_for_service(timeout_sec=1.0):
                self.ros_node.get_logger().info(f'[{self.client_name}]: {service.srv_name} service not available, waiting...')
        
        for action in self.action_list:
            while not action.wait_for_server(timeout_sec=1.0):
                self.ros_node.get_logger().info(f'[{self.client_name}]: {action._action_name} action server not available, waiting...')
        
        self.ros_node.get_logger().info(f'[{self.client_name}]: All services and actions are available!')
    
    @log_exceptions
    async def _health_check_loop(self):
        """健康检查循环"""
        while rclpy.ok():
            healthy = True
            for service in self.service_list:
                if not service.wait_for_service(timeout_sec=1.0):
                    self.ros_node.get_logger().error(f'[{self.client_name}]: {service.srv_name} service not available, health check failed')
                    healthy = False
            
            for action in self.action_list:
                if not action.wait_for_server(timeout_sec=1.0):
                    self.ros_node.get_logger().error(f'[{self.client_name}]: {action._action_name} action server not available, health check failed')
                    healthy = False
            
            await self.healthy.set(healthy)
            await asyncio.sleep(1.0)
    
    async def get_health(self, timeout_sec: float = 3.0) -> Result:
        """获取客户端健康状态"""
        healthy = await self.healthy.get()
        return Result(success=True, message=f'Healthy check succeeded', data={"healthy": healthy})