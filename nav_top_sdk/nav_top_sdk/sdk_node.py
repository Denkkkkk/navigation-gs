"""
主节点模块
整合ROS2导航服务和FastAPI接口
"""

import rclpy
from rclpy.node import Node

from .nav_domain.ros2_navigation_service import NavService
from .api.navigation_api import NavigationAPI
import asyncio, threading

class TopSdkNode():
    """ 
    顶层SDK节点，使用组合模式集成ROS2导航服务和Web API
    
    这个类作为整个系统的入口点，负责：
    1. 创建和管理ROS2导航服务
    2. 创建和管理FastAPI Web服务器
    3. 提供统一的启动和管理接口
    """
    
    def __init__(self, node_name: str = 'top_sdk_node'):
        # 创建ROS2导航服务实例
        self.nav_srv = NavService(node_name)
        
        # 创建API服务实例
        self.api = NavigationAPI(self.nav_srv)
        
        self.nav_srv.ros_node.get_logger().info('Top SDK Node initialization completed.')
        self.nav_srv.ros_node.get_logger().info('Available services:')
        self.nav_srv.ros_node.get_logger().info('  - ROS2 Navigation Service: Ready')
        self.nav_srv.ros_node.get_logger().info('  - FastAPI Web Service: Ready to start')
    
    def start_web_server(self, host: str = "0.0.0.0", port: int = 8000):
        """启动Web服务器"""
        self.api.start_server(host, port)
        self.nav_srv.ros_node.get_logger().info(f'Web API server started on http://{host}:{port}')
        self.nav_srv.ros_node.get_logger().info('API Documentation available at:')
        self.nav_srv.ros_node.get_logger().info(f'  - Swagger UI: http://{host}:{port}/docs')
        self.nav_srv.ros_node.get_logger().info(f'  - ReDoc: http://{host}:{port}/redoc')
    
    async def start_service(self):
        await self.nav_srv.start()
    
    def get_ros2_node(self) -> Node:
        """获取导航服务实例，用于ROS2节点操作"""
        return self.nav_srv.ros_node


async def func(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    sdk_node = TopSdkNode()
    sdk_node.start_web_server(host="0.0.0.0", port=7999)
    
    await sdk_node.start_service()
    
    ros2_node = sdk_node.get_ros2_node()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros2_node,), daemon=True)
    spin_thread.start()
    
    try:
        while rclpy.ok():
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass
    except KeyboardInterrupt:
        print('\nShutting down gracefully...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

def main(): asyncio.run(func())

if __name__ == '__main__':
    main()
