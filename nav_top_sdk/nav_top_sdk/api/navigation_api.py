"""
FastAPI Web API模块
负责处理所有HTTP API相关功能
"""

from fastapi import FastAPI
import uvicorn
import threading

from ..nav_domain.ros2_navigation_service import NavService
from ..controller.navigation_controller import NavigationController
from ..request_model.base_request import NavigationGoal, Waypoint, CyclicPathRequest, NavigationModeRequest, SetMapRequest, SetRobotSizeRequest, ToggleRequest
from ..models.models import WebResponse

class NavigationAPI:
    """导航API类，处理所有HTTP接口"""
    
    def __init__(self, nav_srv: NavService):
        self.nav_srv = nav_srv
        
        # 创建HTTP控制器
        self.controller = NavigationController(self.nav_srv)
        
        self.app = FastAPI(
            title="ROS2 Navigation API", 
            version="1.0.0",
            description="提供ROS2导航系统的HTTP API接口"
        )
        
        # 注册所有路由
        self._register_routes()
    
    def _register_routes(self):
        """注册所有API路由"""
        
        @self.app.post("/", response_model=WebResponse)
        async def read_root():
            """根路径 - API状态检查"""
            return WebResponse(
                success=True,
                message="ROS2 Navigation API is running",
                data={"status": "ok"}
            )
        
        @self.app.post("/switch_mode", response_model=WebResponse)
        async def switch_navigation_mode(mode_req: NavigationModeRequest):
            """切换导航模式"""
            return await self.controller.switch_mode(mode_req)
        
        @self.app.post("/pause_nav", response_model=WebResponse)
        async def pause_nav():
            """暂停导航"""
            return await self.controller.pause_nav()
        
        @self.app.post("/resume_nav", response_model=WebResponse)
        async def resume_nav():
            """继续导航"""
            return await self.controller.resume_nav()
        
        @self.app.post("/navigate_to_pose", response_model=WebResponse)
        async def navigate_to_pose(goal: NavigationGoal):
            """导航到指定位姿"""
            return await self.controller.navigate_to_pose(goal)
        
        @self.app.post("/query_navigation_status", response_model=WebResponse)
        async def query_navigation_status():
            """查询导航状态"""
            return await self.controller.query_navigation_status()

        @self.app.post("/stop_navigation_task", response_model=WebResponse)
        async def stop_navigation_task():
            """停止导航"""
            return await self.controller.stop_navigation()
        
        @self.app.post("/reset_nav", response_model=WebResponse)
        async def reset_nav(goal: NavigationGoal):
            """导航到指定位姿"""
            return await self.controller.reset_nav(goal)
        
        @self.app.post("/start_cyclic_path", response_model=WebResponse)
        async def start_cyclic_path(cyclic_req: CyclicPathRequest):
            """启动循环路径任务"""
            return await self.controller.start_cyclic_path(cyclic_req)
        
        @self.app.post("/set_map", response_model=WebResponse)
        async def set_map(map_req: SetMapRequest):
            """设置地图"""
            return await self.controller.set_map(map_req)
        
        @self.app.post("/get_map_name", response_model=WebResponse)
        async def get_map_name():
            """获取当前地图名称"""
            return await self.controller.get_map_name()
        
        @self.app.post("/get_robot_size", response_model=WebResponse)
        async def get_robot_size():
            """获取机器人尺寸"""
            return await self.controller.get_robot_size()
        
        @self.app.post("/set_robot_size", response_model=WebResponse)
        async def set_robot_size(robot_size: SetRobotSizeRequest):
            """设置机器人尺寸"""
            return await self.controller.set_robot_size(robot_size)
        
        @self.app.post("/get_path", response_model=WebResponse)
        async def get_path():
            """获取路径"""
            return await self.controller.get_path()
        
        @self.app.post("/health", response_model=WebResponse)
        async def health_check():
            """健康检查"""
            return await self.controller.health_check()
        
        @self.app.post("/toggle_relocalization", response_model=WebResponse)
        async def toggle_relocalization(enable: ToggleRequest):
            """切换重定位状态"""
            return await self.controller.toggle_relocalization(enable)
        
        @self.app.post("/set_initial_pose", response_model=WebResponse)
        async def set_initial_pose(pose: Waypoint):
            """设置初始位姿"""
            return await self.controller.set_initial_pose(pose)
    
    def start_server(self, host: str = "0.0.0.0", port: int = 8000):
        """启动FastAPI服务器"""
        def run_server():
            uvicorn.run(self.app, host=host, port=port, log_level="info")
        
        api_thread = threading.Thread(target=run_server, daemon=True)
        api_thread.start()
        
        # 记录日志
        if hasattr(self.nav_srv.ros_node, 'get_logger'):
            self.nav_srv.ros_node.get_logger().info(f'FastAPI server started on http://{host}:{port}')
        else:
            print(f'FastAPI server started on http://{host}:{port}')
    
    def get_app(self):
        """获取FastAPI应用实例"""
        return self.app