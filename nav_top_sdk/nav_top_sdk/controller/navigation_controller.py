"""
导航应用服务和HTTP控制器
分为应用层（业务逻辑）和接口层（HTTP处理）
"""

from typing import Dict, Any, List, Tuple, Optional
from time import perf_counter
import functools
from fastapi import HTTPException

from ..request_model.base_request import NavigationGoal, Waypoint, CyclicPathRequest, NavigationModeRequest, StatusResponse, SetMapRequest, SetRobotSizeRequest, ToggleRequest
from ..nav_domain.ros2_navigation_service import NavService
from ..models.models import Result, NavigationMode, NavPose, WebResponse

import traceback

def time_calling(func):
    @functools.wraps(func)
    async def wrapper(self, *args, **kwargs):
        start = perf_counter()
        result = await func(self, *args, **kwargs)
        end = perf_counter()
        print(f"Calling controller function `{func.__name__}` took {end - start:.6f} seconds")
        return result
    return wrapper

class NavigationController:
    """导航HTTP控制器 - 处理HTTP请求/响应转换"""
    
    def __init__(self, nav_service: NavService):
        self.nav_service = nav_service
        
    async def wrap_service_call(self, call_func, *args, **kwargs):
        """
        调用服务并统一返回 WebResponse
        """
        try:
            result = await call_func(*args, **kwargs)
            if not result.success:
                return WebResponse(
                    code=3000,
                    message=f"Operation failed: {result.message}",
                    data=result.data
                )
            return WebResponse(
                code=0,
                message="success",
                data=result.data
            )
        except Exception as e:
            
            traceback.print_exc()
            
            return WebResponse(
                code=5000,
                message=f"Service error: {str(e)}"
            )
    
    @time_calling
    async def switch_mode(self, mode_req: NavigationModeRequest) -> WebResponse:
        return await self.wrap_service_call(
            self.nav_service.switch_mode,
            NavigationMode(mode_req.mode)
        )
    
    @time_calling
    async def pause_nav(self) -> WebResponse:
        """暂停导航"""
        return await self.wrap_service_call(self.nav_service.pause)
    
    @time_calling
    async def resume_nav(self) -> WebResponse:
        """继续导航"""
        return await self.wrap_service_call(self.nav_service.resume)
    
    @time_calling
    async def navigate_to_pose(self, goal: NavigationGoal) -> WebResponse:
        """导航到指定位姿"""
        return await self.wrap_service_call(
            self.nav_service.nav_to_pose,
            goal.speed,
            NavPose(goal.x, goal.y, goal.theta, goal.frame_id)
        )
    
    @time_calling
    async def query_navigation_status(self) -> WebResponse:
        """查询导航状态 - HTTP接口"""
        return await self.wrap_service_call(self.nav_service.get_status)
    
    @time_calling
    async def stop_navigation(self) -> WebResponse:
        """停止导航"""
        return await self.wrap_service_call(self.nav_service.reset)
    
    @time_calling
    async def reset_nav(self) -> WebResponse:
        """重置导航"""
        return await self.wrap_service_call(self.nav_service.reset)
    
    @time_calling
    async def start_cyclic_path(self, cyclic_req: CyclicPathRequest) -> StatusResponse:
        """启动循环路径任务 - HTTP接口"""
        return await self.wrap_service_call(
            self.nav_service.multiple_nav,
            cyclic_req.speed,
            self.waypoints_2_navPoses(cyclic_req.waypoints, cyclic_req.frame_id),
            cyclic_req.cycles
        )
    
    @time_calling
    async def health_check(self) -> WebResponse:
        """健康检查"""
        return await self.wrap_service_call(self.nav_service.get_health)
    
    @time_calling
    async def set_map(self, map_req: SetMapRequest) -> WebResponse:
        """设置地图"""
        return await self.wrap_service_call(
            self.nav_service.set_map,
            map_req.scene_name
        )
    
    @time_calling
    async def get_map_name(self) -> WebResponse:
        """获取当前地图名称"""
        return await self.wrap_service_call(
            self.nav_service.get_map
        )
    
    @time_calling
    async def get_robot_size(self) -> WebResponse:
        """获取机器人尺寸"""
        return await self.wrap_service_call(
            self.nav_service.get_robot_size
        )
    
    @time_calling
    async def set_robot_size(self, robot_size: SetRobotSizeRequest) -> WebResponse:
        """设置机器人尺寸"""
        return await self.wrap_service_call(
            self.nav_service.set_robot_size,
            robot_size.width,
            robot_size.length
        )
    
    @time_calling
    async def get_path(self) -> WebResponse:
        """获取路径"""
        return await self.wrap_service_call(
            self.nav_service.get_path
        )
    
    @time_calling
    async def toggle_relocalization(self, enable: ToggleRequest) -> WebResponse:
        """切换重定位状态"""
        return await self.wrap_service_call(
            self.nav_service.toggle_relocalization,
            enable.enable
        )
    
    @time_calling
    async def set_initial_pose(self, pose: Waypoint) -> WebResponse:
        """设置初始位姿"""
        return await self.wrap_service_call(
            self.nav_service.set_initial_pose,
            self.waypoints_2_navPoses([pose], "map")[0]
        )
    
    @staticmethod
    def waypoints_2_navPoses(waypoints: List[Waypoint], frame_id: str) -> List[NavPose]:
        """将 Waypoint 列表转换为 NavPose 列表"""
        return [NavPose(wp.x, wp.y, wp.theta, frame_id) for wp in waypoints]
