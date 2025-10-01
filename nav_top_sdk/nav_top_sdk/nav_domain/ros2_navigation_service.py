"""
ROS2 导航服务模块
负责处理所有ROS2相关的导航功能
"""

import functools, os, asyncio
from typing import List, Union
from rclpy.node import Node

from ..models.models import NavServiceStatus, NavigationMode, GlobalNavStatus, LocalNavStatus
from ..models.models import Result, NavPose, RelocalMapStatus, ReLocalStatus
from ..models.models import MAP_MODES, NAVIGATING_MODES, PAUSED_MODES, ARRIVED_MODES, FAILED_MODES, VACANT_MODES
from .ros2_client import LocalNavigation, GlobalNavigation, NavModeSwitcher
from .ros2_client import RelocalizationManager, PathRecorder, MapManager2D, ParameterManager
from .utils import AsynVar, log_exceptions
from .base_client import ROSClient

def require_initialized(func):
    @functools.wraps(func)
    async def wrapper(self, *args, **kwargs):
        if await getattr(self, 'status', AsynVar(None)).get() is None:
            return Result(success=False, message='NavService not initialized')
        return await func(self, *args, **kwargs)
    return wrapper
    
class NavService(object):

    def __init__(self, node_name: str = 'ros2_sdk_client'):
        self.ros_node = Node(node_name)
        self.path_recorder = PathRecorder(self.ros_node)
        self.map_manager = MapManager2D(self.ros_node)
        self.parameter_manager = ParameterManager(self.ros_node)
        self.local_navigation = LocalNavigation(self.ros_node)
        self.global_navigation = GlobalNavigation(self.ros_node)
        self.nav_mode_switcher = NavModeSwitcher(self.ros_node)
        self.relocalization_manager = RelocalizationManager(self.ros_node)
        
        self.ros_clients: List[ROSClient] = [
            self.path_recorder,
            self.map_manager,
            self.parameter_manager,
            self.local_navigation,
            self.global_navigation,
            self.nav_mode_switcher,
            self.relocalization_manager
        ]

        self.status = AsynVar(None)
        self.healthy = AsynVar(False)
        self.relocal_status = AsynVar(ReLocalStatus.DISABLED)
        self.map_status = AsynVar(RelocalMapStatus.EMPTY)
        self.maps_dir = ""
        self.scene_name = ""

        self._cyclic_task = None
    
    async def start(self):
        self._initialization_task = asyncio.create_task(self._reset_to_default_state())
        self._update_nav_status_task = asyncio.create_task(self._update_nav_status_loop())
        self._update_relocal_status_task = asyncio.create_task(self._update_relocal_status_loop())
        self._health_check_task = asyncio.create_task(self._health_check_loop())
        self._map_dir_task = asyncio.create_task(self._initialize_map())
        await asyncio.gather(*(client.start() for client in self.ros_clients))
    
    @log_exceptions
    async def _reset_to_default_state(self):
        self.ros_node.get_logger().info("[NavService]: NavService initialzing to default mode: MAP_VACANT")

        while not (await self.nav_mode_switcher.map_mode()).success:
            self.ros_node.get_logger().info("[NavService]: Switching to map mode failed, retrying...")
        while not (await self.local_navigation.reset()).success:
            self.ros_node.get_logger().info("[NavService]: Local navigation reset failed, retrying...")
        while not (await self.global_navigation.reset()).success:
            self.ros_node.get_logger().info("[NavService]: Global navigation reset failed, retrying...")

        await self.status.set(NavServiceStatus.MAP_VACANT)
        await self.healthy.set(True)
        await self.relocal_status.set(ReLocalStatus.DISABLED)
        await self.map_status.set(RelocalMapStatus.EMPTY)

        self.ros_node.get_logger().info("[NavService]: NavService reset to default state completed")
        
    @log_exceptions
    async def _initialize_map(self):
        while True:
            result = await self.relocalization_manager.get_map()
            if not result.success:
                self.ros_node.get_logger().error(f"[NavService]: Failed to get relocalization map path: {result.message}")
                await asyncio.sleep(3.0)
                continue
            relocal_map = os.path.dirname(result.data['map'])
            self.ros_node.get_logger().info(f"[NavService]: Relocalization map path is: {relocal_map}")
        
            result = await self.map_manager.get_map_name()
            if not result.success:
                self.ros_node.get_logger().error(f"[NavService]: Failed to set map: {result.message}")
                await asyncio.sleep(3.0)
                continue
            nav_map_dir = os.path.dirname(result.data['map_dir'])
            self.ros_node.get_logger().info(f"[NavService]: Navigation map path is: {nav_map_dir}")
        
            if not relocal_map == nav_map_dir: 
                self.ros_node.get_logger().error("[NavService]: Relocalization map and navigation map does not match")
                continue
            
            self.ros_node.get_logger().info("[NavService]: Relocalization map and navigation map match!")
            
            self.maps_dir, self.scene_name = os.path.split(relocal_map)
            self.ros_node.get_logger().info(f"[NavService]: Maps directory is: {self.maps_dir}, scene name is: {self.scene_name}")
            break
        
        
    @log_exceptions
    async def _update_nav_status_loop(self):
        """定期更新导航服务状态"""
        while True:
            
            status = await self.status.get()
            if status is None:
                await asyncio.sleep(1.0)
                continue
            
            # update navigation status
            if status == NavServiceStatus.MAP_SINGLE_NAVIGATING:
                result = await self.global_navigation.get_state()
                if not result.success:
                    self.ros_node.get_logger().error(f"[NavService]: Failed to get global navigation state: {result.message}")
                    continue
                if result.data['state'] == GlobalNavStatus.ARRIVED:
                    status = NavServiceStatus.MAP_ARRIVED
                elif result.data['state'] == GlobalNavStatus.FAILED:
                    status = NavServiceStatus.MAP_FAILED

            elif status == NavServiceStatus.MAPFREE_SINGLE_NAVIGATING:
                result = await self.local_navigation.get_state()
                if not result.success:
                    self.ros_node.get_logger().error(f"[NavService]: Failed to get local navigation state: {result.message}")
                    continue
                if result.data['state'] == LocalNavStatus.ARRIVED:
                    status = NavServiceStatus.MAPFREE_ARRIVED
                elif result.data['state'] == LocalNavStatus.FAILED:
                    status = NavServiceStatus.MAPFREE_FAILED
            
            await self.status.set(status)
            await asyncio.sleep(1.0)
    
    @log_exceptions
    async def _update_relocal_status_loop(self):
        while True:
            result = await self.relocalization_manager.get_relocalization_status()
            if not result.success:
                self.ros_node.get_logger().error(f"[NavService]: Failed to get relocalization status: {result.message}")
                await asyncio.sleep(1.0)
                continue
            await self.relocal_status.set(result.data['relocal_status'])
            await self.map_status.set(result.data['map_status'])
            await asyncio.sleep(1.0)
            
    
    @log_exceptions
    async def _health_check_loop(self):
        """定期检查导航服务健康状态"""
        while True:
            healthy = True
            if await self.status.get() is None: healthy = False
            
            for client in self.ros_clients:
                if (await client.get_health()).data["healthy"]: continue
                
                healthy = False
                self.ros_node.get_logger().error(f"[NavService]: {client.client_name} is not healthy")
            
            await self.healthy.set(healthy)
            await asyncio.sleep(1.0)
        
    async def _cyclic_loop(self, poses: List[NavPose], planner_handle: Union[GlobalNavigation, LocalNavigation]) -> Result:
        """循环导航的内部协程"""
        try:
            current_done, current_goal = True, None
            while True:
                if not current_done:
                    result = await planner_handle.get_state()

                    if not result.success:
                        return Result(success=False, message=f'Failed to get navigation state: {result.message}')
                    
                    if result.data['state'] in [GlobalNavStatus.ARRIVED, LocalNavStatus.ARRIVED]:
                        current_done = True
                        self.ros_node.get_logger().info(f"[NavService]: Reached goal: {current_goal}")

                    if result.data['state'] in [GlobalNavStatus.FAILED, LocalNavStatus.FAILED]:
                        return Result(success=False, message=f'Failed to navigate to: {current_goal}')
                    
                    await asyncio.sleep(1.0)
                    continue

                if not poses: break # poses finished

                current_goal = poses.pop(0)
                self.ros_node.get_logger().info(f"[NavService]: Navigating to next goal: {current_goal}")
                result = await planner_handle.nav_to_pose(current_goal, timeout_sec=3.0)
                
                if not result.success:
                    return Result(success=False, message=f'Failed to navigate to pose: {result.message}')

                current_done = False
                await asyncio.sleep(1.0)
            
            return Result(success=True, message='Cyclic navigation completed successfully')

        except asyncio.CancelledError:
            return Result(success=True, message='Cyclic navigation cancelled')
        except Exception as e:
            return Result(success=False, message=f'Error during cyclic navigation: {str(e)}')
    
    async def _cyclic_finish_callback(self, future: asyncio.Future):
        """循环导航完成后的回调"""
        status = await self.status.get()
        if future.cancelled():
            self.ros_node.get_logger().info("[NavService]: Cyclic navigation task was cancelled")
            if status == NavServiceStatus.MAP_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAP_FAILED)
            elif status == NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAPFREE_FAILED)
        elif future.exception():
            self.ros_node.get_logger().error(f"[NavService]: Cyclic navigation task failed with exception: {future.exception()}")
            if status == NavServiceStatus.MAP_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAP_FAILED)
            elif status == NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAPFREE_FAILED)
        elif not future.result().success:
            self.ros_node.get_logger().error(f"[NavService]: Cyclic navigation task failed: {future.result().message}")
            if status == NavServiceStatus.MAP_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAP_FAILED)
            elif status == NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAPFREE_FAILED)
        else:
            self.ros_node.get_logger().info("[NavService]: Cyclic navigation task completed successfully")
            if status == NavServiceStatus.MAP_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAP_ARRIVED)
            elif status == NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAPFREE_ARRIVED)
        self._cyclic_task = None
        
    @require_initialized
    async def _cyclic_wrapper(self, poses: List[NavPose], planner_handle: Union[GlobalNavigation, LocalNavigation]) -> Result:
        """包装循环导航的协程"""
        task = asyncio.create_task(self._cyclic_loop(poses, planner_handle))
        try:
            result = await task
            return result
        finally:
            # Always run finish callback, even if cancelled or errored
            await self._cyclic_finish_callback(task)
        
    @require_initialized
    async def switch_mode(self, target_mode: NavigationMode) -> Result:
        """切换导航模式"""
        status = await self.status.get()
        if status not in [NavServiceStatus.MAP_VACANT, NavServiceStatus.MAPFREE_VACANT]:
            return Result(success=False, message='Cannot switch mode when not in vacant state')
        if target_mode == NavigationMode.MAP and status == NavServiceStatus.MAP_VACANT:
            return Result(success=True, message='Already in MAP mode')
        if target_mode == NavigationMode.MAP_FREE and status == NavServiceStatus.MAPFREE_VACANT:
            return Result(success=True, message='Already in MAP_FREE mode')
        
        if target_mode == NavigationMode.MAP:
            result = await self.nav_mode_switcher.map_mode()
            if not result.success:
                return Result(success=False, message=f'Failed to switch to MAP mode: {result.message}')
            await self.status.set(NavServiceStatus.MAP_VACANT)
            return Result(success=True, message='Switched to MAP mode successfully')
        
        result = await self.nav_mode_switcher.mapfree_mode()
        if not result.success:
            return Result(success=False, message=f'Failed to switch to MAPFREE mode: {result.message}')
        await self.status.set(NavServiceStatus.MAPFREE_VACANT)
        return Result(success=True, message='Switched to MAPFREE mode successfully')
        
    @require_initialized
    async def nav_to_pose(self, speed: float, pose: NavPose, timeout_sec: float = 3.0) -> Result:
        """导航到指定位姿"""
        
        result = await self.reset()
        if not result.success:
            return Result(success=False, message=f'Failed to reset: {result.message}')

        result = await self.set_speed(speed)
        if not result.success:
            return Result(success=False, message=f'Failed to set speed: {result.message}')

        if await self.status.get() == NavServiceStatus.MAP_VACANT:
            result = await self.global_navigation.nav_to_pose(pose, timeout_sec)
            if not result.success:
                return Result(success=False, message=f'Failed to navigate to pose: {result.message}')
            await self.status.set(NavServiceStatus.MAP_SINGLE_NAVIGATING)
            return Result(success=True, message='Navigation to pose started successfully')
        
        result = await self.local_navigation.nav_to_pose(pose, timeout_sec)
        if not result.success:
            return Result(success=False, message=f'Failed to navigate to pose: {result.message}')
        await self.status.set(NavServiceStatus.MAPFREE_SINGLE_NAVIGATING)
        return Result(success=True, message='Navigation to pose started successfully')
    
    @require_initialized
    async def multiple_nav(self, speed: float, poses: List[NavPose], cycles: int, timeout_sec: float = 3.0) -> Result:
        """多目标导航"""
        
        result = await self.reset()
        if not result.success:
            return Result(success=False, message=f'Failed to reset: {result.message}')
        
        if len(poses) == 0:
            return Result(success=False, message='No poses provided for multiple navigation')
        if cycles <= 0:
            return Result(success=False, message='Cycles must be greater than 0 for multiple navigation')
        
        result = await self.set_speed(speed)
        if not result.success:
            return Result(success=False, message=f'Failed to set speed for multiple navigation: {result.message}')

        poses = cycles * poses
        
        if await self.status.get() == NavServiceStatus.MAP_VACANT:
            self._cyclic_task = asyncio.create_task(self._cyclic_wrapper(poses, self.global_navigation))
            await self.status.set(NavServiceStatus.MAP_MULTIPLE_NAVIGATING)
            return Result(success=True, message='Multiple navigation started successfully')
        
        if await self.status.get() == NavServiceStatus.MAPFREE_VACANT:
            self._cyclic_task = asyncio.create_task(self._cyclic_wrapper(poses, self.local_navigation))
            await self.status.set(NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING)
            return Result(success=True, message='Multiple navigation started successfully')
        
        return Result(success=False, message='Cannot start multiple navigation when not in vacant.')
    
    @require_initialized
    async def pause(self) -> Result:
        """暂停导航"""
        status = await self.status.get()
        if status in [NavServiceStatus.MAP_SINGLE_NAVIGATING, NavServiceStatus.MAP_MULTIPLE_NAVIGATING]:
            result = await self.global_navigation.pause(timeout_sec=3.0)
            if not result.success:
                return Result(success=False, message=f'Failed to pause navigation: {result.message}')
            if status == NavServiceStatus.MAP_SINGLE_NAVIGATING: await self.status.set(NavServiceStatus.MAP_SINGLE_PAUSED)
            elif status == NavServiceStatus.MAP_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAP_MULTIPLE_PAUSED)
            return Result(success=True, message='Navigation paused successfully')
        
        if status in [NavServiceStatus.MAPFREE_SINGLE_NAVIGATING, NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING]:
            result = await self.local_navigation.pause(timeout_sec=3.0)
            if not result.success:
                return Result(success=False, message=f'Failed to pause local navigation: {result.message}')
            if status == NavServiceStatus.MAPFREE_SINGLE_NAVIGATING: await self.status.set(NavServiceStatus.MAPFREE_SINGLE_PAUSED)
            elif status == NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING: await self.status.set(NavServiceStatus.MAPFREE_MULTIPLE_PAUSED)
            return Result(success=True, message='Local navigation paused successfully')
        
        return Result(success=False, message='Cannot pause navigation when not navigating')
    
    @require_initialized
    async def resume(self) -> Result:
        """恢复导航"""
        status = await self.status.get()
        if status in [NavServiceStatus.MAP_SINGLE_PAUSED, NavServiceStatus.MAP_MULTIPLE_PAUSED]:
            result = await self.global_navigation.resume(timeout_sec=3.0)
            if not result.success:
                return Result(success=False, message=f'Failed to resume navigation: {result.message}')
            if status == NavServiceStatus.MAP_SINGLE_PAUSED: await self.status.set(NavServiceStatus.MAP_SINGLE_NAVIGATING)
            elif status == NavServiceStatus.MAP_MULTIPLE_PAUSED: await self.status.set(NavServiceStatus.MAP_MULTIPLE_NAVIGATING)
            return Result(success=True, message='Navigation resumed successfully')
        
        if status in [NavServiceStatus.MAPFREE_SINGLE_PAUSED, NavServiceStatus.MAPFREE_MULTIPLE_PAUSED]:
            result = await self.local_navigation.resume(timeout_sec=3.0)
            if not result.success:
                return Result(success=False, message=f'Failed to resume local navigation: {result.message}')
            if status == NavServiceStatus.MAPFREE_SINGLE_PAUSED: await self.status.set(NavServiceStatus.MAPFREE_SINGLE_NAVIGATING)
            elif status == NavServiceStatus.MAPFREE_MULTIPLE_PAUSED: await self.status.set(NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING)
            return Result(success=True, message='Local navigation resumed successfully')
        
        return Result(success=False, message='Cannot resume navigation when not paused')

    @require_initialized
    async def reset(self) -> Result:
        """重置导航服务"""
        if self._cyclic_task is not None:
            self._cyclic_task.cancel()
            try: await self._cyclic_task
            except asyncio.CancelledError: pass
            
        result = await self.global_navigation.reset()
        if not result.success:
            return Result(success=False, message=f'Failed to reset global navigation: {result.message}')
        result = await self.local_navigation.reset()
        if not result.success:
            return Result(success=False, message=f'Failed to reset local navigation: {result.message}')
        
        result = await self.nav_mode_switcher.get_mode()
        if not result.success:
            return Result(success=False, message=f'Failed to get current navigation mode: {result.message}')
        
        if result.data['mode'] == NavigationMode.MAP: await self.status.set(NavServiceStatus.MAP_VACANT)
        elif result.data['mode'] == NavigationMode.MAP_FREE: await self.status.set(NavServiceStatus.MAPFREE_VACANT)

        return Result(success=True, message=f'NavService reset to {(await self.status.get()).name} state successfully.')
        
    @require_initialized
    async def set_speed(self, speed: float) -> Result:
        """设置导航速度"""
        if await self.status.get() in MAP_MODES:
            result = await self.global_navigation.set_speed(speed)
            if not result.success:
                return Result(success=False, message=f'Failed to set speed for global navigation: {result.message}')
            return Result(success=True, message='Speed set for global navigation successfully')
        
        # map-free case
        result = await self.local_navigation.set_speed(speed)
        if not result.success:
            return Result(success=False, message=f'Failed to set speed for local navigation: {result.message}')
        return Result(success=True, message='Speed set for local navigation successfully')
    
    @require_initialized
    async def set_map(self, map_name: str) -> Result:
        """设置地图"""
        map_path = os.path.join(self.maps_dir, map_name, map_name)
        result2d = await self.map_manager.set_map(map_path + ".yaml")
        result3d = await self.relocalization_manager.set_map(map_path + ".pcd")
        if result2d.success and result3d.success:
            return Result(success=True, message='Map set successfully')
        
        self.maps_dir, self.scene_name = "", ""
        
        return Result(
            success=False, 
            message= f'Failed to set map: '
            + f'2d:{result2d.message if not result2d.success else "successful"}, '
            + f'3d:{result3d.message if not result3d.success else "successful"}. '
            + f'Please set again, or it will lead to undefined behavior.')
    
    @require_initialized
    async def get_map(self) -> Result:
        """获取当前地图"""
        result = await self.map_manager.get_map_name()
        if not result.success:
            return Result(success=False, message=f'Failed to get map: {result.message}')
        return Result(success=True, message='Map retrieved successfully', data=result.data)
    
    @require_initialized
    async def set_robot_size(self, width: float, length: float) -> Result:
        """设置机器人尺寸"""
        result = await self.parameter_manager.set_robot_size(width, length)
        if not result.success:
            return Result(success=False, message=f'Failed to set robot size: {result.message}')
        return Result(success=True, message='Robot size set successfully')
    
    @require_initialized
    async def toggle_relocalization(self, enable: bool) -> Result:
        """启用或禁用重定位功能"""
        result = await self.relocalization_manager.toggle_relocalization(enable)
        if not result.success:
            return Result(success=False, message=f'Failed to toggle relocalization: {result.message}')
        return Result(success=True, message='Relocalization toggled successfully')
    
    @require_initialized
    async def set_initial_pose(self, pose: NavPose) -> Result:
        """设置初始位姿"""
        result = await self.relocalization_manager.set_initial_pose(pose)
        if not result.success:
            return Result(success=False, message=f'Failed to set initial pose: {result.message}')
        return Result(success=True, message='Initial pose set successfully')
    
    @require_initialized
    async def get_robot_size(self) -> Result:
        """获取机器人尺寸"""
        result = await self.parameter_manager.get_robot_size()
        if not result.success:
            return Result(success=False, message=f'Failed to get robot size: {result.message}')
        return Result(success=True, message='Robot size retrieved successfully', data=result.data)
    
    @require_initialized
    async def get_path(self) -> Result:
        """获取当前导航路径"""
        result = await self.path_recorder.get_path()
        if not result.success:
            return Result(success=False, message=f'Failed to get path: {result.message}')
        return Result(success=True, message='Path retrieved successfully', data=result.data)
    
    @require_initialized
    async def get_health(self) -> Result:
        """获取导航服务健康状态"""
        return Result(success=True, message="Health check succeeded", data={"healthy": await self.healthy.get()})
    
    @require_initialized
    async def get_status(self) -> Result:
        """获取导航服务状态"""
        status = await self.status.get()
        if status is None:
            return Result(success=False, message='NavService status is not initialized')
        
        map_mode = "map" if status in MAP_MODES else "map-free"
        
        if status in NAVIGATING_MODES: navigation_mode = "navigating"
        elif status in PAUSED_MODES: navigation_mode = "paused"
        elif status in ARRIVED_MODES: navigation_mode = "arrived"
        elif status in VACANT_MODES: navigation_mode = "vacant"
        elif status in FAILED_MODES: navigation_mode = "failed"
        
        return Result(
            success=True, 
            message='NavService status retrieved successfully', 
            data={
                "map": map_mode, 
                "status": navigation_mode,
                "map_loading": (await self.map_status.get()).value,
                "relocalization": (await self.relocal_status.get()).value
            }
        )
        