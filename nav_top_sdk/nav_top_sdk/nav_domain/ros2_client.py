import os, ast, asyncio
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import LoadMap, SetInitialPose
from std_srvs.srv import Trigger, SetBool
from nav2_msgs.action import NavigateToPose

from controller_interfaces.srv import CustomControllerQuery, NavModeSwitch, SetGoalPose
from small_gicp_relocalization.srv import GetString, SetString
from ..models.models import Result, NavigationMode, NavPose, LocalNavStatus, GlobalNavStatus, NavigationMode, ReLocalStatus, RelocalMapStatus
from .utils import AsynVar, ParamUtils, StringSrvUtils, call_trigger, log_exceptions
from .base_client import ROSClient

class PathRecorder(ROSClient):

    def __init__(self, ros_node_obj: Node):
        super().__init__(ros_node_obj, 'PathRecorder')
        self.global_path = []
        self.local_path = []
        # subscription
        self.global_path_sub = self.ros_node.create_subscription(Path, "plan", self._global_path_callback, 10)
        self.local_path_sub = self.ros_node.create_subscription(Path, "visualization/neupan_plan", self._local_path_callback, 10)
        # log
        self.ros_node.get_logger().info('[PathRecorder]: PathRecorder initialized and subscriptions created')
    
    async def start(self): await super().start()
    
    def _global_path_callback(self, msg: Path):
        """全局路径回调"""
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
    
    def _local_path_callback(self, msg: Path):
        """局部路径回调"""
        self.local_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
    
    async def get_path(self, timeout_sec: float = 3.0) -> Result:
        """获取全局路径"""
        return Result(success=True, message='[PathRecorder]: Global path retrieved successfully', data={"global": self.global_path, "local": self.local_path})

class MapManager2D(ROSClient):

    def __init__(self, ros_node_obj: Node):
        super().__init__(ros_node_obj, 'MapManager2D')
        # Initialization
        self.ros_node = ros_node_obj
        self._init_service_clients()
        self._wait_for_services()
    
    async def start(self): await super().start()

    def _init_service_clients(self):
        """初始化地图相关的服务客户端"""
        self.map_get_params_client = self.ros_node.create_client(GetParameters, 'map_server/get_parameters')
        self.map_set_params_client = self.ros_node.create_client(SetParameters, 'map_server/set_parameters')
        self.nav2_load_map_client = self.ros_node.create_client(LoadMap, 'map_server/load_map')
        
        self._register_services([
            self.map_get_params_client,
            self.map_set_params_client,
            self.nav2_load_map_client
        ])
    
    async def _get_current_map(self, timeout_sec: float = 3.0) -> str:
        
        success, values = await ParamUtils.get_params(self.map_get_params_client, ['yaml_filename'], timeout_sec)

        if not success:
            self.ros_node.get_logger().error('[MapManager]: Failed to get map parameters')
            return None
        
        if values[0].type != ParameterType.PARAMETER_STRING:
            self.ros_node.get_logger().error('[MapManager]: Map parameter is not a string or not found')
            return None
        
        current_yaml_filename = values[0].string_value

        self.ros_node.get_logger().info(f'[MapManager]: Current map path: {current_yaml_filename}')

        return current_yaml_filename
    
    async def _set_new_map(self, map_dir: str, timeout_sec: float = 3.0) -> bool:
        """设置当前地图目录和场景名称"""

        if not os.path.exists(map_dir):
            self.ros_node.get_logger().error(f'[MapManager]: Map file does not exist: {map_dir}')
            return False
        
        self.ros_node.get_logger().info(f'[MapManager]: Setting map: {map_dir}')

        if not await ParamUtils.set_params(
            self.map_set_params_client, ['yaml_filename'],
            [ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=map_dir)],
            timeout_sec
        ):
            self.ros_node.get_logger().error('[MapManager]: Failed to set map parameters')
            return False
        
        self.ros_node.get_logger().info('[MapManager]: Map parameters set successfully')
        
        # Set map server for new map
        map_request = LoadMap.Request()
        map_request.map_url = map_dir
        
        map_future = self.nav2_load_map_client.call_async(map_request)
        
        try:
            await asyncio.wait_for(map_future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.ros_node.get_logger().error(f'[MapManager]: Set map timeout after {timeout_sec}s')
            return False
        
        if map_future.result() is None:
            self.ros_node.get_logger().error('[MapManager]: Failed to set map')
            return False
        if map_future.result().result:
            self.ros_node.get_logger().error(f'[MapManager]: Set map failed with LoadMap_Response code: {map_future.result().result}')
            return False
        
        self.ros_node.get_logger().info(f'[MapManager]: Map set successfully: {map_dir}')
        return True
    
    async def set_map(self, map_dir: str, timeout_sec: float = 3.0) -> Result:
        """设置地图"""
        if not await self._set_new_map(map_dir, timeout_sec):
            return Result(success=False, message='Failed to set new map')
        return Result(success=True, message='Map set successfully')
    
    async def get_map_name(self, timeout_sec: float = 3.0) -> Result:
        """获取当前地图名称"""
        map_dir = await self._get_current_map(timeout_sec)
        if not map_dir:
            return Result(success=False, message='Failed to get current map directory or scene name')
        return Result(success=True, message='Current map name retrieved successfully', data={"map_dir": map_dir})
        

class ParameterManager(ROSClient):
    def __init__(self, ros_node_obj: Node):
        super().__init__(ros_node_obj, 'ParameterManager')
        self.ros_node = ros_node_obj
        self._init_service_clients()
        self._wait_for_services()
    
    async def start(self): await super().start()

    def _init_service_clients(self):
        """初始化参数相关的服务客户端"""
        self.neupan_get_params_client = self.ros_node.create_client(GetParameters, 'neupan_node/get_parameters')
        self.neupan_set_params_client = self.ros_node.create_client(SetParameters, 'neupan_node/set_parameters')
        self.global_costmap_get_params_client = self.ros_node.create_client(GetParameters, 'global_costmap/global_costmap/get_parameters')
        self.global_costmap_set_params_client = self.ros_node.create_client(SetParameters, 'global_costmap/global_costmap/set_parameters')
        self.local_costmap_set_params_client = self.ros_node.create_client(SetParameters, 'local_costmap/local_costmap/set_parameters')
        
        self._register_services([
            self.neupan_get_params_client,
            self.neupan_set_params_client,
            self.global_costmap_get_params_client,
            self.global_costmap_set_params_client,
            self.local_costmap_set_params_client
        ])
    
    async def _get_robot_size(self, timeout_sec: float = 3.0) -> Tuple[bool, float, float]:
        """获取当前的机器人大小"""
        success, values = await ParamUtils.get_params(self.global_costmap_get_params_client, ['footprint'], timeout_sec)
        
        if not success:
            self.ros_node.get_logger().error('[ParameterManager]: Failed to get global_costmap footprint parameter')
            return False, None, None
        
        footprint = values[0].string_value

        if not footprint:
            self.ros_node.get_logger().error('[ParameterManager]: No footprint parameter found')
            return False, None, None
        
        self.ros_node.get_logger().info(f'[ParameterManager]: Current robot footprint: {footprint}')

        footprint = ast.literal_eval(footprint)

        width = max([point[1] for point in footprint]) - min([point[1] for point in footprint])
        length = max([point[0] for point in footprint]) - min([point[0] for point in footprint])

        return True, width, length

    async def _set_robot_size(self, width: float, length: float, timeout_sec: float = 3.0) -> bool:
        """设置当前的机器人大小
        1. 将机器人尺寸转换为四个点的坐标表示
        2. 设置 global_costmap 和 local_costmap 的 footprint 参数
        3. 根据当前 neupan 当前的尺寸进行对比，并确定 neupan 的碰撞参数
        4. 通过参数服务器，设置 neupan 的碰撞参数
        5. 通过读取 yaml 文件和重写实现修改的记忆
        """
        # 1. 将机器人尺寸转换为四个点的坐标表示
        footprint = f"[[{length/2}, {width/2}], [{length/2}, -{width/2}], [-{length/2}, -{width/2}], [-{length/2}, {width/2}]]"

        # 2. 设置 global_costmap 和 local_costmap 的 footprint 参数
        if not await ParamUtils.set_params(
            self.global_costmap_set_params_client, ['footprint'],
            [ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=footprint)], 
            timeout_sec
        ):
            self.ros_node.get_logger().error('[ParameterManager]: Failed to set global_costmap footprint')
            return False
        
        if not await ParamUtils.set_params(
            self.local_costmap_set_params_client, ['footprint'],
            [ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=footprint)], 
            timeout_sec
        ):
            self.ros_node.get_logger().error('[ParameterManager]: Failed to set local_costmap footprint')
            return False

        # 3. 根据当前 neupan 当前的尺寸进行对比，并确定 neupan 的碰撞参数
        success, values = await ParamUtils.get_params(
            self.neupan_get_params_client, ['d_min', 'd_max', 'robot_size'], timeout_sec
        )

        if not success:
            self.ros_node.get_logger().error('[ParameterManager]: Failed to get neupan parameters')
            return False
        
        d_min, d_max = values[0].double_value, values[1].double_value
        neupan_length, neupan_width = ast.literal_eval(values[2].string_value)
        self.ros_node.get_logger().info(f'[ParameterManager]: Current Neupan size: length={neupan_length}, width={neupan_width}, d_min={d_min}, d_max={d_max}')

        d_min = max(length - neupan_length, width - neupan_width, 0.2) / 2.0
        d_max = d_min + 0.05

        self.ros_node.get_logger().info(f'[ParameterManager]: Calculated Neupan collision parameters: d_min={d_min}, d_max={d_max}')

        # 4. 通过参数服务器，设置 neupan 的碰撞参数
        if not await ParamUtils.set_params(
            self.neupan_set_params_client, ['d_min', 'd_max'],
            [ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=d_min),
             ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=d_max)], 
            timeout_sec
        ):
            self.ros_node.get_logger().error('[ParameterManager]: Failed to set Neupan collision parameters')
            return False
        
        # 5. 通过读取 yaml 文件和重写实现修改的记忆
        nav_bringup_package_dir = get_package_share_directory('nav_bringup')
        nav_config_path = os.path.join(nav_bringup_package_dir, 'config', 'nav2_params.yaml')
        if not os.path.exists(nav_config_path):
            self.ros_node.get_logger().error(f'[ParameterManager]: Nav2 config file not found: {nav_config_path}')
            return False
        
        ParamUtils.update_yaml_param(nav_config_path, ["local_costmap", "local_costmap", "ros__parameters", "footprint"], footprint)
        ParamUtils.update_yaml_param(nav_config_path, ["global_costmap", "global_costmap", "ros__parameters", "footprint"], footprint)

        self.ros_node.get_logger().info(f'[ParameterManager]: Robot size written to {nav_config_path} successfully')

        neupan_controller_package_dir = get_package_share_directory('neupan_controller')
        neupan_config_path = os.path.join(neupan_controller_package_dir, 'config', 'neupan_planner.yaml')

        if not os.path.exists(neupan_config_path):
            self.ros_node.get_logger().error(f'[ParameterManager]: Neupan config file not found: {neupan_config_path}')
            return False
        
        ParamUtils.update_yaml_param(neupan_config_path, ["adjust", "d_max"], d_max)
        ParamUtils.update_yaml_param(neupan_config_path, ["adjust", "d_min"], d_min)

        self.ros_node.get_logger().info(f'[ParameterManager]: Neupan parameters written to {neupan_config_path} successfully')
        
        return True
    
    async def get_robot_size(self, timeout_sec: float = 3.0) -> Result:
        """获取当前的机器人大小"""
        success, width, length = await self._get_robot_size(timeout_sec)
        if not success:
            return Result(success=False, message='Failed to get robot size')
        
        return Result(success=True, message='Robot size retrieved successfully', data={"width": width, "length": length})
    
    async def set_robot_size(self, width: float, length: float, timeout_sec: float = 3.0) -> Result:
        """设置当前的机器人大小"""
        if width <= 0 or length <= 0:
            return Result(success=False, message='Invalid robot size, must be positive')
        
        if not await self._set_robot_size(width, length, timeout_sec):
            return Result(success=False, message='Failed to set robot size')
        
        return Result(success=True, message='Robot size set successfully', data={"width": width, "length": length})
        

class LocalNavigation(ROSClient):

    def __init__(self, ros_node_obj: Node):
        super().__init__(ros_node_obj, 'LocalNavigation')

        self._init_service_clients()
        self._wait_for_services()

        self._state = AsynVar(LocalNavStatus.READY)
        
    async def start(self):
        await super().start()
        self._state_update_task = asyncio.create_task(self._state_update_loop())

    def _init_service_clients(self):
        self.neupan_query_client = self.ros_node.create_client(CustomControllerQuery, 'neupan_controller_query')
        self.neupan_set_pose_client = self.ros_node.create_client(SetGoalPose, 'neupan_set_goal')
        self.neupan_reset_client = self.ros_node.create_client(Trigger, 'neupan_reset')
        self.neupan_pause_client = self.ros_node.create_client(Trigger, 'neupan_pause')
        self.neupan_resume_client = self.ros_node.create_client(Trigger, 'neupan_resume')
        self.neupan_set_param_client = self.ros_node.create_client(SetParameters, 'neupan_node/set_parameters')
        
        self._register_services([
            self.neupan_query_client,
            self.neupan_set_pose_client,
            self.neupan_reset_client,
            self.neupan_pause_client,
            self.neupan_resume_client,
            self.neupan_set_param_client
        ])
    
    @log_exceptions
    async def _state_update_loop(self):
        state_dict = {
            "ready": LocalNavStatus.READY,
            "navigating": LocalNavStatus.NAVIGATING,
            "failed": LocalNavStatus.FAILED,
            "arrived": LocalNavStatus.ARRIVED,
            "paused": LocalNavStatus.PAUSED
        }
        request = CustomControllerQuery.Request()

        while rclpy.ok():
            future = self.neupan_query_client.call_async(request)
            try:
                await asyncio.wait_for(future, timeout=1.0)
            except asyncio.TimeoutError:
                self.ros_node.get_logger().error('[LocalNavigation]: Neupan query service call timeout')
                continue

            response = future.result()
            if response is None:
                self.ros_node.get_logger().error('[LocalNavigation]: Neupan query service call failed')
                await asyncio.sleep(1.0)
                continue
            
            try: 
                await self._state.set(state_dict[response.status])
            except Exception as e: 
                self.ros_node.get_logger().error(f"[LocalNavigation]: Local update state failed: {e}")
            
            await asyncio.sleep(1.0)
    
    async def _set_reference_speed(self, speed: float, timeout_sec: float = 3.0) -> bool:
        """设置neupan参考速度"""
        self.ros_node.get_logger().info(f'[LocalNavigation]: Setting Neupan reference speed to: {speed} m/s')

        if not await ParamUtils.set_params(
            self.neupan_set_param_client, ['reference_speed'],
            [ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=speed)],
            timeout_sec
        ):
            self.ros_node.get_logger().error('[LocalNavigation]: Failed to set Neupan reference speed')
            return False
        
        self.ros_node.get_logger().info(f'[LocalNavigation]: Neupan reference speed set to: {speed} m/s')
        return True
    
    async def _nav_to_pose(self, target_pose: NavPose, timeout_sec: float = 3.0) -> bool:
        """设置neupan目标位姿"""
        self.ros_node.get_logger().info(f'[LocalNavigation]: Setting Neupan goal pose: {target_pose}')
        
        request = SetGoalPose.Request()
        request.goal_pose = target_pose.toPoseStamped(self.ros_node.get_clock().now().to_msg())
        
        future = self.neupan_set_pose_client.call_async(request)
        
        try:
            await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.ros_node.get_logger().error(f'[LocalNavigation]: Set Neupan goal pose timeout after {timeout_sec}s')
            return False
        
        if future.result() is None:
            self.ros_node.get_logger().error('[LocalNavigation]: Failed to set Neupan goal pose')
            return False
        
        response: SetGoalPose.Response = future.result()
        if not response.success:
            self.ros_node.get_logger().error(f'[LocalNavigation]: Set Neupan goal pose failed: {response.message}')
            return False
        
        self.ros_node.get_logger().info(f'[LocalNavigation]: Neupan goal pose set successfully: {target_pose}')
        return True

    async def set_speed(self, speed: float, timeout_sec: float = 3.0) -> Result:
        """设置neupan参考速度"""
        if await self._set_reference_speed(speed, timeout_sec):
            return Result(success=True, message='Neupan reference speed set successfully')
        else:
            return Result(success=False, message='Failed to set Neupan reference speed')
    
    async def nav_to_pose(self, target_pose: NavPose, timeout_sec: float = 3.0) -> Result:
        """导航到达neupan目标位姿"""
        if not (await self.reset(timeout_sec=timeout_sec)).success:
            return Result(success=False, message='Failed to reset')
        if await self._nav_to_pose(target_pose, timeout_sec):
            await self._state.set(LocalNavStatus.NAVIGATING)
            return Result(success=True, message='Neupan goal pose set successfully')
        return Result(success=False, message='Failed to set Neupan goal pose')
        
    async def get_state(self, timeout_sec: float = 3.0) -> Result:
        """获取neupan当前状态"""
        cur_state = await self._state.get()
        if cur_state is None: return Result(success=False, message='Neupan state is not initialized')
        return Result(success=True, message='Neupan state retrieved successfully', data={"state": cur_state})
    
    async def reset(self, timeout_sec: float = 3.0) -> Result:
        """重置neupan控制器"""
        if await call_trigger(self.neupan_reset_client, timeout_sec):
            await self._state.set(LocalNavStatus.READY)
            return Result(success=True, message='Neupan reset successfully')
        return Result(success=False, message='Failed to reset Neupan')
    
    async def pause(self, timeout_sec: float = 3.0) -> Result:
        """暂停neupan控制器"""
        if (await self._state.get()) != LocalNavStatus.NAVIGATING:
            return Result(success=False, message='Neupan is not navigating, cannot pause')
        if await call_trigger(self.neupan_pause_client, timeout_sec):
            await self._state.set(LocalNavStatus.PAUSED)
            return Result(success=True, message='Neupan paused successfully')
        return Result(success=False, message='Failed to pause Neupan')
    
    async def resume(self, timeout_sec: float = 3.0) -> Result:
        """继续neupan控制器"""
        if (await self._state.get()) != LocalNavStatus.PAUSED:
            return Result(success=False, message='Neupan is not paused, cannot resume')
        if await call_trigger(self.neupan_resume_client, timeout_sec):
            await self._state.set(LocalNavStatus.NAVIGATING)
            return Result(success=True, message='Neupan resumed successfully')
        return Result(success=False, message='Failed to resume Neupan')


class GlobalNavigation(ROSClient):

    def __init__(self, ros_node_obj: Node):
        super().__init__(ros_node_obj, 'GlobalNavigation')
        self.goal_handle = None

        self._init_action_clients()
        self._init_service_clients()
        self._wait_for_services()

        self._state = GlobalNavStatus.READY
    
    async def start(self): await super().start()

    def _init_action_clients(self): 
        """ Initialize action client """
        self.nav2_to_pose_client = ActionClient(self.ros_node, NavigateToPose, 'navigate_to_pose')
        
        self._register_actions([self.nav2_to_pose_client])
    
    def _init_service_clients(self):
        """ initialize service clients """
        self.neupan_set_param_client = self.ros_node.create_client(SetParameters, 'neupan_node/set_parameters')
        self.neupan_pause_client = self.ros_node.create_client(Trigger, 'neupan_pause')
        self.neupan_resume_client = self.ros_node.create_client(Trigger, 'neupan_resume')
        self.neupan_reset_client = self.ros_node.create_client(Trigger, 'neupan_reset')
        
        self._register_services([
            self.neupan_set_param_client,
            self.neupan_pause_client,
            self.neupan_resume_client,
            self.neupan_reset_client
        ])
    
    async def _set_reference_speed(self, speed: float, timeout_sec: float = 3.0) -> bool:
        """ Set reference speed for controller (neupan) """
        self.ros_node.get_logger().info(f'[GlobalNavigation]: Setting Neupan reference speed to: {speed} m/s')

        if not await ParamUtils.set_params(
            self.neupan_set_param_client, ['reference_speed'],
            [ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=speed)],
            timeout_sec
        ):
            self.ros_node.get_logger().error('[GlobalNavigation]: Failed to set Neupan reference speed')
            return False
        
        self.ros_node.get_logger().info(f'[GlobalNavigation]: Neupan reference speed set to: {speed} m/s')
        return True

    async def _nav_to_pose(self, target_pose: NavPose, timeout_sec: float = 3.0) -> bool:
        """ calls ros2 nav nav_to_pose action """
        self.ros_node.get_logger().info(f'[GlobalNavigation]: Navigating to pose: {target_pose}')
        
        goal = NavigateToPose.Goal()
        goal.pose = target_pose.toPoseStamped(self.ros_node.get_clock().now().to_msg())
        
        future = self.nav2_to_pose_client.send_goal_async(goal)
        
        try:
            await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.ros_node.get_logger().error(f'[GlobalNavigation]: Navigate to pose timeout after {timeout_sec}s')
            return False
        
        if future.result() is None:
            self.ros_node.get_logger().error('[GlobalNavigation]: Failed to send navigate to pose goal')
            return False
        
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.ros_node.get_logger().error('[GlobalNavigation]: Navigate to pose goal was rejected')
            return False
        
        self.ros_node.get_logger().info('[GlobalNavigation]: Navigate to pose goal accepted, waiting for result...')
        
        result_handle = self.goal_handle.get_result_async()
        result_handle.add_done_callback(self._nav2_result_callback)

        return True
    
    def _nav2_result_callback(self, future):
        """ alter the system state based on nav2 result """
        self.goal_handle = None
        
        if future.result() is None:
            self.ros_node.get_logger().error('[GlobalNavigation]: Navigate to pose result is None')
            self._state = GlobalNavStatus.FAILED
            return
        
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.ros_node.get_logger().info('[GlobalNavigation]: Navigate to pose succeeded')
            self._state = GlobalNavStatus.ARRIVED
            return
        
        self.ros_node.get_logger().error(f'[GlobalNavigation]: Navigate to pose failed with status: {result.status}')
        self._state = GlobalNavStatus.FAILED
    
    async def _cancel(self, timeout_sec: float = 3.0) -> bool:
        """ cancel nav goal """
        if self.goal_handle is None:
            return True
        
        self.ros_node.get_logger().info('[GlobalNavigation]: Cancelling current navigation goal...')
        
        future = self.goal_handle.cancel_goal_async()
        
        try:
            await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.ros_node.get_logger().error(f'[GlobalNavigation]: Cancel navigation goal timeout after {timeout_sec}s')
            return False
        
        if future.result() is None:
            self.ros_node.get_logger().error('[GlobalNavigation]: Failed to cancel navigation goal')
            return False
        
        self.ros_node.get_logger().info('[GlobalNavigation]: Navigation goal cancelled successfully')
        return True

    async def set_speed(self, speed: float, timeout_sec: float = 3.0) -> Result:
        """ set speed and return message """
        if await self._set_reference_speed(speed, timeout_sec):
            return Result(success=True, message='Neupan reference speed set successfully')
        else:
            return Result(success=False, message='Failed to set Neupan reference speed')

    async def nav_to_pose(self, target_pose: NavPose, timeout_sec: float = 3.0) -> Result:
        """  """
        reset_result = await self.reset()
        if not reset_result.success:
            return Result(False, message=f"Cannot reset: {reset_result.message}")
        if await self._nav_to_pose(target_pose, timeout_sec):
            self._state = GlobalNavStatus.NAVIGATING
            return Result(success=True, message='Navigate to pose goal sent successfully')
        
        return Result(success=False, message='Failed to send navigate to pose goal')

    async def get_state(self, timeout_sec: float = 3.0) -> Result:
        """获取当前导航状态"""
        return Result(success=True, message='Global navigation state retrieved successfully', data={"state": self._state})
    
    async def reset(self, timeout_sec: float = 3.0) -> Result:
        """重置导航状态"""
        if self._state == GlobalNavStatus.NAVIGATING or self._state == GlobalNavStatus.PAUSED:
            if not (await self._cancel(timeout_sec)):
                return Result(success=False, message='Failed to reset: cannot cancel current navigation goal')
            self.goal_handle = None
            self._state = GlobalNavStatus.READY
        
        if not await call_trigger(self.neupan_reset_client, timeout_sec):
            return Result(success=False, message='Failed to reset Neupan')
            
        self._state = GlobalNavStatus.READY
        return Result(success=True, message='Global navigation reset successfully')
    
    async def pause(self, timeout_sec: float = 3.0) -> Result:
        """暂停neupan控制器"""
        if self._state != GlobalNavStatus.NAVIGATING:
            return Result(success=False, message='Global is not navigating, cannot pause')
        if not await call_trigger(self.neupan_pause_client, timeout_sec):
            return Result(success=False, message='Failed to pause Neupan')
        
        self._state = GlobalNavStatus.PAUSED
        return Result(success=True, message='Global paused successfully')
        
    
    async def resume(self, timeout_sec: float = 3.0) -> Result:
        """继续neupan控制器"""
        if self._state != GlobalNavStatus.PAUSED:
            return Result(success=False, message='Global is not paused, cannot resume')
        if not await call_trigger(self.neupan_resume_client, timeout_sec):
            return Result(success=False, message='Failed to resume Neupan')
        
        self._state = GlobalNavStatus.NAVIGATING
        return Result(success=True, message='Global resumed successfully')

class NavModeSwitcher(ROSClient):
    """导航模式切换器"""
    
    def __init__(self, ros_node_obj: Node):
        super().__init__(ros_node_obj, 'NavModeSwitcher')

        self._init_service_clients()
        self._wait_for_services()

        # navigation_switch_node 默认状态为地图模式
        self.states = NavigationMode.MAP
    
    async def start(self): await super().start()

    def _init_service_clients(self):
        """初始化服务客户端"""
        self.set_mode_client = self.ros_node.create_client(NavModeSwitch, 'nav_mode_switch')
        
        self._register_services([self.set_mode_client])
    
    async def _switch_mode(self, mode: NavigationMode, timeout_sec: float = 3.0) -> bool:
        """切换导航模式"""
        mode_dict = {
            NavigationMode.MAP: 'map',
            NavigationMode.MAP_FREE: 'map-free',
        }
        self.ros_node.get_logger().info(f'[NavModeSwitcher]: Switching navigation mode to: {mode}')
        
        request = NavModeSwitch.Request()
        request.mode = mode_dict[mode]
        
        future = self.set_mode_client.call_async(request)
        
        try:
            await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.ros_node.get_logger().error(f'[NavModeSwitcher]: Switch navigation mode timeout after {timeout_sec}s')
            return False
        
        if future.result() is None:
            self.ros_node.get_logger().error('[NavModeSwitcher]: Failed to switch navigation mode')
            return False
        
        response = future.result()
        if not response.success:
            self.ros_node.get_logger().error(f'[NavModeSwitcher]: Switch navigation mode failed: {response.message}')
            return False
        
        self.ros_node.get_logger().info(f'[NavModeSwitcher]: Navigation mode switched to: {mode}')
        return True

    async def map_mode(self, timeout_sec: float = 3.0) -> Result:
        """切换到地图模式"""
        if self.states == NavigationMode.MAP:
            return Result(success=True, message='Already in map mode')
        if not await self._switch_mode(NavigationMode.MAP, timeout_sec=timeout_sec):
            return Result(success=False, message='Failed to switch to map mode')
        self.states = NavigationMode.MAP
        return Result(success=True, message='Switched to map mode successfully')

    async def mapfree_mode(self, timeout_sec: float = 3.0) -> Result:
        """切换到地图自由模式"""
        if self.states == NavigationMode.MAP_FREE:
            return Result(success=True, message='Already in map-free mode')
        if not await self._switch_mode(NavigationMode.MAP_FREE, timeout_sec=timeout_sec):
            return Result(success=False, message='Failed to switch to mapfree mode')
        self.states = NavigationMode.MAP_FREE
        return Result(success=True, message='Switched to mapfree mode successfully')

    async def get_mode(self, timeout_sec: float = 3.0) -> Result:
        """获取当前导航模式"""
        return Result(success=True, message='Current navigation mode retrieved successfully', data={"mode": self.states})

class RelocalizationManager(ROSClient):
    """导航模式切换器"""
    
    def __init__(self, ros_node_obj: Node):
        super().__init__(ros_node_obj, 'RelocalizationManager')

        self._init_service_clients()
        self._wait_for_services()

        self._relocal_status = AsynVar(None)
        self._map_status = AsynVar(None)
    
    async def start(self):
        await super().start()
        self._relocal_update_task = asyncio.create_task(self._status_update_loop())

    def _init_service_clients(self):
        """初始化服务客户端"""
        self.set_initial_pose_client = self.ros_node.create_client(SetInitialPose, 'relocalization/set_initial_pose')
        self.toggle_client = self.ros_node.create_client(SetBool, 'relocalization/toggle_relocalization')
        self.query_status_client = self.ros_node.create_client(GetString, 'relocalization/query_status')
        self.query_map_loaded_client = self.ros_node.create_client(GetString, 'relocalization/query_map_loaded')
        self.set_pcd_client = self.ros_node.create_client(SetString, 'relocalization/set_pcd_file')
        self.get_pcd_client = self.ros_node.create_client(GetString, 'relocalization/get_pcd_file')
        
        self._register_services([
            self.set_initial_pose_client,
            self.toggle_client,
            self.query_status_client,
            self.query_map_loaded_client,
            self.set_pcd_client,
            self.get_pcd_client
        ])
    
    @log_exceptions
    async def _status_update_loop(self):
        """重定位状态更新循环"""
        relocal_status_conversion = {
            "successful": ReLocalStatus.SUCCESSFUL,
            "failed": ReLocalStatus.FAILED,
            "disabled": ReLocalStatus.DISABLED
        }
        
        map_status_conversion = {
            "empty": RelocalMapStatus.EMPTY,
            "loading": RelocalMapStatus.LOADING,
            "loaded": RelocalMapStatus.LOADED,
            "failed": RelocalMapStatus.FAILED
        }
        
        while rclpy.ok():
            # get relocalization status
            success, result = await StringSrvUtils.get_string(self.query_status_client)
            if not success:
                self.ros_node.get_logger().error(f'[RelocalizationManager]: {self.query_status_client.srv_name} service call failed')
                await asyncio.sleep(1.0)
                continue
            
            await self._relocal_status.set(relocal_status_conversion[result])
            
            # get map status
            success, result = await StringSrvUtils.get_string(self.query_map_loaded_client)
            if not success:
                self.ros_node.get_logger().error(f'[RelocalizationManager]: {self.query_status_client.srv_name} service call failed')
                await asyncio.sleep(1.0)
                continue
            
            await self._map_status.set(map_status_conversion[result])

            await asyncio.sleep(1.0)

    async def set_initial_pose(self, initial_pose: NavPose, timeout_sec: float = 3.0) -> Result:
        """设置重定位初始位姿"""
        self.ros_node.get_logger().info(f'[RelocalizationManager]: Setting initial pose: {initial_pose}')
        
        request = SetInitialPose.Request()
        request.pose = initial_pose.toPoseWithCovarianceStamped(self.ros_node.get_clock().now().to_msg())
        
        future = self.set_initial_pose_client.call_async(request)
        
        try:
            await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.ros_node.get_logger().error(f'[RelocalizationManager]: Set initial pose timeout after {timeout_sec}s')
            return Result(success=False, message='Set initial pose timeout')
        
        if future.result() is None:
            self.ros_node.get_logger().error('[RelocalizationManager]: Failed to set initial pose')
            return Result(success=False, message='Failed to set initial pose')
        
        self.ros_node.get_logger().info('[RelocalizationManager]: Initial pose set successfully')
        return Result(success=True, message='Initial pose set successfully')
    
    async def set_map(self, pcd_file_name: str, timeout_sec: float = 3.0) -> Result:
        """设置重定位地图"""
        self.ros_node.get_logger().info(f'[RelocalizationManager]: Setting relocalization map: {pcd_file_name}')
        
        success, message = await StringSrvUtils.set_string(self.set_pcd_client, pcd_file_name)
        
        if not success:
            self.ros_node.get_logger().error(f'[RelocalizationManager]: {self.set_pcd_client.srv_name} service call failed: {message}')
        
        self.ros_node.get_logger().info('[RelocalizationManager]: Relocalization map set successfully')

        return Result(success=True, message='Relocalization map set successfully')
    
    async def get_map(self, timeout_sec: float = 3.0) -> Result:
        """获取重定位地图"""
        self.ros_node.get_logger().info('[RelocalizationManager]: Getting relocalization map')
        
        success, result = await StringSrvUtils.get_string(self.get_pcd_client)
        
        if not success:
            self.ros_node.get_logger().error(f'[RelocalizationManager]: {self.get_pcd_client.srv_name} service call failed: {result}')
            return Result(success=False, message='Failed to get relocalization map')
        
        self.ros_node.get_logger().info(f'[RelocalizationManager]: Relocalization map retrieved: {result}')
        
        return Result(success=True, message='Relocalization map retrieved successfully', data={"map": result})
    
    async def toggle_relocalization(self, enable: bool, timeout_sec: float = 3.0) -> Result:
        """启用或禁用重定位"""
        self.ros_node.get_logger().info(f'[RelocalizationManager]: Toggling relocalization to: {"enabled" if enable else "disabled"}')
        
        request = SetBool.Request()
        request.data = enable
        
        future = self.toggle_client.call_async(request)
        try:
            await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.ros_node.get_logger().error(f'[RelocalizationManager]: Toggle relocalization timeout after {timeout_sec}s')
            return Result(success=False, message='Toggle relocalization timeout')
        
        if future.result() is None:
            self.ros_node.get_logger().error('[RelocalizationManager]: Failed to toggle relocalization')
            return Result(success=False, message='Failed to toggle relocalization')
        response = future.result()
        if not response.success:
            self.ros_node.get_logger().error(f'[RelocalizationManager]: Toggle relocalization failed: {response.message}')
            return Result(success=False, message=f'Toggle relocalization failed: {response.message}')
        self.ros_node.get_logger().info(f'[RelocalizationManager]: Relocalization toggled to: {"enabled" if enable else "disabled"}')
        return Result(success=True, message='Relocalization toggled successfully')
    
    async def get_relocalization_status(self, timeout_sec: float = 3.0) -> Result:
        """获取当前导航模式"""
        relocal_status = await self._relocal_status.get()
        map_status = await self._map_status.get()
        if relocal_status is None or map_status is None:
            return Result(success=False, message='Relocalization status is not initialized')
        return Result(
            success=True, 
            message='Relocalization status retrieved successfully', 
            data={
                "relocal_status": relocal_status,
                "map_status": map_status
            }
        )
