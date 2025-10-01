from pydantic import BaseModel
from typing import Dict, Any, Optional
from enum import Enum
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
import math
import os


class Result:
    def __init__(self, success: bool, message: str = "", data: Dict[str, Any] = None):
        self.success = success
        self.message = message
        self.data = data

class LocalNavStatus(Enum):
    READY = 1
    NAVIGATING = 2
    FAILED = 3
    ARRIVED = 4
    PAUSED = 5

class GlobalNavStatus(Enum):
    READY = 1
    NAVIGATING = 2
    FAILED = 3
    ARRIVED = 4
    PAUSED = 5

class ReLocalStatus(Enum):
    DISABLED = "disabled"
    SUCCESSFUL = "successful"
    FAILED = "failed"

class RelocalMapStatus(Enum):
    EMPTY = "empty"
    LOADING = "loading"
    LOADED = "loaded"
    FAILED = "failed"

class NavigationMode(Enum):
    """导航模式测试枚举"""
    MAP = "map"
    MAP_FREE = "map-free"

class NavServiceStatus(Enum):
    """导航服务状态枚举"""
    MAP_VACANT = 1

    MAP_SINGLE_NAVIGATING = 2
    MAP_SINGLE_PAUSED = 3
    
    MAP_MULTIPLE_NAVIGATING = 4
    MAP_MULTIPLE_PAUSED = 5
    
    MAP_ARRIVED = 6
    MAP_FAILED = 7

    MAPFREE_VACANT = 8

    MAPFREE_SINGLE_NAVIGATING = 9
    MAPFREE_SINGLE_PAUSED = 10

    MAPFREE_MULTIPLE_NAVIGATING = 11
    MAPFREE_MULTIPLE_PAUSED = 12
    
    MAPFREE_ARRIVED = 13
    MAPFREE_FAILED = 14

MAP_MODES = [
    NavServiceStatus.MAP_SINGLE_NAVIGATING, 
    NavServiceStatus.MAP_MULTIPLE_NAVIGATING,
    NavServiceStatus.MAP_SINGLE_PAUSED,
    NavServiceStatus.MAP_MULTIPLE_PAUSED,
    NavServiceStatus.MAP_ARRIVED,
    NavServiceStatus.MAP_FAILED,
    NavServiceStatus.MAP_VACANT
]

NAVIGATING_MODES = [
    NavServiceStatus.MAP_SINGLE_NAVIGATING,
    NavServiceStatus.MAP_MULTIPLE_NAVIGATING,
    NavServiceStatus.MAPFREE_SINGLE_NAVIGATING,
    NavServiceStatus.MAPFREE_MULTIPLE_NAVIGATING
]

PAUSED_MODES = [
    NavServiceStatus.MAP_SINGLE_PAUSED,
    NavServiceStatus.MAP_MULTIPLE_PAUSED,
    NavServiceStatus.MAPFREE_SINGLE_PAUSED,
    NavServiceStatus.MAPFREE_MULTIPLE_PAUSED
]

ARRIVED_MODES = [
    NavServiceStatus.MAP_ARRIVED,
    NavServiceStatus.MAPFREE_ARRIVED
]

FAILED_MODES = [
    NavServiceStatus.MAP_FAILED,
    NavServiceStatus.MAPFREE_FAILED
]

VACANT_MODES = [
    NavServiceStatus.MAP_VACANT,
    NavServiceStatus.MAPFREE_VACANT
]


class NavPose:
    """导航目标点模型"""
    def __init__(self, x: float, y: float, theta: float, frame_id: str = "map"):
        self.x = x
        self.y = y
        self.theta = theta
        self.frame_id = os.environ.get("HUB_ID", "") + '/map'

    def toPoseStamped(self, cur_time: Time) -> PoseStamped:
        """将NavPose转换为PoseStamped"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.header.stamp = cur_time
        pose_stamped.pose.position.x = self.x
        pose_stamped.pose.position.y = self.y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.w = math.cos(self.theta / 2.0)
        pose_stamped.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        return pose_stamped
    
    def toPoseWithCovarianceStamped(self, cur_time: Time) -> PoseWithCovarianceStamped:
        """将NavPose转换为PoseWithCovarianceStamped"""
        pose_with_covariance_stamped = PoseWithCovarianceStamped()
        pose_with_covariance_stamped.header.frame_id = self.frame_id
        pose_with_covariance_stamped.header.stamp = cur_time
        pose_with_covariance_stamped.pose.pose.position.x = self.x
        pose_with_covariance_stamped.pose.pose.position.y = self.y
        pose_with_covariance_stamped.pose.pose.position.z = 0.0
        pose_with_covariance_stamped.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        pose_with_covariance_stamped.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_with_covariance_stamped.pose.pose.orientation.x = 0.0
        pose_with_covariance_stamped.pose.pose.orientation.y = 0.0
        # 设置协方差矩阵为单位矩阵
        pose_with_covariance_stamped.pose.covariance = [0.0] * 36
        pose_with_covariance_stamped.pose.covariance[0] = 0.1  # x position variance
        pose_with_covariance_stamped.pose.covariance[7] = 0.1  # y position variance
        pose_with_covariance_stamped.pose.covariance[14] = 0.1  # z position variance
        pose_with_covariance_stamped.pose.covariance[21] = 0.1  # orientation x variance
        pose_with_covariance_stamped.pose.covariance[28] = 0.1  # orientation y variance
        pose_with_covariance_stamped.pose.covariance[35] = 0.1  # orientation z variance
        return pose_with_covariance_stamped
    
    def __str__(self):
        return f"x: {self.x}, y: {self.y}, theta: {self.theta}, frame_id: {self.frame_id}"

class WebResponse(BaseModel):
    """Web响应模型"""
    code: int
    message: str = ""
    data: Optional[Dict[str, Any]] = None