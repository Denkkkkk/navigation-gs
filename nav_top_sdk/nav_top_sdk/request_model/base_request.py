from pydantic import BaseModel
from typing import List, Optional

# FastAPI数据模型

class NavigationModeRequest(BaseModel):
    """导航模式切换请求模型"""
    mode: str  # "map", "map-free", "pause"

class NavigationGoal(BaseModel):
    """导航目标点模型"""
    x: float
    y: float
    theta: float
    speed: float
    frame_id: str = "map"


class Waypoint(BaseModel):
    """路径点模型"""
    x: float
    y: float
    theta: float
    
    
class CyclicPathRequest(BaseModel):
    """循环路径请求模型"""
    waypoints: List[Waypoint]
    frame_id: str = "map"
    speed: float
    cycles: Optional[int] = None  # None表示无限循环

class StatusResponse(BaseModel):
    """状态响应模型"""
    success: bool
    message: str
    data: Optional[dict] = None

class SetMapRequest(BaseModel):
    """设置地图请求模型"""
    scene_name: str

class SetRobotSizeRequest(BaseModel):
    """设置机器人尺寸请求模型"""
    width: float
    length: float

class ToggleRequest(BaseModel):
    """切换请求模型"""
    enable: bool