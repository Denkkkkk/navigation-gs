import asyncio, traceback, functools
from typing import List, Tuple, Generic, TypeVar

from ruamel.yaml import YAML

from rclpy.client import Client
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterValue

from std_srvs.srv import Trigger

from small_gicp_relocalization.srv import GetString, SetString


T = TypeVar("T")
class AsynVar(Generic[T]):
    def __init__(self, initial_value: T):
        self._value: T = initial_value
        self._lock = asyncio.Lock()
    
    async def get(self) -> T:
        async with self._lock:
            return self._value
    
    async def set(self, new_value: T):
        async with self._lock:
            self._value = new_value

def log_exceptions(coro):
    @functools.wraps(coro)
    async def wrapper(*args, **kwargs):
        try: return await coro(*args, **kwargs)
        except Exception as e:
            print(f"Exception in {coro.__name__}: {e}")
            traceback.print_exc()
            raise  # Optional: re-raise so the Task is marked as failed
    return wrapper

class ParamUtils(object):

    @staticmethod
    async def set_params(client: Client, names: List[str], values: List[ParameterValue], timeout_sec: float = 3.0) -> bool:
        """通用参数设置函数"""
        request = SetParameters.Request()
        request.parameters = [RclParameter(name=name, value=value) for name, value in zip(names, values)]

        future = client.call_async(request)

        try: await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError: return False
        
        if future.result() is None or not future.result().results or not all(r.successful for r in future.result().results):
            return False
        
        return True
    
    @staticmethod
    async def get_params(client: Client, names: List[str], timeout_sec: float = 3.0) -> Tuple[bool, List[ParameterValue]]:
        """通用参数获取函数"""
        request = GetParameters.Request()
        request.names = names

        future = client.call_async(request)

        try: await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError: return False, []
        
        if future.result() is None or not future.result().values: return False, []
        
        return True, future.result().values
    
    @staticmethod
    def update_yaml_param(file_path, key_path, new_value):
        yaml = YAML()
        with open(file_path, 'r') as f: data = yaml.load(f)
        
        # Navigate the nested dictionary
        d = data
        for key in key_path[:-1]: d = d.setdefault(key, {})
        d[key_path[-1]] = new_value

        with open(file_path, 'w') as f: yaml.dump(data, f)

class StringSrvUtils(object):
    
    @staticmethod
    async def set_string(client: Client, data: str, timeout_sec: float = 1.0) -> Tuple[bool, str]:
        """通用字符串设置函数"""
        request = SetString.Request()
        request.data = data

        future = client.call_async(request)

        try: await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError: return False
        
        if future.result() is None or not future.result().success:
            return False, future.result().message
        
        return True, future.result().message
    
    @staticmethod
    async def get_string(client: Client, timeout_sec: float = 1.0) -> Tuple[bool, str]:
        """通用字符串获取函数"""
        request = GetString.Request()

        future = client.call_async(request)

        try: await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError: return False, ""
        
        if future.result() is None:
            return False, ""
        
        return True, future.result().data

async def call_trigger(client: Client, timeout_sec = 3.0) -> bool:
    """ universal trigger call """
    
    future = client.call_async(Trigger.Request())
    
    try: await asyncio.wait_for(future, timeout=timeout_sec)
    except asyncio.TimeoutError: return False
    
    if future.result() is None: return False
    
    response = future.result()
    if not response.success: return False
    
    return True