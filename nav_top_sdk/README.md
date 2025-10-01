# Nav Top SDK - 重构后的架构

## 概述

这个项目已经从单一的大文件重构为模块化的架构，提供更好的可维护性和可扩展性。

## 新的文件结构

```
nav_top_sdk/
├── __init__.py              # 包导入文件
├── main_node.py             # 主节点（整合所有功能）
├── ros2_navigation_service.py  # ROS2导航服务
├── navigation_api.py        # FastAPI Web接口
├── config.py               # 配置管理
├── utils.py                # 工具函数
├── sdk_node.py             # 向后兼容文件（已弃用）
└── README.md               # 本文件
```

## 主要组件

### 1. NavigationService (ros2_navigation_service.py)
- 处理所有ROS2相关的导航功能
- 包含Action客户端、服务客户端和发布者
- 提供同步和异步导航接口
- 支持路径规划、循环任务和参数设置

### 2. NavigationAPI (navigation_api.py)
- 提供完整的FastAPI Web接口
- RESTful API端点，包含数据验证
- 异步操作支持
- 标准化的响应格式

### 3. TopSdkNode (main_node.py)
- 主节点类，继承NavigationService
- 整合ROS2服务和Web API
- 提供示例和测试方法
- 简化的初始化流程

### 4. 配置管理 (config.py)
- 集中的配置常量
- API、导航、ROS2和日志配置
- 易于维护和修改

### 5. 工具函数 (utils.py)
- 几何计算工具
- 数据验证工具
- 路径处理工具
- 日志格式化工具

## 使用方法

### 基本用法

```python
import rclpy
from nav_top_sdk.main_node import TopSdkNode

def main():
    rclpy.init()
    
    # 创建节点
    node = TopSdkNode()
    
    # 启动Web服务器
    node.start_web_server(host="0.0.0.0", port=8000)
    
    # 运行示例
    node.run_navigation_examples()
    
    # 保持节点运行
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 仅使用ROS2服务

```python
import rclpy
from nav_top_sdk.ros2_navigation_service import NavigationService, NavigationMode

def main():
    rclpy.init()
    
    # 创建导航服务
    nav_service = NavigationService('my_nav_node')
    
    # 切换到无图导航模式
    nav_service.switch_navigation_mode(NavigationMode.MAP_FREE)
    
    # 导航到指定位置
    nav_service.navigate_to_pose(1.0, 2.0, 0.0)
    
    rclpy.spin(nav_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 仅使用Web API

```python
from nav_top_sdk.ros2_navigation_service import NavigationService
from nav_top_sdk.navigation_api import NavigationAPI
import uvicorn

# 创建ROS2服务（在实际应用中需要在ROS2环境中运行）
nav_service = NavigationService()

# 创建API
api = NavigationAPI(nav_service)

# 直接运行服务器
if __name__ == "__main__":
    uvicorn.run(api.get_app(), host="0.0.0.0", port=8000)
```

## API端点

### 基本端点
- `GET /` - API状态检查
- `GET /status` - 获取系统状态
- `GET /get_current_mode` - 获取当前导航模式

### 导航控制
- `POST /navigate` - 导航到指定位姿
- `POST /navigate_async` - 异步导航（Nav2）
- `POST /switch_mode` - 切换导航模式

### 路径控制
- `POST /publish_path` - 发布路径
- `POST /navigate_through_path` - 通过路径导航（Nav2）
- `POST /start_cyclic_path` - 启动循环路径任务
- `POST /stop_cyclic_path` - 停止循环路径任务

### 参数设置
- `POST /set_speed` - 设置参考速度
- `GET /query_controller` - 查询控制器状态

## 向后兼容性

原始的`sdk_node.py`仍然可以使用，但会显示弃用警告。建议迁移到新的模块化架构：

```python
# 旧的用法（仍然可用，但已弃用）
from nav_top_sdk.sdk_node import TopSdkNode

# 新的推荐用法
from nav_top_sdk.main_node import TopSdkNode
```

## 优势

1. **模块化设计**: 每个组件职责明确，易于测试和维护
2. **更好的可扩展性**: 可以独立扩展ROS2功能或Web API
3. **配置集中管理**: 所有配置项集中在config.py中
4. **丰富的工具函数**: 提供常用的几何计算和验证工具
5. **标准化API**: 统一的响应格式和错误处理
6. **向后兼容**: 保持与原有代码的兼容性

## 测试

启动节点后，可以通过以下方式测试API：

```bash
# 获取状态
curl -X GET http://localhost:8000/status

# 切换导航模式
curl -X POST http://localhost:8000/switch_mode \
     -H "Content-Type: application/json" \
     -d '{"mode": "map-free"}'

# 导航到指定位置
curl -X POST http://localhost:8000/navigate \
     -H "Content-Type: application/json" \
     -d '{"x": 2.0, "y": 1.0, "theta": 1.57}'
```

或访问自动生成的API文档：
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc
