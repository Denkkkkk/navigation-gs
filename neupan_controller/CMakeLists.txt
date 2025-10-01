cmake_minimum_required(VERSION 3.5)
project(neupan_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

nav2_package()

set(library_name neupan_controller)

add_library(${library_name} SHARED
  src/neupan_controller.cpp
  src/parameter_handler.cpp
  src/path_handler.cpp)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${library_name}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav2_core_TARGETS}
  ${nav2_costmap_2d_TARGETS}
  ${nav2_util_TARGETS}
  ${nav_msgs_TARGETS}
  ${pluginlib_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  angles::angles
)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY third_parties/neupan/neupan
  DESTINATION local/lib/python3.10/dist-packages
)

install(DIRECTORY include
  DESTINATION include/${library_name}
)

install(DIRECTORY 
  launch
  checkpoints
  config
  third_parties
  DESTINATION share/${library_name}
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/neupan_controller/neupan_node
     DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
     FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                      GROUP_READ GROUP_EXECUTE
                      WORLD_READ WORLD_EXECUTE)

ament_export_include_directories(include/${library_name})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  std_msgs
  tf2
  tf2_ros
)
ament_export_targets(${library_name})

ament_python_install_package(${library_name})

pluginlib_export_plugin_description_file(nav2_core neupan_controller.xml)

ament_package()
