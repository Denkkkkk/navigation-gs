cmake_minimum_required(VERSION 3.8)
project(gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(gazebo_sim src/gazebo_sim.cpp)
ament_target_dependencies(gazebo_sim
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_msgs
  gazebo_ros
  pcl_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(gazebo_sim
  ${PCL_LIBRARIES}
)

add_executable(map_to_odom src/map_to_odom.cpp)
ament_target_dependencies(map_to_odom
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  pcl_msgs
  gazebo_msgs
  pcl_conversions
)
target_include_directories(map_to_odom PUBLIC
  include
)
target_link_libraries(map_to_odom
  ${PCL_LIBRARIES}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  gazebo_sim
  map_to_odom
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config world meshes map 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
