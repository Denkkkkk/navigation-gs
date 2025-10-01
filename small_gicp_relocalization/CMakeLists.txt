cmake_minimum_required(VERSION 3.10)
project(small_gicp_relocalization)

# Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenMP REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

########################################
## Generate service interface headers ##
########################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetString.srv"
  "srv/SetString.srv"
)

ament_export_dependencies(rosidl_default_runtime)

#################
## Subprojects ##
#################

add_subdirectory(third_parties/small_gicp)

##################
## Build target ##
##################

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_parties/small_gicp/include
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  DIRECTORY src
)

# Link with dependencies
target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  small_gicp
  Eigen3::Eigen
  small_gicp_relocalization__rosidl_typesupport_cpp
)

# Link with ROS dependencies and generated service interfaces
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  std_srvs
  geometry_msgs
  sensor_msgs
  nav2_msgs
  tf2_ros
  pcl_conversions
  tf2_eigen
)

# Register the component
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "small_gicp_relocalization::SmallGicpRelocalizationNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
