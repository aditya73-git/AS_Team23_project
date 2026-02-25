cmake_minimum_required(VERSION 3.8)
project(perception_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Find dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# ========================
# Include directories
# ========================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ========================
# Create executable
# ========================
add_executable(depth_to_pointcloud_node
  src/depth_to_pointcloud_node.cpp
)

# ========================
# Link dependencies
# ========================
ament_target_dependencies(depth_to_pointcloud_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_geometry
  pcl_conversions
)

target_link_libraries(depth_to_pointcloud_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# ========================
# Install executable
# ========================
install(TARGETS
  depth_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ========================
# Testing
# ========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ========================
# Finalize package
# ========================
ament_package()