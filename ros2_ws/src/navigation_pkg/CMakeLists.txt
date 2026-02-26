cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

# ========================
# Compiler options
# ========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Find dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ========================
# Include directories
# ========================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}  # Eigen headers
)

# ========================
# Frontier exploration node
# ========================
add_executable(frontier_exploration_node src/frontier_exploration_node.cpp)
ament_target_dependencies(frontier_exploration_node
  rclcpp
  tf2
  tf2_ros
  octomap_msgs
  geometry_msgs
)
target_link_libraries(frontier_exploration_node
  octomap
)

# ========================
# Path planner node
# ========================
add_executable(path_planner_node src/path_planner_node.cpp)
ament_target_dependencies(path_planner_node
  rclcpp
  octomap
  octomap_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ========================
# Trajectory generator node
# ========================
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
ament_target_dependencies(trajectory_generator_node
  rclcpp
  nav_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_ros
)
target_link_libraries(trajectory_generator_node
  Eigen3::Eigen
)

# ========================
# Install executables
# ========================
install(TARGETS
  frontier_exploration_node
  path_planner_node
  trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

# ========================
# Install launch directory
# ========================
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