cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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

# frontier_exploration node
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
#path planner node
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
#traectory planner node
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
ament_target_dependencies(trajectory_generator_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  trajectory_msgs
)

install(TARGETS 
  frontier_exploration_node
  path_planner_node
  trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
