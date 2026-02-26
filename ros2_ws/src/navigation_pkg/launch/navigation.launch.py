from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. Frontier Exploration Node (The Brain)
        # Finds the edges of the unknown map and publishes a /goal_waypoint
        Node(
            package='navigation_pkg',
            executable='frontier_exploration_node',
            name='frontier_exploration_node',
            output='screen',
            parameters=[{
                'robot_base_frame': 'true_body',  # Overriding with the correct TF frame
                'global_frame': 'world'
            }]
        ),

        # 2. 3D A* Path Planner Node (The GPS)
        # Listens to /goal_waypoint and publishes a collision-free /planned_path
        Node(
            package='navigation_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[{
                'robot_base_frame': 'true_body',
                'global_frame': 'world'
            }]
        )
    ])