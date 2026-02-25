from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. YOUR CUSTOM C++ NODE: Depth to PointCloud
        Node(
            package='perception_cpp_pkg',
            executable='depth_to_pointcloud_node',
            name='depth_to_pointcloud_node',
            output='screen',
            # You can override your default params here if needed
            parameters=[{
                'depth_topic': '/realsense/depth/image',
                'pointcloud_topic': '/points'
            }]
        ),

        # 2. THE PRE-BUILT C++ NODE: Octomap Server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                {'frame_id': 'world'},                          # The stationary global map frame
                {'base_frame_id': 'Quadrotor/DepthCamera'},     # The moving sensor frame (for raycasting/clearing empty space)
                {'resolution': 0.1},                            # Map resolution (10cm voxels)
                {'sensor_model.max_range': 49.0},               # Match the filter limit from your custom node
            ],
            remappings=[
                ('cloud_in', '/points'),                        # Tell Octomap to listen to your custom node's output!
            ]
        )
    ])