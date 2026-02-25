#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from image_geometry import PinholeCameraModel
import cv2
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_node')

        # Parameters matching your simulation's topic list
        self.declare_parameter('depth_topic', '/realsense/depth/image')
        self.declare_parameter('camera_info_topic', '/realsense/depth/camera_info')
        self.declare_parameter('pointcloud_topic', '/points')

        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value

        # Tools
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        # Subscribers
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)

        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, pointcloud_topic, 10)

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info('Camera info received and camera model set.')

    def depth_callback(self, msg: Image):
        if not self.camera_info_received:
            return

        # 1. Convert ROS Image to OpenCV/Numpy array
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Ensure we are doing math on floats
        depth_image = depth_image.astype(np.float32)

        # ==========================================
        # SCALING FIX:
        # If your debug prints show max values > 100, your simulation is outputting millimeters.
        # UNCOMMENT THE LINE BELOW TO CONVERT TO METERS:
        depth_image = depth_image / 1000.0
        # ==========================================

        # 2. Vectorized Math Setup
        height, width = depth_image.shape
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        z = depth_image.flatten()
        u = u.flatten()
        v = v.flatten()

        # 3. Debug Print (Prints roughly once every 30 frames)
        if np.random.rand() < 0.03:
            max_z = np.nanmax(z) if len(z) > 0 else 0.0
            min_z = np.nanmin(z) if len(z) > 0 else 0.0
            self.get_logger().info(f"RAW DEPTH -> Min: {min_z:.2f}, Max: {max_z:.2f}")

        # 4. Strict Filtering (Remove NaNs, Inf, and Out-of-Bounds values)
        # Assuming values are in meters: keep points between 0.1m and 20.0m
        valid_mask = (z > 0.1) & (z < 20.0) & np.isfinite(z)
        
        z_valid = z[valid_mask]
        u_valid = u[valid_mask]
        v_valid = v[valid_mask]

        # Early exit if all points were destroyed by the filter
        if len(z_valid) == 0:
            return

        # 5. Apply Pinhole Camera Model Math (Optical Frame)
        x_opt = (u_valid - self.camera_model.cx()) * z_valid / self.camera_model.fx()
        y_opt = (v_valid - self.camera_model.cy()) * z_valid / self.camera_model.fy()
        z_opt = z_valid
        
        # 6. Convert to ROS Standard Frame (X-forward, Y-left, Z-up)
        # We rotate the optical coordinates to match the drone's body frame
        x_body = z_opt
        y_body = -x_opt
        z_body = -y_opt
        
        # Stack into list of [x, y, z] arrays
        points = np.vstack((x_body, y_body, z_body)).T.tolist()

        # 6. Override the frame_id to match the simulation's actual TF Tree
        msg.header.frame_id = 'Quadrotor/DepthCamera'
        
        # 7. Create and Publish the PointCloud2 message
        pc2_msg = pc2.create_cloud_xyz32(msg.header, points)
        self.pc_pub.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()