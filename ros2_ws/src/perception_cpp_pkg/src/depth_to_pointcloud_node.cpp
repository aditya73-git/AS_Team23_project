#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>

class DepthToPointCloudNode : public rclcpp::Node {
public:
    DepthToPointCloudNode() : Node("depth_to_pointcloud_node"), camera_info_received_(false) {
        // Declare and get parameters
        this->declare_parameter<std::string>("depth_topic", "/realsense/depth/image");
        this->declare_parameter<std::string>("camera_info_topic", "/realsense/depth/camera_info");
        this->declare_parameter<std::string>("pointcloud_topic", "/points");

        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
        std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();

        rclcpp::QoS qos(10);

        // Subscribers & Publishers
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, qos, std::bind(&DepthToPointCloudNode::cameraInfoCallback, this, std::placeholders::_1));
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, qos, std::bind(&DepthToPointCloudNode::depthCallback, this, std::placeholders::_1));

        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, qos);
        
        RCLCPP_INFO(this->get_logger(), "C++ Depth to PointCloud Node Started.");
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!camera_info_received_) {
            camera_model_.fromCameraInfo(*msg);
            camera_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera info received and model set.");
        }
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!camera_info_received_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            // Safely convert any depth format into a float matrix
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_image = cv_ptr->image;

        // Initialize PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header = msg->header;
        cloud_msg.header.frame_id = "Quadrotor/DepthCamera"; // Override TF frame
        cloud_msg.height = 1;
        cloud_msg.width = 0; // Will be updated
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        // Efficiently allocate memory for the points
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(depth_image.rows * depth_image.cols);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        size_t valid_points = 0;

        // Iterate through all pixels
        for (int v = 0; v < depth_image.rows; ++v) {
            for (int u = 0; u < depth_image.cols; ++u) {
                float z = depth_image.at<float>(v, u);
                
                // Millimeter to Meter conversion (if values are huge like we saw in Python)
                if (z > 100.0) { 
                    z = z / 1000.0;
                }

                // Strict Filter: Remove NaN, Inf, and limits
                if (std::isfinite(z) && z > 0.1 && z < 50.0) {
                    
                    // Pinhole math (Optical Frame)
                    float x_opt = (u - camera_model_.cx()) * z / camera_model_.fx();
                    float y_opt = (v - camera_model_.cy()) * z / camera_model_.fy();
                    
                    // Rotate to ROS Body Frame (X-forward, Y-left, Z-up)
                    *iter_x = z;
                    *iter_y = -x_opt;
                    *iter_z = -y_opt;

                    ++iter_x; ++iter_y; ++iter_z;
                    ++valid_points;
                }
            }
        }

        if (valid_points == 0) return;

        // Shrink cloud to actual number of valid points to save bandwidth
        modifier.resize(valid_points); 
        pc_pub_->publish(cloud_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    
    image_geometry::PinholeCameraModel camera_model_;
    bool camera_info_received_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}