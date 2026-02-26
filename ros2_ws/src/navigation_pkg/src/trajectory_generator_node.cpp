#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator_node") {
        
        // Flight Speed Parameter (meters per second)
        this->declare_parameter<double>("flight_speed", 1.0);

        // Publisher to the drone's actual flight controller
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "/command/trajectory", 10);

        // Listen to the A* Planner for complex cave routes
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&TrajectoryGeneratorNode::pathCallback, this, std::placeholders::_1));

        // Listen to a simple waypoint topic (Perfect for your State Machine Takeoff!)
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/direct_waypoint", 10, std::bind(&TrajectoryGeneratorNode::waypointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ Trajectory Generator Node Started. Ready for takeoff!");
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

    // Helper function to create a fully formatted trajectory point
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint createTrajectoryPoint(
        const geometry_msgs::msg::Point& pos,
        const geometry_msgs::msg::Quaternion& rot,
        double time_sec)
    {
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        
        geometry_msgs::msg::Transform transform;
        transform.translation.x = pos.x;
        transform.translation.y = pos.y;
        transform.translation.z = pos.z;
        transform.rotation = rot;
        point.transforms.push_back(transform);

        // Leave velocities and accelerations at 0; the Unity PID handles the physics
        point.velocities.push_back(geometry_msgs::msg::Twist());
        point.accelerations.push_back(geometry_msgs::msg::Twist());

        // Convert float seconds to ROS 2 sec/nanosec format
        point.time_from_start.sec = static_cast<int32_t>(std::floor(time_sec));
        point.time_from_start.nanosec = static_cast<uint32_t>((time_sec - std::floor(time_sec)) * 1e9);

        return point;
    }

    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received direct waypoint to Z: %.2f. Generating straight trajectory...", msg->pose.position.z);
        
        auto traj_msg = trajectory_msgs::msg::MultiDOFJointTrajectory();
        traj_msg.header.stamp = this->now();
        traj_msg.header.frame_id = "world";
        traj_msg.joint_names.push_back("base_link");

        // Give it 2 seconds to reach the simple waypoint
        auto point = createTrajectoryPoint(msg->pose.position, msg->pose.orientation, 2.0);
        traj_msg.points.push_back(point);

        traj_pub_->publish(traj_msg);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) return;

        double speed = this->get_parameter("flight_speed").as_double();
        
        auto traj_msg = trajectory_msgs::msg::MultiDOFJointTrajectory();
        traj_msg.header.stamp = this->now();
        traj_msg.header.frame_id = "world";
        traj_msg.joint_names.push_back("base_link");

        double cumulative_time = 1.0; // Give the drone 1 second to spool up and start moving

        for (size_t i = 0; i < msg->poses.size(); ++i) {
            auto pose = msg->poses[i].pose;
            
            // Calculate time to reach this point based on distance from the previous point
            if (i > 0) {
                auto prev_pose = msg->poses[i-1].pose;
                double dist = std::sqrt(
                    std::pow(pose.position.x - prev_pose.position.x, 2) +
                    std::pow(pose.position.y - prev_pose.position.y, 2) +
                    std::pow(pose.position.z - prev_pose.position.z, 2)
                );
                cumulative_time += (dist / speed);
            }

            auto point = createTrajectoryPoint(pose.position, pose.orientation, cumulative_time);
            traj_msg.points.push_back(point);
        }

        traj_pub_->publish(traj_msg);
        RCLCPP_INFO(this->get_logger(), "Published A* trajectory taking %.2f seconds.", cumulative_time);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}