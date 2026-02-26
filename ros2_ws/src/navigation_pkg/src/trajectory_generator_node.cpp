#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator_node"), current_wp_index_(0), is_following_path_(false) {
        
        this->declare_parameter<double>("flight_speed", 1.5);
        this->declare_parameter<double>("acceptance_radius", 0.4);
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/command/trajectory", 10);
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&TrajectoryGeneratorNode::pathCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TrajectoryGeneratorNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Kinematic Waypoint Follower (Anti-Spin Edition) Ready.");
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<geometry_msgs::msg::Pose> pruned_path_;
    size_t current_wp_index_;
    bool is_following_path_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) return;

        pruned_path_.clear();
        for (size_t i = 0; i < msg->poses.size(); i += 3) {
            pruned_path_.push_back(msg->poses[i].pose);
        }
        if (pruned_path_.back() != msg->poses.back().pose) {
            pruned_path_.push_back(msg->poses.back().pose);
        }

        current_wp_index_ = 0;
        is_following_path_ = true;
        RCLCPP_INFO(this->get_logger(), "Commencing flight! Path downsampled to %zu waypoints.", pruned_path_.size());
    }

    void controlLoop() {
        if (!is_following_path_ || pruned_path_.empty()) return;

        double speed = this->get_parameter("flight_speed").as_double();
        double radius = this->get_parameter("acceptance_radius").as_double();

        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform("world", "true_body", tf2::TimePointZero);
        } catch (...) { return; }

        Eigen::Vector3d current_pos(
            tf_stamped.transform.translation.x,
            tf_stamped.transform.translation.y,
            tf_stamped.transform.translation.z);

        auto target_pose = pruned_path_[current_wp_index_];
        // Add 1.0m to Z so we don't scrape the floor
        Eigen::Vector3d target_pos(target_pose.position.x, target_pose.position.y, target_pose.position.z + 1.0); 

        double dist = (target_pos - current_pos).norm();
        if (dist < radius) {
            current_wp_index_++;
            if (current_wp_index_ >= pruned_path_.size()) {
                RCLCPP_INFO(this->get_logger(), "Destination Reached! Hovering.");
                is_following_path_ = false;
                
                // Extract current yaw to stop spinning when we hover
                double q_w = tf_stamped.transform.rotation.w;
                double q_x = tf_stamped.transform.rotation.x;
                double q_y = tf_stamped.transform.rotation.y;
                double q_z = tf_stamped.transform.rotation.z;
                double current_yaw = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));

                publishPoint(target_pos, Eigen::Vector3d(0,0,0), current_yaw);
                return;
            }
            target_pose = pruned_path_[current_wp_index_];
            target_pos = Eigen::Vector3d(target_pose.position.x, target_pose.position.y, target_pose.position.z + 1.0);
        }

        // --- THE YAW FIX ---
        Eigen::Vector3d dir = (target_pos - current_pos);
        double horizontal_dist = std::hypot(dir.x(), dir.y()); // Check horizontal travel distance
        
        dir.normalize(); 
        Eigen::Vector3d vel = dir * speed;

        // Extract current yaw from the TF tree
        double q_w = tf_stamped.transform.rotation.w;
        double q_x = tf_stamped.transform.rotation.x;
        double q_y = tf_stamped.transform.rotation.y;
        double q_z = tf_stamped.transform.rotation.z;
        double current_yaw = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));

        double target_yaw = current_yaw; // Default to keeping our current heading
        
        // Only point the nose if we are actually flying forward (not just up/down)
        if (horizontal_dist > 0.1) {
            target_yaw = std::atan2(dir.y(), dir.x());
        }

        publishPoint(target_pos, vel, target_yaw);
    }

    void publishPoint(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw) {
        auto traj_msg = trajectory_msgs::msg::MultiDOFJointTrajectory();
        traj_msg.header.stamp.sec = 0;
        traj_msg.header.stamp.nanosec = 0;
        traj_msg.header.frame_id = "world";
        traj_msg.joint_names.push_back("base_link");

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        geometry_msgs::msg::Transform transform;
        transform.translation.x = pos.x();
        transform.translation.y = pos.y();
        transform.translation.z = pos.z();
        
        transform.rotation.w = std::cos(yaw * 0.5);
        transform.rotation.x = 0.0;
        transform.rotation.y = 0.0;
        transform.rotation.z = std::sin(yaw * 0.5);
        point.transforms.push_back(transform);

        geometry_msgs::msg::Twist twist_vel;
        twist_vel.linear.x = vel.x();
        twist_vel.linear.y = vel.y();
        twist_vel.linear.z = vel.z();
        point.velocities.push_back(twist_vel);
        point.accelerations.push_back(geometry_msgs::msg::Twist());
        point.time_from_start.sec = 0;
        
        traj_msg.points.push_back(point);
        traj_pub_->publish(traj_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}