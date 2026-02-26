#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// --- NEW TF2 HEADERS ---
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class FrontierExplorationNode : public rclcpp::Node
{
public:
    FrontierExplorationNode()
    : Node("frontier_exploration_node")
    {
        // Parameter for the drone's body frame (adjust if your simulation uses something else)
        this->declare_parameter<std::string>("robot_base_frame", "true_body");
        this->declare_parameter<std::string>("global_frame", "world");
        
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        global_frame_ = this->get_parameter("global_frame").as_string();

        // Initialize TF2 Buffer and Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_full", 10,
            std::bind(&FrontierExplorationNode::octomapCallback, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_waypoint", 10);

        RCLCPP_INFO(this->get_logger(), "Autonomous Frontier Exploration Node Started");
    }

private:
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string robot_base_frame_;
    std::string global_frame_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_z_ = 0.0;

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        // 1. UPDATE DRONE POSITION VIA TF
        try {
            // Ask TF: "Where is the robot in the world RIGHT NOW?"
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
                global_frame_, robot_base_frame_, tf2::TimePointZero);
            
            robot_x_ = transformStamped.transform.translation.x;
            robot_y_ = transformStamped.transform.translation.y;
            robot_z_ = transformStamped.transform.translation.z;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Waiting for TF data: %s", ex.what());
            return; // Don't calculate frontiers if we don't know where we are!
        }

        // 2. CONVERT OCTOMAP
        std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::fullMsgToMap(*msg));
        if (!tree) return;

        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree.get());
        if (!octree) return;

        octomap::point3d best_frontier;
        double best_distance = std::numeric_limits<double>::max();
        bool frontier_found = false;

        // 3. SEARCH FOR FRONTIERS
        for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
             end = octree->end_leafs(); it != end; ++it)
        {
            if (octree->isNodeOccupied(&*it)) continue; // Only check free space

            octomap::point3d coord = it.getCoordinate();

            if (isFrontier(octree, coord))
            {
                double dist = distanceToRobot(coord);

                if (dist < best_distance)
                {
                    best_distance = dist;
                    best_frontier = coord;
                    frontier_found = true;
                }
            }
        }

        // 4. PUBLISH GOAL
        if (frontier_found)
        {
            publishGoal(best_frontier, msg->header.frame_id);
            RCLCPP_INFO(this->get_logger(),
                "Targeting nearest frontier at (%.2f, %.2f, %.2f) | Distance: %.2fm",
                best_frontier.x(), best_frontier.y(), best_frontier.z(), best_distance);
        }
    }

    bool isFrontier(octomap::OcTree* octree, const octomap::point3d& point)
    {
        // WARNING FIX: Explicitly cast resolution to float
        float res = static_cast<float>(octree->getResolution());

        std::vector<octomap::point3d> neighbors =
        {
            {point.x() + res, point.y(), point.z()},
            {point.x() - res, point.y(), point.z()},
            {point.x(), point.y() + res, point.z()},
            {point.x(), point.y() - res, point.z()},
            {point.x(), point.y(), point.z() + res},
            {point.x(), point.y(), point.z() - res}
        };

        for (const auto& n : neighbors)
        {
            octomap::OcTreeNode* node = octree->search(n);
            if (!node) return true; // If nullptr, it's an unknown boundary
        }
        return false;
    }

    double distanceToRobot(const octomap::point3d& point)
    {
        return std::sqrt(
            std::pow(point.x() - robot_x_, 2) +
            std::pow(point.y() - robot_y_, 2) +
            std::pow(point.z() - robot_z_, 2));
    }

    void publishGoal(const octomap::point3d& frontier, const std::string& frame_id)
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = frame_id;

        goal.pose.position.x = frontier.x();
        goal.pose.position.y = frontier.y();
        goal.pose.position.z = frontier.z();
        goal.pose.orientation.w = 1.0;

        goal_pub_->publish(goal);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}