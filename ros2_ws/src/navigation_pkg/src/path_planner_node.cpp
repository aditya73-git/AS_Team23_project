#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

// Custom hash for Octomap Keys so we can use them in std::unordered_map
struct KeyHash {
    std::size_t operator()(const octomap::OcTreeKey& k) const {
        return k[0] ^ k[1] ^ k[2];
    }
};

// Node structure for A*
struct AStarNode {
    octomap::OcTreeKey key;
    double f_score;
    bool operator>(const AStarNode& other) const { return f_score > other.f_score; }
};

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner_node") {
        this->declare_parameter<std::string>("global_frame", "world");
        this->declare_parameter<std::string>("robot_base_frame", "true_body");
        
        global_frame_ = this->get_parameter("global_frame").as_string();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_full", 10, std::bind(&PathPlannerNode::octomapCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_waypoint", 10, std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(this->get_logger(), "3D A* Path Planner Ready.");
    }

private:
    std::shared_ptr<octomap::OcTree> current_octree_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string global_frame_, robot_base_frame_;

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::fullMsgToMap(*msg));
        if (tree) {
            current_octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree.release()));
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!current_octree_) {
            RCLCPP_WARN(this->get_logger(), "No Octomap received yet!");
            return;
        }

        // 1. Get Drone's Current Position (Start)
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            return;
        }

        octomap::point3d start_pt(tf_stamped.transform.translation.x, tf_stamped.transform.translation.y, tf_stamped.transform.translation.z);
        octomap::point3d goal_pt(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        // 2. Run 3D A* Algorithm
        std::vector<octomap::point3d> path = computeAStarPath(start_pt, goal_pt);

        // 3. Publish the Path
        if (!path.empty()) {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = global_frame_;

            for (const auto& p : path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = p.x();
                pose.pose.position.y = p.y();
                pose.pose.position.z = p.z();
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }
            path_pub_->publish(path_msg);
            RCLCPP_INFO(this->get_logger(), "Published safe path with %zu waypoints.", path.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid path found to the frontier!");
        }
    }

    std::vector<octomap::point3d> computeAStarPath(const octomap::point3d& start, const octomap::point3d& goal) {
        std::vector<octomap::point3d> path;
        
        octomap::OcTreeKey start_key = current_octree_->coordToKey(start);
        octomap::OcTreeKey goal_key = current_octree_->coordToKey(goal);

        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
        std::unordered_map<octomap::OcTreeKey, double, KeyHash> g_score;
        std::unordered_map<octomap::OcTreeKey, octomap::OcTreeKey, KeyHash> came_from;

        g_score[start_key] = 0.0;
        open_set.push({start_key, heuristic(start_key, goal_key)});

        // Safety limit to prevent infinite loops in massive caves
        int max_iterations = 50000; 
        int iterations = 0;

        while (!open_set.empty() && iterations < max_iterations) {
            iterations++;
            octomap::OcTreeKey current = open_set.top().key;
            open_set.pop();

            if (current == goal_key) {
                return reconstructPath(came_from, current);
            }

            // 26-Connected 3D Grid Search
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;

                        octomap::OcTreeKey neighbor = current;
                        neighbor[0] += dx; neighbor[1] += dy; neighbor[2] += dz;

                        // Collision Check: Treat 'Unknown' space as free for exploration
                        octomap::OcTreeNode* node = current_octree_->search(neighbor);
                        if (node && current_octree_->isNodeOccupied(node)) {
                            continue; // Hit a rock!
                        }

                        // Calculate distance (diagonal vs straight)
                        double dist = std::sqrt(dx*dx + dy*dy + dz*dz) * current_octree_->getResolution();
                        double tentative_g = g_score[current] + dist;

                        if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                            came_from[neighbor] = current;
                            g_score[neighbor] = tentative_g;
                            double f = tentative_g + heuristic(neighbor, goal_key);
                            open_set.push({neighbor, f});
                        }
                    }
                }
            }
        }
        return path; // Empty path if failed
    }

    double heuristic(const octomap::OcTreeKey& a, const octomap::OcTreeKey& b) {
        octomap::point3d pa = current_octree_->keyToCoord(a);
        octomap::point3d pb = current_octree_->keyToCoord(b);
        return pa.distance(pb);
    }

    std::vector<octomap::point3d> reconstructPath(
        const std::unordered_map<octomap::OcTreeKey, octomap::OcTreeKey, KeyHash>& came_from,
        octomap::OcTreeKey current) {
        
        std::vector<octomap::point3d> path;
        path.push_back(current_octree_->keyToCoord(current));

        while (came_from.find(current) != came_from.end()) {
            current = came_from.at(current);
            path.push_back(current_octree_->keyToCoord(current));
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}