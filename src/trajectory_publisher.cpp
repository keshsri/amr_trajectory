#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "trajectory_package/srv/save_trajectory.hpp"

#include <vector>
#include <fstream>
#include <chrono>

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher")
    {
        // Subscriber to /odom topic
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectoryPublisher::odomCallback, this, std::placeholders::_1));

        // Publisher to publish the trajectory as marker array
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);

        // Service to save trajectory to a file
        save_service_ = this->create_service<trajectory_package::srv::SaveTrajectory>(
            "save_trajectory", std::bind(&TrajectoryPublisher::saveTrajectory, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Trajectory Publisher Node has been started.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<trajectory_package::srv::SaveTrajectory>::SharedPtr save_service_;

    std::vector<geometry_msgs::msg::Point> trajectory_points_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Store the current position from odometry
        geometry_msgs::msg::Point current_point;
        current_point.x = msg->pose.pose.position.x;
        current_point.y = msg->pose.pose.position.y;
        current_point.z = msg->pose.pose.position.z;

        trajectory_points_.push_back(current_point);

        // Publish the trajectory markers for visualization in RViz
        publishTrajectory();
    }

    void publishTrajectory()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05; // Line width
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Add points to the marker
        for (auto &point : trajectory_points_)
        {
            marker.points.push_back(point);
        }

        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }

    bool saveTrajectory(const std::shared_ptr<trajectory_package::srv::SaveTrajectory::Request> request,
                        std::shared_ptr<trajectory_package::srv::SaveTrajectory::Response> response)
    {
        std::ofstream file;
        file.open(request->filename);
        if (!file.is_open())
        {
            response->success = false;
            response->message = "Failed to open the file.";
            return true;
        }

        // Save trajectory points to file
        for (const auto &point : trajectory_points_)
        {
            file << point.x << "," << point.y << "," << point.z << "\n";
        }

        file.close();
        response->success = true;
        response->message = "Trajectory saved successfully.";
        return true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
