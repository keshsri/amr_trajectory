#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp> 
#include "trajectory_package/srv/save_trajectory.hpp"

// Alias for convenience
using json = nlohmann::json;

class TrajectoryReader : public rclcpp::Node
{
public:
    TrajectoryReader() : Node("trajectory_reader")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("trajectory_file", "/path/to/trajectory.csv");
        this->declare_parameter<std::string>("marker_topic", "/trajectory_markers");

        // Retrieve parameters
        this->get_parameter("trajectory_file", trajectory_file_);
        this->get_parameter("marker_topic", marker_topic_);

        // Publisher for MarkerArray to visualize the trajectory
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

        // Service for reading trajectory files
        service_ = this->create_service<trajectory_package::srv::SaveTrajectory>(
            "read_trajectory", std::bind(&TrajectoryReader::read_trajectory_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Trajectory Reader Node has been started.");
    }

private:
    // Service callback to read trajectory from file
    void read_trajectory_callback(const std::shared_ptr<trajectory_package::srv::SaveTrajectory::Request> request,
                                  std::shared_ptr<trajectory_package::srv::SaveTrajectory::Response> response)
    {
        std::string file_path = request->filename;

        // Check if the file exists
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            response->success = false;
            response->message = "File does not exist.";
            RCLCPP_ERROR(this->get_logger(), "File %s does not exist.", file_path.c_str());
            return;
        }

        std::vector<std::tuple<double, double, double>> waypoints;

        // Determine file format and read data accordingly
        if (file_path.find(".csv") != std::string::npos)
        {
            waypoints = read_csv(file_path);
        }
        else if (file_path.find(".json") != std::string::npos)
        {
            waypoints = read_json(file_path);
        }
        else
        {
            response->success = false;
            response->message = "Unsupported file format. Use .csv or .json";
            RCLCPP_ERROR(this->get_logger(), "Unsupported file format.");
            return;
        }

        // Publish trajectory for visualization
        publish_trajectory(waypoints);

        response->success = true;
        response->message = "Trajectory successfully read and published.";
    }

    // Read trajectory data from CSV file
    std::vector<std::tuple<double, double, double>> read_csv(const std::string &file_path)
    {
        std::ifstream file(file_path);
        std::vector<std::tuple<double, double, double>> waypoints;
        std::string line;

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            double x, y, z;
            ss >> x >> y >> z;
            waypoints.emplace_back(x, y, z);
        }

        RCLCPP_INFO(this->get_logger(), "Successfully read CSV file.");
        return waypoints;
    }

    // Read trajectory data from JSON file
    std::vector<std::tuple<double, double, double>> read_json(const std::string &file_path)
    {
        std::ifstream file(file_path);
        json json_data;
        file >> json_data;

        std::vector<std::tuple<double, double, double>> waypoints;

        for (const auto &point : json_data["waypoints"])
        {
            double x = point["x"];
            double y = point["y"];
            double z = point["z"];
            waypoints.emplace_back(x, y, z);
        }

        RCLCPP_INFO(this->get_logger(), "Successfully read JSON file.");
        return waypoints;
    }

    // Publish the read trajectory for visualization in RViz
    void publish_trajectory(const std::vector<std::tuple<double, double, double>> &waypoints)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;  // Line width
        marker.color.a = 1.0;   // Alpha
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Add waypoints to the Marker
        for (const auto &waypoint : waypoints)
        {
            geometry_msgs::msg::Point point;
            point.x = std::get<0>(waypoint);
            point.y = std::get<1>(waypoint);
            point.z = std::get<2>(waypoint);
            marker.points.push_back(point);
        }

        marker_array.markers.push_back(marker);

        // Publish the trajectory for visualization
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Trajectory published for visualization.");
    }

    std::string trajectory_file_;
    std::string marker_topic_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<trajectory_package::srv::SaveTrajectory>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
