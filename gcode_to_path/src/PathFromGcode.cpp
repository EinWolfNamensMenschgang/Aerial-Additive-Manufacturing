#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "gcode_to_path/msg/print_pose.hpp"
#include "gcode_to_path/msg/print_path.hpp"

struct Point {
    double x;
    double y;
    double z;

    Point(double x, double y, double z) : x(x), y(y), z(z) {}
};

class GCodePathPublisher : public rclcpp::Node {
public:
    GCodePathPublisher(const std::string& inputFilePath)
        : Node("gcode_path_publisher"), inputFilePath_(inputFilePath) {
        publisher_ = create_publisher<gcode_to_path::msg::PrintPath>("/print/path", 10);
        navPathPublisher_ = create_publisher<nav_msgs::msg::Path>("/nav/path", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&GCodePathPublisher::publishPath, this));
    }

private:
    void publishPath() {
        std::vector<Point> points;
        double currentZ = 0.0;

        // Read each line from the file
        std::ifstream inputFile(inputFilePath_);
        if (!inputFile.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open the specified file.");
            return;
        }

        gcode_to_path::msg::PrintPath printPathMsg;
        nav_msgs::msg::Path navPathMsg;
        printPathMsg.header.stamp = this->now();
        printPathMsg.header.frame_id = "map"; // Set the frame ID as needed
        navPathMsg.header.stamp = this->now();
        navPathMsg.header.frame_id = "map";

        std::string line;
        while (std::getline(inputFile, line)) {
            // Check if the line starts with G0 or G1
            if (line.compare(0, 2, "G0") == 0 || line.compare(0, 2, "G1") == 0) {
                // Extract X, Y, and Z coordinates from the line
                double x = 0.0, y = 0.0, z = currentZ;
                std::istringstream iss(line);
                std::string token;
                while (iss >> token) {
                    if (token[0] == 'X') {
                        x = std::stod(token.substr(1));
                    } else if (token[0] == 'Y') {
                        y = std::stod(token.substr(1));
                    } else if (token[0] == 'Z') {
                        z = std::stod(token.substr(1));
                        currentZ = z;  // Update the current Z coordinate
                    }
                }

                // Create a PrintPose message and add it to the PrintPath
                gcode_to_path::msg::PrintPose printPoseMsg;
                printPoseMsg.pose.pose.position.x = x/1000;
                printPoseMsg.pose.pose.position.y = y/1000;
                printPoseMsg.pose.pose.position.z = z/1000;

                // Set the print field based on the G-code command
                printPoseMsg.print = (line.compare(0, 2, "G0") == 0) ? false : true;

                printPathMsg.path.push_back(printPoseMsg);

                geometry_msgs::msg::PoseStamped poseStampedMsg;
                poseStampedMsg.pose.position.x = x/1000;
                poseStampedMsg.pose.position.y = y/1000;
                poseStampedMsg.pose.position.z = z/1000;

                navPathMsg.poses.push_back(poseStampedMsg);
            }
        }

        // Close the input file
        inputFile.close();

        // Publish the PrintPath message
        publisher_->publish(printPathMsg);
        navPathPublisher_->publish(navPathMsg);
    }

    rclcpp::Publisher<gcode_to_path::msg::PrintPath>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr navPathPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string inputFilePath_;
};

int main(int argc, char* argv[]) {
    // Check if the correct number of command-line arguments is provided
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file_path.gcode>\n";
        return 1;
    }

    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GCodePathPublisher>(argv[1]);

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
