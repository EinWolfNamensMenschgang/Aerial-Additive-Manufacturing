#include <cmath>
#include <vector>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gcode_to_path/msg/print_path.hpp"
#include "gcode_to_path/msg/print_pose.hpp"
#define PRINT_VELOCITY 0.6
#define NON_PRINT_VELOCITY 0.6
#define ACCURACY_DISTANCE 0.15

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode()
        : Node("path_follower_node") {
        printPathSubscription_ = create_subscription<gcode_to_path::msg::PrintPath>(
            "/print/path", 10, std::bind(&PathFollowerNode::onPrintPath, this, std::placeholders::_1));

        odomSubscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/drone1/base_odom", 1, std::bind(&PathFollowerNode::onOdom, this, std::placeholders::_1));

        cmdVelPublisher_ = create_publisher<geometry_msgs::msg::Twist>("/drone1/cmd_vel", 1);
        extrudePublisher_ = create_publisher<std_msgs::msg::Bool>("/print/extrude", 1);
        markerPublisher_ = create_publisher<visualization_msgs::msg::Marker>("/marker", 0);
        isExtruding_ = false;
        gotPath_ = false;
        currentPrintPoseIndex_ = 0;
        printPath_.path = {};
        color_.a = 1;
        color_.r = 1;
        color_.g = 0;
        color_.b = 0;
        marker_.header.frame_id = "map";
        // marker_.lifetime = rclcpp::Duration::from_seconds(0);;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker_.scale.x = 0.001;
        marker_.scale.y = 0.001;
        marker_.scale.z = 0.001;
        marker_.color = color_;
        marker_points_.reserve(10000);
    }

private:
    void onPrintPath(const gcode_to_path::msg::PrintPath::SharedPtr msg) {
        //RCLCPP_INFO(get_logger(), "Received PrintPath message");
        if (!gotPath_) {
            if (msg->path.empty()) {
                RCLCPP_WARN(get_logger(), "Received an empty PrintPath message.");
                return;
            } else {
                printPath_ = *msg;
                currentPrintPoseIndex_ = 0;
                gotPath_ = true;
                RCLCPP_INFO(get_logger(), "Stored the print path with %ld poses", printPath_.path.size());
            }
        }
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 mat(tf2_quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (currentPrintPoseIndex_ >= printPath_.path.size()) {
            gotPath_ = false;
            //RCLCPP_INFO(get_logger(), "Completed the print path.");
            return;
        }
        gcode_to_path::msg::PrintPose printPose = printPath_.path[currentPrintPoseIndex_];
        isExtruding_ = printPose.print;

        //RCLCPP_DEBUG(get_logger(), "Current PrintPose index: %ld", currentPrintPoseIndex_);
        RCLCPP_DEBUG(get_logger(), "Print Pose - x: %f, y: %f, z: %f", printPose.pose.pose.position.x, printPose.pose.pose.position.y, printPose.pose.pose.position.z);
        RCLCPP_DEBUG(get_logger(), "Drone Pose - x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        double linearVelX = printPose.pose.pose.position.x - msg->pose.pose.position.x;
        double linearVelY = printPose.pose.pose.position.y - msg->pose.pose.position.y;
        double linearVelZ = printPose.pose.pose.position.z - msg->pose.pose.position.z;
        double yaw = getYawFromQuaternion(msg->pose.pose.orientation);
        double distance = std::sqrt(linearVelX * linearVelX + linearVelY * linearVelY + linearVelZ * linearVelZ);

        //RCLCPP_DEBUG(get_logger(), "Distance to next PrintPose: %f", distance);

        if (distance < ACCURACY_DISTANCE) {
            RCLCPP_INFO(get_logger(), "Reached PrintPose index: %ld", currentPrintPoseIndex_);
            // Move to the next PrintPose
            currentPrintPoseIndex_++;
        } else {
            // Calculate and publish the linear velocities
            if (isExtruding_) {
                linearVelX = linearVelX * PRINT_VELOCITY;
                linearVelY = linearVelY * PRINT_VELOCITY;
                linearVelZ = linearVelZ * PRINT_VELOCITY;
                RCLCPP_DEBUG(get_logger(), "Extruding: true");
            } else {
                linearVelX = linearVelX * NON_PRINT_VELOCITY;
                linearVelY = linearVelY * NON_PRINT_VELOCITY;
                linearVelZ = linearVelZ * NON_PRINT_VELOCITY;
                RCLCPP_DEBUG(get_logger(), "Extruding: false");
            }

            geometry_msgs::msg::Twist twistMsg;
            twistMsg.linear.x = linearVelX * cos(yaw) + linearVelY * sin(yaw);
            twistMsg.linear.y = -linearVelX * sin(yaw) + linearVelY * cos(yaw);
            twistMsg.linear.z = linearVelZ;
            twistMsg.angular.z = 0.0;
            cmdVelPublisher_->publish(twistMsg);

            //RCLCPP_INFO(get_logger(), "Published cmd_vel - linear.x: %f, linear.y: %f, linear.z: %f", linearVelX, linearVelY, linearVelZ);

            // Publish the extrude value from the current PrintPose
            std_msgs::msg::Bool extrudeMsg;
            extrudeMsg.data = printPose.print;
            extrudePublisher_->publish(extrudeMsg);

            //RCLCPP_INFO(get_logger(), "Published extrude message: %s", extrudeMsg.data ? "true" : "false");

            if (isExtruding_) {
                marker_points_.push_back(msg->pose.pose.position);
                marker_.points = marker_points_;
                markerPublisher_->publish(marker_);
                //RCLCPP_INFO(get_logger(), "Published marker at position - x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            }
        }
    }

    rclcpp::Subscription<gcode_to_path::msg::PrintPath>::SharedPtr printPathSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr extrudePublisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPublisher_;

    gcode_to_path::msg::PrintPath printPath_;
    bool gotPath_;
    bool isExtruding_;
    size_t currentPrintPoseIndex_;
    std_msgs::msg::ColorRGBA color_;
    visualization_msgs::msg::Marker marker_;
    std::vector<geometry_msgs::msg::Point> marker_points_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting PathFollowerNode...");
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down PathFollowerNode...");
    rclcpp::shutdown();
    return 0;
}
