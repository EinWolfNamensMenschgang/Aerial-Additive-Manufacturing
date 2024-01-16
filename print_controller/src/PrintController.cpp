#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gcode_to_path/msg/print_path.hpp"
#include "gcode_to_path/msg/print_pose.hpp"
#define PRINT_VELOCITY 0.02
#define NON_PRINT_VELOCITY 0.05
#define ACCURACY_DISTANCE 0.004

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode()
        : Node("path_follower_node") {
        printPathSubscription_ = create_subscription<gcode_to_path::msg::PrintPath>(
            "/print/path", 10, std::bind(&PathFollowerNode::onPrintPath, this, std::placeholders::_1));

        odomSubscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/payload/odom", 10, std::bind(&PathFollowerNode::onOdom, this, std::placeholders::_1));

        cmdVelPublisher1_ = create_publisher<geometry_msgs::msg::Twist>("/drone1/cmd_vel", 1);
        cmdVelPublisher2_ = create_publisher<geometry_msgs::msg::Twist>("/drone2/cmd_vel", 1);
        cmdVelPublisher3_ = create_publisher<geometry_msgs::msg::Twist>("/drone3/cmd_vel", 1);
        extrudePublisher_ = create_publisher<std_msgs::msg::Bool>("/print/extrude", 1);
        isExtruding_ = false;
        gotPath_ = false;
        currentPrintPoseIndex_ = 0;
        printPath_.path = {};
    }

private:
    void onPrintPath(const gcode_to_path::msg::PrintPath::SharedPtr msg) {
        if(!gotPath_){
            if (msg->path.empty()) {
                RCLCPP_WARN(get_logger(), "Received an empty PrintPath message.");
                    return;
        }
            else {
                printPath_ = *msg;
                currentPrintPoseIndex_ = 0;
                gotPath_ = true;
            }
        }
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (currentPrintPoseIndex_ >= printPath_.path.size()) {
            gotPath_ = false;
            return;
        }
        gcode_to_path::msg::PrintPose printPose = printPath_.path[currentPrintPoseIndex_];
        isExtruding_ = printPose.print;
        double linearVelX = printPose.pose.pose.position.x - msg->pose.pose.position.x;
        double linearVelY = printPose.pose.pose.position.y - msg->pose.pose.position.y;
        double linearVelZ = printPose.pose.pose.position.z - msg->pose.pose.position.z;
        double distance = std::sqrt(linearVelX * linearVelX + linearVelY * linearVelY + linearVelZ * linearVelZ);

        if (distance < ACCURACY_DISTANCE) {
            // Move to the next PrintPose
            currentPrintPoseIndex_++;


        } else {
            // Calculate and publish the linear velocities
            /*double magnitude = std::sqrt(linearVelX * linearVelX + linearVelY * linearVelY + linearVelZ * linearVelZ);
                double normalizedLinearVelX = magnitude > 0.0 ? linearVelX / magnitude : 0.0;
                double normalizedLinearVelY = magnitude > 0.0 ? linearVelY / magnitude : 0.0;
                double normalizedLinearVelZ = magnitude > 0.0 ? linearVelZ / magnitude : 0.0;*/
            if(isExtruding_){
                linearVelX = linearVelX * PRINT_VELOCITY;
                linearVelY = linearVelY * PRINT_VELOCITY;
                linearVelZ = linearVelZ * PRINT_VELOCITY;
            }else{
                linearVelX = linearVelX * NON_PRINT_VELOCITY;
                linearVelY = linearVelY * NON_PRINT_VELOCITY;
                linearVelZ = linearVelZ * NON_PRINT_VELOCITY;
            }

            geometry_msgs::msg::Twist twistMsg;
            twistMsg.linear.x = linearVelX;
            twistMsg.linear.y = linearVelY;
            twistMsg.linear.z = linearVelZ;
            twistMsg.angular.z = 0.0;
            cmdVelPublisher1_->publish(twistMsg);
            cmdVelPublisher2_->publish(twistMsg);
            cmdVelPublisher3_->publish(twistMsg);

            // Publish the extrude value from the current PrintPose
            std_msgs::msg::Bool extrudeMsg;
            extrudeMsg.data = printPose.print;
            extrudePublisher_->publish(extrudeMsg);
        }
    }

    rclcpp::Subscription<gcode_to_path::msg::PrintPath>::SharedPtr printPathSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher2_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher3_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr extrudePublisher_;

    gcode_to_path::msg::PrintPath printPath_;
    bool gotPath_;
    bool isExtruding_;
    size_t currentPrintPoseIndex_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
