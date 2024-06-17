#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gcode_to_path/msg/print_path.hpp"
#include "gcode_to_path/msg/print_pose.hpp"

class PrintVisualizationNode : public rclcpp::Node {
public:
    PrintVisualizationNode()
        : Node("print_visualization_node") {
        extrudeSubscription_ = create_subscription<std_msgs::msg::Bool>(
            "/print/extrude", 10, std::bind(&PrintVisualizationNode::onExtrude, this, std::placeholders::_1));

        odomSubscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/drone1/camera_odom", 10, std::bind(&PrintVisualizationNode::onOdom, this, std::placeholders::_1));

        markerPublisher_ = create_publisher<visualization_msgs::msg::Marker>("/print/visualization", 0);

        isExtruding_ = false;
        color_.a=1;
        color_.r=1;
        color_.g=0;
        color_.b=0;
        marker_.header.frame_id = "map";
        //marker_.lifetime = rclcpp::Duration::from_seconds(0);;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker_.scale.x = 0.001;
        marker_.scale.y = 0.001;
        marker_.scale.z = 0.001;
        marker_.color = color_;
        marker_points_.reserve(10000);
    }

private:
    void onExtrude(const std_msgs::msg::Bool::SharedPtr msg) {
        isExtruding_=msg->data;
        return;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if(isExtruding_){
            marker_points_.push_back(msg->pose.pose.position);
            marker_.points = marker_points_;
            markerPublisher_->publish(marker_);  
        }
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr extrudeSubscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPublisher_;
    
    gcode_to_path::msg::PrintPath printPath_;
    bool isExtruding_;
    std_msgs::msg::ColorRGBA color_;
    visualization_msgs::msg::Marker marker_;
    std::vector<geometry_msgs::msg::Point> marker_points_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrintVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
