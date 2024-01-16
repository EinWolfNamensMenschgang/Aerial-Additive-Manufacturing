#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class PayloadPositionPublisherNode : public rclcpp::Node {
public:
    nav_msgs::msg::Odometry payloadPositionFromOdoms(const nav_msgs::msg::Odometry::SharedPtr odom1, const nav_msgs::msg::Odometry::SharedPtr odom2, const nav_msgs::msg::Odometry::SharedPtr odom3){
        nav_msgs::msg::Odometry payloadOdom;
        payloadOdom.header.frame_id = odom1->header.frame_id;
        payloadOdom.header.stamp = odom1->header.stamp;
        payloadOdom.pose.pose.position.x = (odom1->pose.pose.position.x + odom2->pose.pose.position.x + odom3->pose.pose.position.x)/3;
        payloadOdom.pose.pose.position.y = (odom1->pose.pose.position.y + odom2->pose.pose.position.y + odom3->pose.pose.position.y)/3;
        payloadOdom.pose.pose.position.z = (odom1->pose.pose.position.z + odom2->pose.pose.position.z + odom3->pose.pose.position.z)/3;
        return payloadOdom;
    }       
    using approximate_policy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>;
    typedef message_filters::Synchronizer<approximate_policy> Synchronizer1;
    typedef message_filters::Synchronizer<approximate_policy> Synchronizer2;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom1_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom2_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom3_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom1gt_sub_; // for groundtruth
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom2gt_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom3gt_sub_;
    //typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry ,nav_msgs::msg::Odometry, nav_msgs::msg::Odometry> approximate_policy;
    std::unique_ptr<Synchronizer1> sync1;
    std::unique_ptr<Synchronizer2> sync2;
    PayloadPositionPublisherNode()
        : Node("payload_position_publisher_node") {
            odom1_sub_.subscribe(this, "/drone1/base_odom");
            odom2_sub_.subscribe(this, "/drone2/base_odom");
            odom3_sub_.subscribe(this, "/drone3/base_odom");
            odom1gt_sub_.subscribe(this, "/drone1/odom");
            odom2gt_sub_.subscribe(this, "/drone2/odom");
            odom3gt_sub_.subscribe(this, "/drone3/odom");
            //sync.reset(new message_filters::Synchronizer<approximate_policy>syncApproximate(approximate_policy(10), odom1_sub_, odom2_sub_, odom3_sub_));
            sync1.reset(new message_filters::Synchronizer<approximate_policy>(approximate_policy(10), odom1_sub_, odom2_sub_, odom3_sub_));
            sync1->registerCallback(&PayloadPositionPublisherNode::topic_callback, this);
            sync1->setMaxIntervalDuration(rclcpp::Duration(1,0));
            sync2.reset(new message_filters::Synchronizer<approximate_policy>(approximate_policy(10), odom1gt_sub_, odom2gt_sub_, odom3gt_sub_));
            sync2->registerCallback(&PayloadPositionPublisherNode::groundtruth_callback, this);
            //syncApproximate.registerCallback(&PayloadPositionPublisherNode::topic_callback, this);
            //syncApproximate.setMaxIntervalDuration(rclcpp::Duration(1,0));
        payloadPositionPublisher_ = create_publisher<nav_msgs::msg::Odometry>("/payload/odom", 1);
        payloadPositionGTPublisher_ = create_publisher<nav_msgs::msg::Odometry>("/payload/odom/groundtruth", 1);


    }

    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr &odom1, const 
    nav_msgs::msg::Odometry::SharedPtr &odom2, const nav_msgs::msg::Odometry::SharedPtr &odom3)
    { 
        nav_msgs::msg::Odometry payloadOdom;
        payloadOdom = payloadPositionFromOdoms(odom1, odom2, odom3);
        payloadPositionPublisher_->publish(payloadOdom);
    }

    void groundtruth_callback(const nav_msgs::msg::Odometry::SharedPtr &odom1, const 
    nav_msgs::msg::Odometry::SharedPtr &odom2, const nav_msgs::msg::Odometry::SharedPtr &odom3)
    { 
        nav_msgs::msg::Odometry payloadOdom;
        payloadOdom = payloadPositionFromOdoms(odom1, odom2, odom3);
        payloadPositionGTPublisher_->publish(payloadOdom);
    }

private:

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr payloadPositionPublisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr payloadPositionGTPublisher_;




};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PayloadPositionPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
