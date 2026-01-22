#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath> 

class DynamicTfPublisher : public rclcpp::Node {
public:
    DynamicTfPublisher() : Node("dynamic_tf_publisher") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // IMPOSTAZIONE QOS BEST EFFORT (Fondamentale per Gazebo)
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/fra2mo/odometry", 
            qos_profile,
            std::bind(&DynamicTfPublisher::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dynamic TF publisher initialized.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Recuperiamo i dati
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        geometry_msgs::msg::TransformStamped transform_stamped;
        
        // --- MODIFICA CRUCIALE: Usiamo il tempo del messaggio, NON l'ora attuale ---
        transform_stamped.header.stamp = msg->header.stamp; 
        // --------------------------------------------------------------------------
        
        transform_stamped.header.frame_id = "fra2mo/odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}