#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class InitialPosePublisher : public rclcpp::Node {
public:
    InitialPosePublisher() : Node("initial_pose_pub_node") {
        publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
        subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point",
            rclcpp::QoS(rclcpp::SensorDataQoS()),
            std::bind(&InitialPosePublisher::callback, this, std::placeholders::_1));
    }

private:
    void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received Data:\n X : %f \n Y : %f \n Z : %f", msg->point.x, msg->point.y, msg->point.z);
        publish(msg->point.x, msg->point.y);
    }

    void publish(double x, double y) {
        auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
        msg->header.frame_id = "/map";
        msg->pose.pose.position.x = x;
        msg->pose.pose.position.y = y;
        RCLCPP_INFO(get_logger(), "Publishing Initial Position \n X= %f \n Y= %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
        publisher_->publish(std::move(msg));
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
