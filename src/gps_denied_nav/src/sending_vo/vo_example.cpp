/**
 * ROS2 Node for controlling drone using MAVLink
 * 
 * Compile with:
 * colcon build --packages-select drone_controller
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <thread>

// Kendi MAVLink connection class'ınızı include edin
#include "gps_denied_nav/drone_mav_connection.hpp"
#include <mavros_msgs/msg/manual_control.hpp>

class DroneControllerNode : public rclcpp::Node {
public:
    DroneControllerNode() 
        : Node("vo_sender_node")
    {

        // Subscribers
        cmd_move_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10,
            std::bind(&DroneControllerNode::cmdMoveCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/simulation_pose_info",
            std::bind(&DroneControllerNode::poseCallback, this, std::placeholders::_1)
        );

        // Publishers
        vo_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

        // Timer for publishing status
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DroneControllerNode::publishStatus, this));

        // Timer for sending manual control (10Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DroneControllerNode::sendManualControl, this));

        RCLCPP_INFO(this->get_logger(), "Drone Controller Node started!");
        RCLCPP_INFO(this->get_logger(), "Subscribe to: /cmd_vel (geometry_msgs/Twist)");
        RCLCPP_INFO(this->get_logger(), "Services: /arm_drone, /disarm_drone");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            target_pitch_ = msg->x;
            target_roll_ = msg->y;
            target_throttle_ = msg->z;
            target_yaw_ = msg->r;
    }

    void armCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        
        RCLCPP_INFO(this->get_logger(), "Arming drone...");

        // Önce ALT_HOLD moduna geç
        mavlink_conn_->set_mode("ALT_HOLD");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        if (mavlink_conn_->arm_vehicle()) {
            is_armed_ = true;
            response->success = true;
            response->message = "Drone armed successfully";
            RCLCPP_INFO(this->get_logger(), "Drone armed!");
        } else {
            response->success = false;
            response->message = "Failed to arm drone";
            RCLCPP_ERROR(this->get_logger(), "Failed to arm drone!");
        }
    }

    void sendManualControl() {
        if (mavlink_conn_) {
            // MAVLink manual control gönder
            // x: pitch, y: roll, z: throttle, r: yaw
            mavlink_conn_->send_manual_control(
                target_pitch_,
                target_roll_,
                target_throttle_,
                target_yaw_
            );
        }
    }

    void publishStatus() {
        auto msg = std_msgs::msg::String();
        msg.data = "Armed: " + std::string(is_armed_ ? "true" : "false") +
                   ", Mode: " + current_mode_ +
                   ", Throttle: " + std::to_string(target_throttle_) +
                   ", Pitch: " + std::to_string(target_pitch_) +
                   ", Roll: " + std::to_string(target_roll_) +
                   ", Yaw: " + std::to_string(target_yaw_);
        status_pub_->publish(msg);
    }

    // MAVLink connection
    std::unique_ptr<MAVLinkConnection> mavlink_conn_;

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr cmd_move_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State variables
    bool is_armed_;
    std::string current_mode_;
    int16_t target_pitch_ = 0;
    int16_t target_roll_ = 0;
    int16_t target_throttle_ = 0;
    int16_t target_altitude_ = 0;
    int16_t target_yaw_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}