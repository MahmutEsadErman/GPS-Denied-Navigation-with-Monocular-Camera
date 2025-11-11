/**
 * ROS2 Node for controlling drone using MAVLink
 * 
 * Compile with:
 * colcon build --packages-select gps_denied_nav
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mavros_msgs/msg/manual_control.hpp>


class DroneControllerNode : public rclcpp::Node {
public:
    DroneControllerNode() : Node("drone_controller_node")
    {
        
    }

private:
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}