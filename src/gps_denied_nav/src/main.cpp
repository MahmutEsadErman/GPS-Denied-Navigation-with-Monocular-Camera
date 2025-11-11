/**
 * ROS2 Node for controlling drone using MAVLink
 * 
 * Compile with:
 * colcon build --packages-select gps_denied_nav
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <cmath>
#include <thread>
#include <iostream>

#include "rclcpp/serialization.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

struct FrameData {
    rclcpp::Time timestamp;
    sensor_msgs::msg::Image image;
    sensor_msgs::msg::Imu imu;
    std_msgs::msg::Float64 altitude;
};

class FollowPathNode : public rclcpp::Node {
public:
    FollowPathNode() : Node("follow_path_node"), timer_started_(false)
    {
        this->declare_parameter<std::string>("bag_file_path", "simple_path");

        // publishers
        manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
            "/drone/cmd_move",
            10
        );
        pose_publisher = this->create_publisher<geometry_msgs::msg::Pose>(
            "/target_pose",
            10
        );

        // Initialize the ROS2 bag reader
        reader_ = std::make_shared<rosbag2_cpp::Reader>();
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = this->get_parameter("bag_file_path").as_string();
        reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader_->open(storage_options);
        create_path_from_bag();
        
        // Wait for user input before starting
        RCLCPP_INFO(this->get_logger(), "Path created. Type 'y' and press Enter to start following the path:");
        std::string input;
        std::getline(std::cin, input);
        if (input == "y" || input == "Y") {
            RCLCPP_INFO(this->get_logger(), "Starting path following...");
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(33), // 30 Hz
                std::bind(&FollowPathNode::follow_path, this)
            );
            timer_started_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Path following not started. Expected 'y' but got: %s", input.c_str());
        }

    }

private:
    // Member variables
    std::vector<FrameData> path_data_;
    std::shared_ptr<rosbag2_cpp::Reader> reader_;
    rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    bool timer_started_;

    // Helper function to deserialize messages
    template<typename T>
    std::shared_ptr<T> deserializeMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg) {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        auto ros_message = std::make_shared<T>();
        rclcpp::Serialization<T> serialization;
        serialization.deserialize_message(&serialized_msg, ros_message.get());
        return ros_message;
    }

    void follow_path()
    {
        static size_t path_index = 0;

        if (path_data_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path data is empty");
            return;
        }

        // Loop through the path data
        const FrameData &current_frame = path_data_[path_index];

        // Convert quaternion to roll/pitch/yaw
        double qx = current_frame.imu.orientation.x;
        double qy = current_frame.imu.orientation.y;
        double qz = current_frame.imu.orientation.z;
        double qw = current_frame.imu.orientation.w;

        // Calculate roll
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        // Calculate pitch
        double sinp = 2.0 * (qw * qy - qz * qx);
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        // Calculate yaw
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // Create and publish manual control message
        mavros_msgs::msg::ManualControl manual_msg;
        manual_msg.x = pitch;  // Pitch value
        manual_msg.y = roll;   // Roll value
        manual_msg.z = current_frame.altitude.data;  // Altitude value
        manual_msg.r = yaw;    // Yaw value
        manual_pub_->publish(manual_msg);
        RCLCPP_INFO(this->get_logger(), "p: %.2f, r: %.2f, y: %.2f, alt: %.2f", pitch, roll, yaw, current_frame.altitude.data);

        // Increment path index
        path_index = (path_index + 1) % path_data_.size();
    }

    void create_path_from_bag()
    {
        FrameData frame;
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> last_imu_msg;
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> last_alt_msg;
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> last_pose_msg;
        
        if (!reader_ || !reader_->has_next()) {
            RCLCPP_WARN(this->get_logger(), "Reader not initialized or bag is empty");
            return;
        }
        
        while (reader_->has_next()) {
            auto msg = reader_->read_next();
            
            // Deserialize based on topic name
            if (msg->topic_name == "/camera/image") {
                frame = FrameData();
                frame.timestamp = this->get_clock()->now();
                auto image_msg = deserializeMessage<sensor_msgs::msg::Image>(msg);
                frame.image = *image_msg;
                
                if (last_imu_msg) {
                    auto imu_msg = deserializeMessage<sensor_msgs::msg::Imu>(last_imu_msg);
                    frame.imu = *imu_msg;
                }
                if (last_alt_msg) {
                    auto altitude_msg = deserializeMessage<std_msgs::msg::Float64>(last_alt_msg);
                    frame.altitude = *altitude_msg;
                }
                if (last_pose_msg) {
                    // Create a SerializedMessage object from the bag data
                    auto pose_msg = deserializeMessage<geometry_msgs::msg::PoseArray>(last_pose_msg);

                    // Publish the generic message
                    pose_publisher->publish(pose_msg->poses[2]);
                }
                path_data_.push_back(frame);
            }
            else if (msg->topic_name == "/mavros/imu/data") {
                last_imu_msg = msg;
            }
            else if (msg->topic_name == "/mavros/global_position/rel_alt") {
                last_alt_msg = msg;
            }
            else if (msg->topic_name == "/simulation_pose_info") {
                last_pose_msg = msg;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Created path with %zu frames", path_data_.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}