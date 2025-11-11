#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <signal.h>
#include <mutex>
#include <fstream>
#include <vector>
#include <sstream>

struct FrameData {
    sensor_msgs::msg::Image image;
    sensor_msgs::msg::Imu imu;
    std_msgs::msg::Float64 altitude;
    geometry_msgs::msg::Pose gt_pose;
};

class DataRecorder : public rclcpp::Node {
public:
    DataRecorder() : Node("data_recorder") {
        using std::placeholders::_1;

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10, std::bind(&DataRecorder::image_callback, this, _1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", 50, std::bind(&DataRecorder::imu_callback, this, _1));
        alt_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/rel_alt", 10, std::bind(&DataRecorder::alt_callback, this, _1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/simulation_pose_info", 10, std::bind(&DataRecorder::pose_callback, this, _1));
        
        
        RCLCPP_INFO(get_logger(), "Recorder started");
    }

    ~DataRecorder() {
        write_to_file(output_file_name);
    }

    void write_to_file(const std::string &filename) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Could not open output file");
            return;
        }

        std::ostringstream buffer;
        buffer << "\n";
        for (const auto &f : recorded_data_) {
            buffer << f.timestamp.seconds() << ","
                   << f.roll << "," << f.pitch << "," << f.yaw << ","
                   << f.altitude << ","
                   << f.px << "," << f.py << "," << f.pz << "\n";
        }
        file << buffer.str();
        file.close();
        RCLCPP_INFO(get_logger(), "Saved %zu frames to %s", recorded_data_.size(), filename.c_str());
    }

private:
    std::mutex data_mutex_;
    std::vector<FrameData> recorded_data_;

    sensor_msgs::msg::Imu last_imu_;
    std_msgs::msg::Float64 last_alt_;
    geometry_msgs::msg::Pose last_pose_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        FrameData frame;
        std::lock_guard<std::mutex> lock(data_mutex_);
        frame.image = *msg;
        frame.imu = last_imu_;
        frame.altitude = last_alt_;
        frame.gt_pose = last_pose_;
        recorded_data_.push_back(frame);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_imu_ = *msg;
    }

    void alt_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_alt_ = *msg;
    }

    void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_pose_ = msg->poses[2];
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<DataRecorder> recorder_node = std::make_shared<DataRecorder>();
    rclcpp::spin(recorder_node);
    rclcpp::shutdown();
    return 0;
}
