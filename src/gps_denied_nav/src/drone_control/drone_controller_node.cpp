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
#include <memory>

#include "gps_denied_nav/drone_mav_connection.hpp"

class DroneControllerNode : public rclcpp::Node {
public:
    DroneControllerNode() 
        : Node("drone_controller_node")
    {
        // MAVLink bağlantısını başlat
        try {
            mavlink_conn_ = std::make_unique<MAVLinkConnection>("127.0.0.1", 14551);
            
            RCLCPP_INFO(this->get_logger(), "Waiting for heartbeat...");
            if (!mavlink_conn_->wait_heartbeat()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive heartbeat!");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Connected to drone (System ID: %d)", 
                       mavlink_conn_->get_target_system());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Connection error: %s", e.what());
            return;
        }

        // QoS profile for MAVROS topics (Best Effort + larger queue)
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        // Subscribers
        cmd_move_sub_ = this->create_subscription<mavros_msgs::msg::ManualControl>(
            "drone/cmd_move", 10,
            std::bind(&DroneControllerNode::cmdMoveCallback, this, std::placeholders::_1));
        

        alt_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/rel_alt", sensor_qos,
            std::bind(&DroneControllerNode::altitudeCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", sensor_qos,
            std::bind(&DroneControllerNode::imuCallback, this, std::placeholders::_1));

        // PID params
        this->declare_parameter("Kp_alt", 0.4);
        this->declare_parameter("Ki_alt", 0.02);
        this->declare_parameter("Kd_alt", 0.1);
        
        // Roll PID
        this->declare_parameter("Kp_roll", 1.0);
        this->declare_parameter("Ki_roll", 0.05);
        this->declare_parameter("Kd_roll", 0.2);
        
        // Pitch PID
        this->declare_parameter("Kp_pitch", 1.0);
        this->declare_parameter("Ki_pitch", 0.05);
        this->declare_parameter("Kd_pitch", 0.2);
        
        // Yaw PID (typically needs different tuning)
        this->declare_parameter("Kp_yaw", 0.5);
        this->declare_parameter("Ki_yaw", 0.02);
        this->declare_parameter("Kd_yaw", 0.1);

        getParams();

        // Timer for sending manual control (20Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DroneControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Drone Controller Node started!");
        RCLCPP_INFO(this->get_logger(), "Subscribe to: drone/cmd_move (mavros_msgs/ManualControl, instead of throttle use altitude)");
    }

private:
    void cmdMoveCallback(const mavros_msgs::msg::ManualControl::SharedPtr msg) {
        target_pitch_ = msg->x;
        target_roll_ = msg->y;
        target_altitude_ = msg->z;
        target_yaw_ = msg->r;
    }

    void sendManualControl(float pitch, float roll, float throttle, float yaw) {
        if (mavlink_conn_) {
            mavlink_conn_->send_manual_control(
                pitch,
                roll,
                throttle,
                yaw
            );
        }
    }

    void getParams()
    {
        Kp_alt_ = get_parameter("Kp_alt").as_double();
        Ki_alt_ = get_parameter("Ki_alt").as_double();
        Kd_alt_ = get_parameter("Kd_alt").as_double();
        
        Kp_roll_ = get_parameter("Kp_roll").as_double();
        Ki_roll_ = get_parameter("Ki_roll").as_double();
        Kd_roll_ = get_parameter("Kd_roll").as_double();
        
        Kp_pitch_ = get_parameter("Kp_pitch").as_double();
        Ki_pitch_ = get_parameter("Ki_pitch").as_double();
        Kd_pitch_ = get_parameter("Kd_pitch").as_double();
        
        Kp_yaw_ = get_parameter("Kp_yaw").as_double();
        Ki_yaw_ = get_parameter("Ki_yaw").as_double();
        Kd_yaw_ = get_parameter("Kd_yaw").as_double();
    }

    // --- Callbacks ---
    void altitudeCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_altitude_ = msg->data;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert quaternion to roll/pitch/yaw
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        current_roll_ = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            current_pitch_ = std::copysign(M_PI / 2, sinp);
        else
            current_pitch_ = std::asin(sinp);

        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    // --- Control Loop ---
    void controlLoop()
    {   
        static rclcpp::Time last_time = this->now();
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        // Altitude PID
        double err_alt = target_altitude_ - current_altitude_;
        integral_alt_ += err_alt * dt;
        // anti wind-up
        integral_alt_ = clamp(integral_alt_, -10.0, 10.0);
        double deriv_alt = 0.7 * deriv_alt_prev + 0.3 * ((err_alt - prev_err_alt_) / dt);
        deriv_alt_prev = deriv_alt;
        prev_err_alt_ = err_alt;
        double throttle = Kp_alt_ * err_alt + Ki_alt_ * integral_alt_ + Kd_alt_ * deriv_alt;

        // Roll PID
        double err_roll = target_roll_ - current_roll_;
        integral_roll_ += err_roll * dt;
        double deriv_roll = (err_roll - prev_err_roll_) / dt;
        prev_err_roll_ = err_roll;
        double roll_out = Kp_roll_ * err_roll + Ki_roll_ * integral_roll_ + Kd_roll_ * deriv_roll;

        // Pitch PID
        double err_pitch = target_pitch_ - current_pitch_;
        integral_pitch_ += err_pitch * dt;
        double deriv_pitch = (err_pitch - prev_err_pitch_) / dt;
        prev_err_pitch_ = err_pitch;
        double pitch_out = Kp_pitch_ * err_pitch + Ki_pitch_ * integral_pitch_ + Kd_pitch_ * deriv_pitch;

        // Yaw PID
        double err_yaw = atan2(sin(target_yaw_ - current_yaw_),
                       cos(target_yaw_ - current_yaw_));
        integral_yaw_ += err_yaw * dt;
        double deriv_yaw = (err_yaw - prev_err_yaw_) / dt;
        prev_err_yaw_ = err_yaw;
        double yaw_out = Kp_yaw_ * err_yaw + Ki_yaw_ * integral_yaw_ + Kd_yaw_ * deriv_yaw;

        // Normalize outputs to [-1000,1000]
        auto clamp = [](double v, double min, double max) { return std::max(min, std::min(max, v)); };
        float x = clamp(pitch_out * 500, -1000.0f, 1000.0f);   // pitch
        float y = clamp(roll_out * 500, -1000.0f, 1000.0f);    // roll
        float z = clamp(throttle * 100, 0.0f, 1000.0f);        // throttle
        float r = clamp(yaw_out * 200, -1000.0f, 1000.0f);     // yaw

        sendManualControl(x, y, z, r);

        RCLCPP_INFO(get_logger(),
                    "Alt %.2f/%.2f | RPY [%.2f %.2f %.2f] | Thr %.1f",
                    current_altitude_, target_altitude_,
                    current_roll_, current_pitch_, current_yaw_, z);
    }

    double clamp(double v, double min, double max) { 
        return std::max(min, std::min(max, v));
    }
    
    // --- ROS2 Components ---
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr cmd_move_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // --- MAVLink connection ---
    std::unique_ptr<MAVLinkConnection> mavlink_conn_;

    // --- Current States ---
    double current_altitude_ = 0.0;
    double current_roll_ = 0.0;
    double current_pitch_ = 0.0;
    double current_yaw_ = 0.0;

    // --- Target States ---
    double target_altitude_ = 0.0;
    double target_roll_ = 0.0;
    double target_pitch_ = 0.0;
    double target_yaw_ = 0.0;

    // --- PID Gains ---
    double Kp_alt_, Ki_alt_, Kd_alt_;
    double Kp_roll_, Ki_roll_, Kd_roll_;
    double Kp_pitch_, Ki_pitch_, Kd_pitch_;
    double Kp_yaw_, Ki_yaw_, Kd_yaw_;

    // --- PID State ---
    double prev_err_alt_ = 0.0, integral_alt_ = 0.0, deriv_alt_prev = 0.0;
    double prev_err_roll_ = 0.0, integral_roll_ = 0.0;
    double prev_err_pitch_ = 0.0, integral_pitch_ = 0.0;
    double prev_err_yaw_ = 0.0, integral_yaw_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}