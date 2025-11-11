#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mavros_msgs/msg/manual_control.hpp>

class AttitudeAltitudeHold : public rclcpp::Node
{
public:
    AttitudeAltitudeHold() : Node("pid_node")
    {
        // Parameters
        this->declare_parameter("target_altitude", 10.0);
        this->declare_parameter("target_roll", 0.0);   // radians
        this->declare_parameter("target_pitch", 0.0);  // radians
        this->declare_parameter("target_yaw", 0.0);    // radians

        // PID params
        this->declare_parameter("Kp_alt", 0.8);
        this->declare_parameter("Ki_alt", 0.05);
        this->declare_parameter("Kd_alt", 0.3);
        
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

        // QoS profile for MAVROS topics (Best Effort + larger queue)
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        alt_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/rel_alt", sensor_qos,
            std::bind(&AttitudeAltitudeHold::altitudeCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", sensor_qos,
            std::bind(&AttitudeAltitudeHold::imuCallback, this, std::placeholders::_1));

        manual_pub_ = create_publisher<mavros_msgs::msg::ManualControl>(
            "/drone/cmd_move", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&AttitudeAltitudeHold::controlLoop, this));
    }

private:
    void getParams()
    {
        target_altitude_ = get_parameter("target_altitude").as_double();
        target_roll_ = get_parameter("target_roll").as_double();
        target_pitch_ = get_parameter("target_pitch").as_double();
        target_yaw_ = get_parameter("target_yaw").as_double();

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
        double dt = 0.05;

        // Altitude PID
        double err_alt = target_altitude_ - current_altitude_;
        integral_alt_ += err_alt * dt;
        double deriv_alt = (err_alt - prev_err_alt_) / dt;
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
        double err_yaw = target_yaw_ - current_yaw_;
        integral_yaw_ += err_yaw * dt;
        double deriv_yaw = (err_yaw - prev_err_yaw_) / dt;
        prev_err_yaw_ = err_yaw;
        double yaw_out = Kp_yaw_ * err_yaw + Ki_yaw_ * integral_yaw_ + Kd_yaw_ * deriv_yaw;

        // Normalize outputs to [-1000,1000]
        auto clamp = [](double v) { return std::max(-1000.0, std::min(1000.0, v)); };
        mavros_msgs::msg::ManualControl msg;
        msg.x = clamp(pitch_out * 500);   // pitch
        msg.y = clamp(roll_out * 500);    // roll
        msg.z = clamp(throttle * 100);    // throttle
        msg.r = clamp(yaw_out * 200);     // yaw
        msg.buttons = 0;

        manual_pub_->publish(msg);

        RCLCPP_INFO(get_logger(),
                    "Alt %.2f/%.2f | RPY [%.2f %.2f %.2f] | Thr %.1f",
                    current_altitude_, target_altitude_,
                    current_roll_, current_pitch_, current_yaw_, msg.z);
    }

    // --- ROS2 Components ---
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- States ---
    double current_altitude_ = 0.0, current_roll_ = 0.0, current_pitch_ = 0.0, current_yaw_ = 0.0;
    double target_altitude_, target_roll_, target_pitch_, target_yaw_;

    // PID variables
    double Kp_alt_, Ki_alt_, Kd_alt_;
    double Kp_roll_, Ki_roll_, Kd_roll_;
    double Kp_pitch_, Ki_pitch_, Kd_pitch_;
    double Kp_yaw_, Ki_yaw_, Kd_yaw_;

    double prev_err_alt_ = 0, integral_alt_ = 0;
    double prev_err_roll_ = 0, integral_roll_ = 0;
    double prev_err_pitch_ = 0, integral_pitch_ = 0;
    double prev_err_yaw_ = 0, integral_yaw_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudeAltitudeHold>());
    rclcpp::shutdown();
    return 0;
}
