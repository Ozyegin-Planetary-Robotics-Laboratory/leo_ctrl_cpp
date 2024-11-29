#ifndef JOYSTICK_INTERFACE_HPP
#define JOYSTICK_INTERFACE_HPP

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace leo_ctrl_cpp {

class JoystickInterface : public rclcpp::Node {
public:
    JoystickInterface()
    : Node("joystick_interface"),
      m_loop_rate(30.0),
      m_linear_scale(1.0),
      m_angular_scale(1.0) // Set default scale values
    {
        // Declare and get parameters
        this->declare_parameter<double>("linear_scale", 1.0);
        this->declare_parameter<double>("angular_scale", 1.0);

        m_linear_scale = this->get_parameter("linear_scale").as_double();
        m_angular_scale = this->get_parameter("angular_scale").as_double();

        // Subscribe to the "joy" topic
        m_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickInterface::joyCallback, this, std::placeholders::_1)
        );

        // Publish to the "cmd_vel" topic
        m_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Timer for the control loop
        m_control_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / m_loop_rate)),
            std::bind(&JoystickInterface::controlLoop, this)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_control_timer;  // Timer for the control loop
    std::mutex m_command_mutex;
    double m_loop_rate;
    double m_linear_scale;
    double m_angular_scale;
    geometry_msgs::msg::Twist m_twist;

    void joyCallback(const std::shared_ptr<sensor_msgs::msg::Joy> joy)
    {
        std::lock_guard<std::mutex> lock(m_command_mutex);

        if (joy->axes.size() > 1) { // Ensure there are enough axes
            m_twist.linear.x = joy->axes[1] * m_linear_scale;
            m_twist.angular.z = joy->axes[0] * m_angular_scale;

            RCLCPP_INFO(this->get_logger(), "axes[0]: %f, axes[1]: %f", joy->axes[0], joy->axes[1]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Joystick message has insufficient axes data.");
        }
    }

    // Control loop, called periodically by the timer
    void controlLoop()
    {
        auto twist = std::make_shared<geometry_msgs::msg::Twist>();
        {
            std::lock_guard<std::mutex> lock(m_command_mutex);
            twist->linear.x = m_twist.linear.x;
            twist->angular.z = m_twist.angular.z;
        }
        m_pub->publish(*twist);
    }
};

} // namespace leo_ctrl_cpp

#endif // JOYSTICK_INTERFACE_HPP