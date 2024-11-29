#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace leo_ctrl_cpp {

class Controller : public rclcpp::Node {
public:
    Controller()
    : Node("controller_node")
    {
        // Declare parameters
        this->declare_parameter<double>("wheel_base", 0.5);
        this->declare_parameter<double>("max_linear_velocity", 1.0);
        this->declare_parameter<double>("max_angular_velocity", 1.0);

        wheel_base_ = this->get_parameter("wheel_base").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();

        // Subscribe to Twist messages
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&Controller::twistCallback, this, std::placeholders::_1)
        );

        // Publish motor commands
        motor_command_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_commands", 10);

        RCLCPP_INFO(this->get_logger(), "Controller node initialized.");
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        // Compute wheel velocities
        double left_wheel_velocity = linear_x - (angular_z * wheel_base_ / 2.0);
        double right_wheel_velocity = linear_x + (angular_z * wheel_base_ / 2.0);

        // Map velocities to PWM
        int left_pwm = mapVelocityToPWM(left_wheel_velocity);
        int right_pwm = mapVelocityToPWM(right_wheel_velocity);

        // Publish motor commands
        std_msgs::msg::Int32MultiArray motor_commands;
        motor_commands.data.push_back(left_pwm);
        motor_commands.data.push_back(right_pwm);

        motor_command_publisher_->publish(motor_commands);

        RCLCPP_DEBUG(this->get_logger(), "Linear x: %f, Angular z: %f", linear_x, angular_z);
        RCLCPP_DEBUG(this->get_logger(), "Left PWM: %d, Right PWM: %d", left_pwm, right_pwm);
    }

    int mapVelocityToPWM(double velocity)
    {
        int pwm = static_cast<int>((velocity / max_linear_velocity_) * 255.0);
        return std::clamp(pwm, -255, 255);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_command_publisher_;

    double wheel_base_;
    double max_linear_velocity_;
    double max_angular_velocity_;
};

} // namespace leo_ctrl_cpp

#endif // CONTROLLER_HPP