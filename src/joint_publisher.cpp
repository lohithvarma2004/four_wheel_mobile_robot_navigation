#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
        : Node("joint_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&JointPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        
        // Add your robot's wheel joint names
        message.joint_names = {"first_wheel_joint", "second_wheel_joint", "third_wheel_joint", "fourth_wheel_joint"};

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        
        // Example movement: simple oscillating motion
        double position = 1.5 * (1 - cos(count_ * 0.1));

        point.positions = {position, -position, position, -position};
        point.velocities = {0.5, 0.5, 0.5, 0.5}; // Example velocities
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);

        message.points.push_back(point);
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing Joint Positions: %f, %f, %f, %f", position, -position, position, -position);

        count_ += 1;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}
