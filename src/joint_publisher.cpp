#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class FourWheelController : public rclcpp::Node
{
public:
  FourWheelController()
  : Node("four_wheel_controller"),
    wheel_separation_(0.4),    // Distance between left/right wheels (2 * wheel_offset_y)
    wheel_base_(0.6),          // Distance between front/rear wheels
    wheel_radius_(0.1)
  {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Cmd_vel: lin.x=%.2f ang.z=%.2f", 
                     msg->linear.x, msg->angular.z);
      });

    timer_ = this->create_wall_timer(
      50ms, [this]() { this->updateWheelVelocities(); });

    declare_parameter("wheel_names", std::vector<std::string>{
      "first_wheel_joint", "second_wheel_joint", 
      "third_wheel_joint", "fourth_wheel_joint"});

    // Initialize wheel positions for all four wheels
    current_positions_ = {0.0, 0.0, 0.0, 0.0};
  }

private:
  void updateWheelVelocities()
  {
    const double dt = 0.05; // Timer period: 50ms
    const double linear = last_cmd_.linear.x;
    const double angular = last_cmd_.angular.z;

    // Four-wheel skid-steer kinematics:
    // For left wheels: (linear - angular * (wheel_separation_/2)) / wheel_radius_
    // For right wheels: (linear + angular * (wheel_separation_/2)) / wheel_radius_
    const double left_speed = (linear - angular * wheel_separation_ / 2.0) / wheel_radius_;
    const double right_speed = (linear + angular * wheel_separation_ / 2.0) / wheel_radius_;

    // Map computed speeds to individual wheels.
    // Front wheels are inverted (due to URDF axis inversion)
    std::vector<double> velocities = {
      -right_speed,   // Front-right (first_wheel_joint)
      -left_speed,    // Front-left (second_wheel_joint)
      left_speed,     // Rear-left (third_wheel_joint)
      right_speed     // Rear-right (fourth_wheel_joint)
    };

    // Integrate velocities over dt to update positions
    for (size_t i = 0; i < current_positions_.size(); ++i) {
      current_positions_[i] += velocities[i] * dt;
    }

    publishJointTrajectory(velocities, current_positions_);
  }

  void publishJointTrajectory(const std::vector<double>& velocities,
                              const std::vector<double>& positions)
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = get_parameter("wheel_names").as_string_array();

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration(100ms);
    point.velocities = velocities;
    point.positions = positions;  // Provide positions to avoid mismatch errors

    msg.points.push_back(point);
    publisher_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), "Wheel velocities: FR=%.2f FL=%.2f RL=%.2f RR=%.2f",
                 velocities[0], velocities[1], velocities[2], velocities[3]);
  }

  // Member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  geometry_msgs::msg::Twist last_cmd_;

  // Robot parameters
  double wheel_separation_;
  double wheel_base_;
  double wheel_radius_;

  // Current wheel positions (integrated over time)
  std::vector<double> current_positions_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FourWheelController>());
  rclcpp::shutdown();
  return 0;
}
