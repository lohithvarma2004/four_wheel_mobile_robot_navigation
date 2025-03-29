#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class DifferentialDriveController : public rclcpp::Node
{
public:
  DifferentialDriveController()
  : Node("diff_drive_controller"), 
    count_(0),
    // Initial wheel positions
    current_positions_{0.0, 0.0, 0.0, 0.0},
    // Default parameters (you can later load these from a parameter server)
    wheel_radius_(0.1),
    wheel_offset_y_(0.2)  // Example: (chassis_width/2 + wheel_thickness/2). Adjust as needed.
  {
    // Publisher for the joint trajectory (to the joint trajectory controller)
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);

    // Subscriber to Twist commands (for mobile base movement)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

    // Timer callback at 100ms
    timer_ = this->create_wall_timer(
      100ms, std::bind(&DifferentialDriveController::timerCallback, this));

    // Initialize last_cmd with zeros
    last_cmd_.linear.x = 0.0;
    last_cmd_.angular.z = 0.0;
  }

private:
  // Callback to update the desired robot twist command
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Store the latest command
    last_cmd_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=%f, angular=%f",
                msg->linear.x, msg->angular.z);
  }

  // Timer callback: compute wheel velocities from the Twist command and publish a trajectory message
  void timerCallback()
  {
    // Differential drive kinematics:
    // For a given desired linear velocity v and angular velocity w,
    // left wheel velocity = (v + w * (wheel_offset_y_)) / wheel_radius_
    // right wheel velocity = (v - w * (wheel_offset_y_)) / wheel_radius_
    double v = last_cmd_.linear.x;
    double w = last_cmd_.angular.z;

    double left_wheel_velocity  = (v + w * wheel_offset_y_) / wheel_radius_;
    double right_wheel_velocity = (v - w * wheel_offset_y_) / wheel_radius_;

    // For our 4-wheel robot, assume:
    // Wheels 2 and 3 (left side) get left_wheel_velocity
    // Wheels 1 and 4 (right side) get right_wheel_velocity
    std::vector<double> wheel_velocities = {
      right_wheel_velocity, // first_wheel_joint
      left_wheel_velocity,  // second_wheel_joint
      left_wheel_velocity,  // third_wheel_joint
      right_wheel_velocity  // fourth_wheel_joint
    };

    // For this simple demo, integrate velocity to update wheel positions (dt = 0.1s)
    double dt = 0.1;
    for (size_t i = 0; i < current_positions_.size(); ++i) {
      current_positions_[i] += wheel_velocities[i] * dt;
    }

    // Construct the trajectory message
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.joint_names = {"first_wheel_joint", "second_wheel_joint", "third_wheel_joint", "fourth_wheel_joint"};
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = current_positions_;
    point.velocities = wheel_velocities;
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    traj_msg.points.push_back(point);

    publisher_->publish(traj_msg);

    RCLCPP_INFO(this->get_logger(), "Published wheel velocities: [%.2f, %.2f, %.2f, %.2f]",
                wheel_velocities[0], wheel_velocities[1],
                wheel_velocities[2], wheel_velocities[3]);

    count_++;
  }

  // Members
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  size_t count_;
  std::vector<double> current_positions_;

  // Last received Twist command
  geometry_msgs::msg::Twist last_cmd_;

  // Parameters for kinematics
  double wheel_radius_;
  double wheel_offset_y_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialDriveController>());
  rclcpp::shutdown();
  return 0;
}
