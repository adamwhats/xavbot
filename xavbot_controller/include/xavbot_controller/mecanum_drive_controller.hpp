#ifndef XAVBOT_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
#define XAVBOT_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace mecanum_drive_controller
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class MecanumDriveController : public controller_interface::ControllerInterface
  {
    using Twist = geometry_msgs::msg::Twist;

  public:
    MecanumDriveController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update() override;

    controller_interface::return_type init(const std::string & controller_name) override;

    CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_error(
      const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State & previous_state) override;

  protected:
      rclcpp::Subscription<Twist>::SharedPtr velocity_command_subsciption_;
      realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;

      std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
      std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;

      std::string fl_name_;
      std::string fr_name_;
      std::string rl_name_;
      std::string rr_name_;
      double wheel_radius_;
      double wheelbase_width_;
      double wheelbase_length_;

      Eigen::Matrix<double, 4, 3> T_inv_;
      Eigen::Matrix<double, 3, 4> T_fw_;
      Eigen::Vector3d odom_kinematics_;
      rclcpp::Time last_time_;
      double heading_ = 0;
      geometry_msgs::msg::Pose odom_pose_;

  };
}  // namespace mecanum_drive_controller
#endif  // XAVBOT_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_