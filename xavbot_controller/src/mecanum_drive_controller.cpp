#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "mecanum_drive_controller.hpp"

namespace mecanum_drive_controller
{

  MecanumDriveController::MecanumDriveController() : controller_interface::ControllerInterface()
      , velocity_command_subsciption_(nullptr)
      , velocity_command_ptr_(nullptr)
      , odometry_publisher_(nullptr)
      , realtime_odometry_publisher_(nullptr)
  {

  }

  controller_interface::InterfaceConfiguration MecanumDriveController::command_interface_configuration() const
  {
      controller_interface::InterfaceConfiguration command_interfaces_config;
      command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

      command_interfaces_config.names.push_back(fl_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interfaces_config.names.push_back(fr_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interfaces_config.names.push_back(rl_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interfaces_config.names.push_back(rr_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

      return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration MecanumDriveController::state_interface_configuration() const
  {
      controller_interface::InterfaceConfiguration state_interfaces_config;
      state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

      state_interfaces_config.names.push_back(fl_name_ + "/" + hardware_interface::HW_IF_POSITION);
      state_interfaces_config.names.push_back(fl_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interfaces_config.names.push_back(fr_name_ + "/" + hardware_interface::HW_IF_POSITION);
      state_interfaces_config.names.push_back(fr_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interfaces_config.names.push_back(rl_name_ + "/" + hardware_interface::HW_IF_POSITION);
      state_interfaces_config.names.push_back(rl_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interfaces_config.names.push_back(rr_name_ + "/" + hardware_interface::HW_IF_POSITION);
      state_interfaces_config.names.push_back(rr_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

      return state_interfaces_config;
  }

  controller_interface::return_type MecanumDriveController::init(const std::string & controller_name)
  {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) 
    {
      return ret;
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type MecanumDriveController::update()
  {
    // Get the last velocity command
    auto velocity_command = velocity_command_ptr_.readFromRT();
    if (!velocity_command || !(*velocity_command)) {
        return controller_interface::return_type::OK;
    }

    // Calculate the wheel velocities
    // Kinematics from: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
    // const auto twist = (*velocity_command)->twist;
    const auto twist = *velocity_command->get();
    Eigen::Vector4d target_vels_;
    Eigen::Vector3d target_kinematics_ {twist.linear.x, twist.linear.y, twist.angular.z};
    target_vels_ = (1 / wheel_radius_) * T_inv_ * target_kinematics_;

    // Set the wheel velocities
    // TODO: Don't hardcode command_interface_ indexes 
    command_interfaces_[0].set_value(target_vels_[0]);
    command_interfaces_[1].set_value(target_vels_[1]);
    command_interfaces_[2].set_value(target_vels_[2]);
    command_interfaces_[3].set_value(target_vels_[3]);

    // Read the actual velocities
    // TODO: Don't hardcode command_interface_ indexes 
    double fl_wheel_velocity_actual = state_interfaces_[1].get_value();
    double fr_wheel_velocity_actual = state_interfaces_[3].get_value();
    double rl_wheel_velocity_actual = state_interfaces_[5].get_value();
    double rr_wheel_velocity_actual = state_interfaces_[7].get_value();

    // Forward Kinematics
    Eigen::Vector4d actual_vels_ = {
      fl_wheel_velocity_actual, 
      fr_wheel_velocity_actual,
      rl_wheel_velocity_actual,
      rr_wheel_velocity_actual
    };
    odom_kinematics_ = (wheel_radius_ / 4) * T_fw_ * actual_vels_;

    // Calculate odometry
    rclcpp::Time current_time_ = node_->get_clock()->now();
    if (last_time_.seconds() != 0)
    {
      const double dt = current_time_.seconds() - last_time_.seconds();
      last_time_ = current_time_;
      heading_ += odom_kinematics_[2] * dt;
      
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, heading_);
      odom_pose_.orientation = tf2::toMsg(orientation);
      odom_pose_.position.x += ((odom_kinematics_[0] * cos(heading_)) - (odom_kinematics_[1] * sin(heading_))) * dt;
      odom_pose_.position.y += ((odom_kinematics_[0] * sin(heading_)) + (odom_kinematics_[1] * cos(heading_))) * dt;


      // Publish odometry
      if (realtime_odometry_publisher_->trylock())
      {
        auto & odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.stamp = current_time_;
        odometry_message.header.frame_id = "odom";
        odometry_message.child_frame_id  = "base_footprint";
        odometry_message.pose.pose = odom_pose_;
        odometry_message.twist.twist.linear.x = odom_kinematics_[0];
        odometry_message.twist.twist.linear.y = odom_kinematics_[1];
        odometry_message.twist.twist.angular.z = odom_kinematics_[2];
        realtime_odometry_publisher_->unlockAndPublish();
      }
    }
    else
    {
      last_time_ = current_time_;
    }
    
    return controller_interface::return_type::OK;
  }

  CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Configure MecanumDriverController");

    fl_name_ = get_node()->get_parameter("fl_name").as_string();
    fr_name_ = get_node()->get_parameter("fr_name").as_string();
    rl_name_ = get_node()->get_parameter("rl_name").as_string();
    rr_name_ = get_node()->get_parameter("rr_name").as_string();
    if (fl_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'fl_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (fr_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'fr_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (rl_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'rl_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (rr_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'rr_name' parameter was empty");
        return CallbackReturn::ERROR;
    }

    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    wheelbase_width_ = get_node()->get_parameter("wheelbase_width").as_double();
    wheelbase_length_ = get_node()->get_parameter("wheelbase_length").as_double();
    if (wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_radius' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }
    if (wheelbase_width_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheelbase_width' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }
    if (wheelbase_length_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheelbase_length' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }

    // Populate kinematics constants
    double wheel_offset_ = (wheelbase_width_ + wheelbase_length_) / 2;
    T_inv_ << 1, -1, -wheel_offset_,
              1, 1, wheel_offset_,
              1, 1, -wheel_offset_,
              1, -1, wheel_offset_;
    T_fw_ << 1, 1, 1, 1,
          -1, 1, 1, -1,
          -1/wheel_offset_, 1/wheel_offset_, -1/wheel_offset_, 1/wheel_offset_;

    velocity_command_subsciption_ = get_node()->create_subscription<Twist>("/cmd_vel", 10, [this](const Twist::SharedPtr twist)
      {
        velocity_command_ptr_.writeFromNonRT(twist);
      }
    );

    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    realtime_odometry_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_activate(const rclcpp_lifecycle::State &)
  {
      
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_cleanup(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_error(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }
} // mecanum_drive_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerInterface)