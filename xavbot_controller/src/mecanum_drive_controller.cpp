#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mecanum_drive_controller.hpp"

namespace mecanum_drive_controller
{

  MecanumDriveController::MecanumDriveController() : controller_interface::ControllerInterface()
      , velocity_command_subsciption_(nullptr)
      , velocity_command_ptr_(nullptr)
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
    // const auto twist = (*velocity_command)->twist;
    const auto twist = *velocity_command->get();
    double half_width = wheelbase_width_ / 2;
    double half_length = wheelbase_length_ / 2;
    double fl_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y - (half_width + half_length) * twist.angular.z);
    double fr_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y + (half_width + half_length) * twist.angular.z);
    double rl_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y - (half_width + half_length) * twist.angular.z);
    double rr_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y + (half_width + half_length) * twist.angular.z);

    // Set the wheel velocities
    // TODO: Don't hardcode command_interface_ indexes 
    command_interfaces_[0].set_value(fl_wheel_velocity);
    command_interfaces_[1].set_value(fr_wheel_velocity);
    command_interfaces_[2].set_value(rl_wheel_velocity);
    command_interfaces_[3].set_value(rr_wheel_velocity);
    
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

    velocity_command_subsciption_ = get_node()->create_subscription<Twist>("/cmd_vel", 10, [this](const Twist::SharedPtr twist)
      {
        velocity_command_ptr_.writeFromNonRT(twist);
      }
    );
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