#include "xavbot_hardware_interface.hpp"
#include "motor2040_comms.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace xavbot_hardware_interface
{
hardware_interface::return_type XavBotHardware::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) 
  {
    return hardware_interface::return_type::ERROR;
  }

  cfg_.fl_name = info_.hardware_parameters["fl_name"];
  cfg_.fr_name = info_.hardware_parameters["fr_name"];
  cfg_.rl_name = info_.hardware_parameters["rl_name"];
  cfg_.rr_name = info_.hardware_parameters["rr_name"];
  cfg_.fl_port = std::stoi(info_.hardware_parameters["fl_port"]);
  cfg_.fr_port = std::stoi(info_.hardware_parameters["fr_port"]);
  cfg_.rl_port = std::stoi(info_.hardware_parameters["rl_port"]);
  cfg_.rr_port = std::stoi(info_.hardware_parameters["rr_port"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Expects exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XavBotHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XavBotHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XavBotHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XavBotHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XavBotHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> XavBotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      *cfg_.wheel_names[i], hardware_interface::HW_IF_POSITION, &wheels_.positions[*cfg_.wheel_ports[i]]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      *cfg_.wheel_names[i], hardware_interface::HW_IF_VELOCITY, &wheels_.velocities[*cfg_.wheel_ports[i]]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> XavBotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      *cfg_.wheel_names[i], hardware_interface::HW_IF_VELOCITY, &wheels_.cmd[*cfg_.wheel_ports[i]]));
  }
  return command_interfaces;
}

hardware_interface::return_type XavBotHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("XavBotHardware"), "Activating...");
  motor_controller_.connect(cfg_.device, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("XavBotHardware"), "OKfully activated!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XavBotHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("XavBotHardware"), "Deactivating...");
  motor_controller_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("XavBotHardware"), "OKfully deactivated!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XavBotHardware::read()
{
  wheels_.positions = motor_controller_.get_positions();
  wheels_.velocities = motor_controller_.get_velocities();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XavBotHardware::write()
{
  motor_controller_.set_velocities(wheels_.cmd);
  return hardware_interface::return_type::OK;
}

}  // namespace xavbot_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  xavbot_hardware_interface::XavBotHardware, hardware_interface::SystemInterface)