#ifndef XAVBOT_HARDWARE_INTERFACE_HPP_
#define XAVBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motor2040_comms.hpp"

namespace xavbot_hardware_interface
{
class XavBotHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{

struct Config
{
  std::string fl_name = "";
  std::string fr_name = "";
  std::string rl_name = "";
  std::string rr_name = "";
  std::string* wheel_names[4] = {&fl_name, &fr_name, &rl_name, &rr_name};
  int fl_port = 0;
  int fr_port = 0;
  int rl_port = 0;
  int rr_port = 0;
  int* wheel_ports[4] = {&fl_port, &fr_port, &rl_port, &rr_port};
  std::string device = "";
  int timeout_ms = 0;
};

struct WheelStates
{
  std::array<double, 4> positions = {0, 0, 0, 0};
  std::array<double, 4> velocities = {0., 0., 0., 0.};
  std::array<double, 4> cmd = {0., 0., 0., 0.};
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(XavBotHardware)

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;
  
  hardware_interface::return_type stop() override;
  
  hardware_interface::return_type read() override;
  
  hardware_interface::return_type write() override;

private:
  // Create an instance of the Motor2040Comms class
  Motor2040Comms motor_controller_;

  // Create the config and wheelstate structs
  Config cfg_;
  WheelStates wheels_;
};

}  // xavbot_hardware_interface
#endif  // XAVBOT_HARDWARE_INTERFACE_HPP_
