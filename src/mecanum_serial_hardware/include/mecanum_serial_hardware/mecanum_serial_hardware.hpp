#ifndef MECANUM_SERIAL_HARDWARE__MECANUM_SERIAL_HARDWARE_HPP_
#define MECANUM_SERIAL_HARDWARE__MECANUM_SERIAL_HARDWARE_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mecanum_serial_hardware
{

class MecanumSerialHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumSerialHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool open_serial();
  void close_serial();
  bool send_line(const std::string & line);
  void process_receive_buffer();

  int serial_fd_{-1};
  std::string port_{"/dev/ttyUSB0"};
  int baud_rate_{115200};
  std::string receive_buffer_;
  bool run_announced_{false};
  unsigned int startup_stop_cycles_{0};

  // 순서: FL, FR, RL, RR
  std::array<double, 4> velocity_commands_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> position_states_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> velocity_states_{{0.0, 0.0, 0.0, 0.0}};
};

}  // namespace mecanum_serial_hardware

#endif
