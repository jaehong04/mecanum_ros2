#include "mecanum_serial_hardware/mecanum_serial_hardware.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecanum_serial_hardware
{

hardware_interface::CallbackReturn MecanumSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 4)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MecanumSerialHardware"),
      "Exactly four wheel joints are required.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto port_parameter = info_.hardware_parameters.find("port");
  if (port_parameter != info_.hardware_parameters.end())
  {
    port_ = port_parameter->second;
  }

  const auto baud_parameter = info_.hardware_parameters.find("baud_rate");
  if (baud_parameter != info_.hardware_parameters.end())
  {
    baud_rate_ = std::stoi(baud_parameter->second);
  }

  if (baud_rate_ != 115200)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MecanumSerialHardware"),
      "Only baud rate 115200 is currently supported.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("MecanumSerialHardware"),
        "Joint %s must have one velocity command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_velocity_state = false;

    for (const auto & state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity_state = true;
      }
    }

    if (!has_velocity_state)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("MecanumSerialHardware"),
        "Joint %s must have a velocity state interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSerialHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSerialHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  send_line("S\n");
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSerialHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  velocity_commands_.fill(0.0);
  velocity_states_.fill(0.0);
  receive_buffer_.clear();

  if (!send_line("S\n"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MecanumSerialHardware"),
    "Serial hardware activated on %s.",
    port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSerialHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  velocity_commands_.fill(0.0);
  send_line("S\n");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MecanumSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_states_[i]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MecanumSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_commands_[i]);
  }

  return interfaces;
}

hardware_interface::return_type MecanumSerialHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  char buffer[256];

  while (true)
  {
    const ssize_t count = ::read(serial_fd_, buffer, sizeof(buffer));

    if (count > 0)
    {
      receive_buffer_.append(buffer, static_cast<std::size_t>(count));
      continue;
    }

    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("MecanumSerialHardware"),
        "Serial read failed: %s",
        std::strerror(errno));
      return hardware_interface::return_type::ERROR;
    }

    break;
  }

  process_receive_buffer();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumSerialHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  std::array<double, 4> safe_commands{};

  for (std::size_t i = 0; i < safe_commands.size(); ++i)
  {
    double value = velocity_commands_[i];

    if (!std::isfinite(value))
    {
      value = 0.0;
    }

    if (value > 12.0)
    {
      value = 12.0;
    }
    else if (value < -12.0)
    {
      value = -12.0;
    }

    safe_commands[i] = value;
  }

  std::ostringstream message;
  message << std::fixed << std::setprecision(4)
          << "V,"
          << safe_commands[0] << ","
          << safe_commands[1] << ","
          << safe_commands[2] << ","
          << safe_commands[3] << "\n";

  const std::string serial_message = message.str();

  static unsigned int debug_write_count = 0;
  if (++debug_write_count % 50 == 0)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("MecanumSerialHardware"),
      "Serial TX: %s",
      serial_message.c_str());
  }

  if (!send_line(serial_message))
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool MecanumSerialHardware::open_serial()
{
  close_serial();

  serial_fd_ = ::open(
    port_.c_str(),
    O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MecanumSerialHardware"),
      "Cannot open %s: %s",
      port_.c_str(),
      std::strerror(errno));
    return false;
  }

  termios tty{};

  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MecanumSerialHardware"),
      "tcgetattr failed: %s",
      std::strerror(errno));
    close_serial();
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MecanumSerialHardware"),
      "tcsetattr failed: %s",
      std::strerror(errno));
    close_serial();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void MecanumSerialHardware::close_serial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool MecanumSerialHardware::send_line(const std::string & line)
{
  if (serial_fd_ < 0)
  {
    return false;
  }

  const char * data = line.data();
  std::size_t remaining = line.size();

  while (remaining > 0)
  {
    const ssize_t written = ::write(serial_fd_, data, remaining);

    if (written > 0)
    {
      data += written;
      remaining -= static_cast<std::size_t>(written);
      continue;
    }

    if (written < 0 && errno == EINTR)
    {
      continue;
    }

    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      return true;
    }

    RCLCPP_ERROR(
      rclcpp::get_logger("MecanumSerialHardware"),
      "Serial write failed: %s",
      std::strerror(errno));
    return false;
  }

  return true;
}

void MecanumSerialHardware::process_receive_buffer()
{
  std::size_t newline_position;

  while ((newline_position = receive_buffer_.find('\n')) != std::string::npos)
  {
    std::string line = receive_buffer_.substr(0, newline_position);
    receive_buffer_.erase(0, newline_position + 1);

    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }

    if (
      line == "OK,V" ||
      line == "OK,S" ||
      line == "PONG" ||
      line.rfind("READY,", 0) == 0 ||
      line.rfind("FORMAT,", 0) == 0 ||
      line.rfind("ERR,", 0) == 0)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("MecanumSerialHardware"),
        "Serial RX: %s",
        line.c_str());
    }

    double fl;
    double fr;
    double rl;
    double rr;

    if (std::sscanf(
        line.c_str(),
        "E,%lf,%lf,%lf,%lf",
        &fl, &fr, &rl, &rr) == 4)
    {
      velocity_states_[0] = fl;
      velocity_states_[1] = fr;
      velocity_states_[2] = rl;
      velocity_states_[3] = rr;
    }
  }

  if (receive_buffer_.size() > 1024)
  {
    receive_buffer_.clear();
  }
}

}  // namespace mecanum_serial_hardware

PLUGINLIB_EXPORT_CLASS(
  mecanum_serial_hardware::MecanumSerialHardware,
  hardware_interface::SystemInterface)
