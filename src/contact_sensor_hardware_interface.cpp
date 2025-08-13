#include "contact_sensor_hardware_interface.hpp"
#include <cstring>
#include <iostream>
#include <pluginlib/class_list_macros.hpp>

namespace contact_sensor
{

ContactSensor::~ContactSensor()
{
  rx_running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  close_udp();
}

hardware_interface::CallbackReturn
ContactSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (info.hardware_parameters.count("udp_port")) {
    udp_port_ = static_cast<uint16_t>(std::stoi(info.hardware_parameters.at("udp_port")));
  }
  latest_contact_.store(false);
  contact_state_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ContactSensor::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_udp(udp_port_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  rx_running_ = true;
  rx_thread_ = std::thread(&ContactSensor::rx_thread_fn, this);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ContactSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  rx_running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  close_udp();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ContactSensor::on_cleanup(const rclcpp_lifecycle::State &)
{
  rx_running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  close_udp();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ContactSensor::export_state_interfaces()
{
  return { hardware_interface::StateInterface(
              "foot_contact_joint", hardware_interface::HW_IF_POSITION, &contact_state_) };
}

hardware_interface::return_type
ContactSensor::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  contact_state_ = latest_contact_.load(std::memory_order_relaxed) ? 1.0 : 0.0;
  return hardware_interface::return_type::OK;
}

bool ContactSensor::open_udp(uint16_t port)
{
  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) { perror("socket"); return false; }

  int opt = 1;
  if (setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    perror("setsockopt");
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    perror("bind");
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  int flags = fcntl(sock_fd_, F_GETFL, 0);
  fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

  RCLCPP_INFO(rclcpp::get_logger("pressure_sensor_hw"),
              "UDP listening on port %u", (unsigned)port);
  return true;
}

void ContactSensor::close_udp()
{
  if (sock_fd_ >= 0) { ::close(sock_fd_); sock_fd_ = -1; }
}

void ContactSensor::rx_thread_fn()
{
  constexpr int BUF_SZ = 64;
  unsigned char buf[BUF_SZ];          

  while (rx_running_) {
    fd_set rfds; FD_ZERO(&rfds); FD_SET(sock_fd_, &rfds);
    timeval tv{0, 100000};           

    int ready = select(sock_fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ready > 0 && FD_ISSET(sock_fd_, &rfds)) {
      sockaddr_in peer{}; socklen_t plen = sizeof(peer);
      const ssize_t n = recvfrom(sock_fd_, buf, BUF_SZ, 0,
                                 reinterpret_cast<sockaddr*>(&peer), &plen);
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          latest_contact_.store(buf[i] != 0, std::memory_order_relaxed);
        }
      }
    }
  }
}

} 

PLUGINLIB_EXPORT_CLASS(
  contact_sensor::ContactSensor,
  hardware_interface::SensorInterface)
