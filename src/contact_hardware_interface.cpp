#include "contact_hardware_interface.hpp"

namespace contact_sensor_hardware_interface

{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

ContactSensorHardwareInterface::~ContactSensorHardwareInterface()
{
  if (rx_running_.exchange(false)) {
    close_udp();
    if (rx_thread_.joinable()) rx_thread_.join();
  } else {
    close_udp();
  }
  connected_.store(false);
}

CallbackReturn ContactSensorHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (hardware_info.sensors.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"), "No <sensor> entries in ros2_control block.");
    return CallbackReturn::ERROR;
  }
  if (hardware_info.sensors[0].state_interfaces.empty() ||
      hardware_info.sensors[0].state_interfaces[0].name != "contact") {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "Expected state_interface named 'contact' for the first sensor.");
    return CallbackReturn::ERROR;
  }

  auto it = hardware_info.hardware_parameters.find("bind_port");
  if (it == hardware_info.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"), "Missing parameter: bind_port");
    return CallbackReturn::ERROR;
  }

  try {
    serverPortNum_ = std::stoi(it->second);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "bind_port is not a valid integer: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (serverPortNum_ < 1 || serverPortNum_ > 65535) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "bind_port out of range: %d", serverPortNum_);
    return CallbackReturn::ERROR;
  }

  onContact_.store(0, std::memory_order_relaxed);
  contact_state_ = 0.0;
  state_seq_.store(0, std::memory_order_relaxed);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ContactSensorHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state; 
  if (connected_.load(std::memory_order_relaxed)) {
    RCLCPP_INFO(rclcpp::get_logger("ContactSensorHardwareInterface"), "Already connected.");
    return CallbackReturn::SUCCESS;
  }

  if (!open_udp(serverPortNum_)) {
    return CallbackReturn::ERROR;
  }

  bool expected = false;
  if (!rx_running_.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
    RCLCPP_INFO(rclcpp::get_logger("ContactSensorHardwareInterface"), "RX thread already running.");
    return CallbackReturn::SUCCESS;
  }

  rx_thread_ = std::thread(&ContactSensorHardwareInterface::rx_thread_fn, this);

  const uint64_t start_seq = state_seq_.load(std::memory_order_relaxed);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  bool got_first = false;

  while (std::chrono::steady_clock::now() < deadline) {
    if (state_seq_.load(std::memory_order_relaxed) > start_seq) {
      got_first = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!got_first) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "Failed to receive first sensor packet (timeout).");
    rx_running_.store(false, std::memory_order_relaxed);
    close_udp();
    if (rx_thread_.joinable()) rx_thread_.join();
    return CallbackReturn::ERROR;
  }

  connected_.store(true, std::memory_order_relaxed);
  RCLCPP_INFO(rclcpp::get_logger("ContactSensorHardwareInterface"),
              "Contact sensor activated (UDP %d).", serverPortNum_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ContactSensorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state; 
  const bool was_running = rx_running_.exchange(false, std::memory_order_relaxed);
  close_udp();
  if (was_running && rx_thread_.joinable()) {
    rx_thread_.join();
  }
  connected_.store(false, std::memory_order_relaxed);
  RCLCPP_INFO(rclcpp::get_logger("ContactSensorHardwareInterface"), "Contact sensor deactivated.");
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> ContactSensorHardwareInterface::export_state_interfaces()
{
  if (info_.sensors.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "No sensors in hardware info.");
    return {};
  }

  const std::string& sensor_name = info_.sensors[0].name;
  StateInterface state_if(sensor_name, "contact", &contact_state_);
  return { state_if };
}

return_type ContactSensorHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  (void)time;
  (void)period;
  contact_state_ = onContact_.load(std::memory_order_relaxed) ? 1.0 : 0.0;
  return return_type::OK;
}

bool ContactSensorHardwareInterface::open_udp(int port)
{
  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);                                          
  if (sock_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "socket() failed: %s", std::strerror(errno));
    return false;
  }

  int opt = 1;
  if (::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    RCLCPP_WARN(rclcpp::get_logger("ContactSensorHardwareInterface"),
                "setsockopt(SO_REUSEADDR) failed: %s", std::strerror(errno));
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_ANY); 

  if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "bind() failed on port %d: %s", port, std::strerror(errno));
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("ContactSensorHardwareInterface"),
              "UDP listening on 0.0.0.0:%u", static_cast<unsigned>(port));
  return true;
}

void ContactSensorHardwareInterface::rx_thread_fn()
{
  while (rx_running_.load(std::memory_order_relaxed)) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock_fd_, &rfds);
    timeval tv{0, 100000};

    int ready = ::select(sock_fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ready < 0) {
      if (errno == EINTR) continue; 
      break;
    }
    if (ready == 0) {
      continue;
    }

    if (FD_ISSET(sock_fd_, &rfds)) {
      uint8_t byte{};
      sockaddr_in peer{};
      socklen_t plen = sizeof(peer);
      const ssize_t n = ::recvfrom(sock_fd_, &byte, 1, 0,
                                   reinterpret_cast<sockaddr*>(&peer), &plen);
      if (n == 1) {
        onContact_.store(byte ? 1 : 0, std::memory_order_relaxed);
        state_seq_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

void ContactSensorHardwareInterface::close_udp()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

} 

PLUGINLIB_EXPORT_CLASS(contact_sensor_hardware_interface::ContactSensorHardwareInterface, hardware_interface::SensorInterface)
