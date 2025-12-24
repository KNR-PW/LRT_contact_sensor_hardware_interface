#include "contact_hardware_interface.hpp"
#include <sstream> 

namespace contact_sensor_hardware_interface

{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

ContactSensorHardwareInterface::~ContactSensorHardwareInterface()
{
  close_udp();
}

CallbackReturn ContactSensorHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto it_port = hardware_info.hardware_parameters.find("bind_port");
  if (it_port == hardware_info.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"), "Missing parameter: bind_port");
    return CallbackReturn::ERROR;
  }
  serverPortNum_ = std::stoi(it_port->second);

  id_to_idx_.clear();
  contacts_.assign(info_.sensors.size(), 0.0);

  for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
    const auto& sensor = info_.sensors[i];
    auto it_id = sensor.parameters.find("sensor_id");
    
    if (it_id == sensor.parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                   "Sensor '%s' missing parameter 'sensor_id'", sensor.name.c_str());
      return CallbackReturn::ERROR;
    }

    try {
      uint16_t parsed_id = static_cast<uint16_t>(std::stoi(it_id->second));
      id_to_idx_[parsed_id] = i;
      RCLCPP_INFO(rclcpp::get_logger("ContactSensorHardwareInterface"),
                  "Registered sensor '%s' with UDP ID %d at index %zu", 
                  sensor.name.c_str(), parsed_id, i);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                   "Invalid sensor_id for '%s': %s", sensor.name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ContactSensorHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  if (!open_udp(serverPortNum_)) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ContactSensorHardwareInterface"),
              "Activated. Listening on port %d (Non-blocking)", serverPortNum_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ContactSensorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
  close_udp();
  RCLCPP_INFO(rclcpp::get_logger("ContactSensorHardwareInterface"), "Deactivated.");
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> ContactSensorHardwareInterface::export_state_interfaces()
{
  std::vector<StateInterface> result;
  result.reserve(info_.sensors.size());
  for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
    result.emplace_back(info_.sensors[i].name, "contact", &contacts_[i]);
  }
  return result;
}

return_type ContactSensorHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
{

  struct Sample
  {
    uint32_t recv_calls = 0;   
    uint32_t datagrams = 0;    
    uint32_t drops = 0;        
    uint32_t ack_fail = 0;     
    uint32_t recv_err = 0;     
  };

  struct Window
  {
    using Steady = std::chrono::steady_clock;

    Steady::time_point t0 = Steady::now();
    uint64_t reads = 0;
    uint64_t recv_calls = 0;
    uint64_t datagrams = 0;
    uint64_t drops = 0;
    uint64_t ack_fail = 0;
    uint64_t recv_err = 0;
    uint64_t max_datagrams_in_read = 0;

    void add(const Sample& s)
    {
      reads += 1;
      recv_calls += s.recv_calls;
      datagrams += s.datagrams;
      drops += s.drops;
      ack_fail += s.ack_fail;
      recv_err += s.recv_err;
      if (s.datagrams > max_datagrams_in_read) {
        max_datagrams_in_read = s.datagrams;
      }
    }

    void maybe_log()
    {
      const auto now = Steady::now();
      const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - t0).count();

      if (elapsed < 1.0) {
        return;
      }

      const double read_hz = (elapsed > 0.0) ? static_cast<double>(reads) / elapsed : 0.0;
      const double datagrams_per_read = (reads > 0) ? static_cast<double>(datagrams) / static_cast<double>(reads) : 0.0;
      const double recv_calls_per_read = (reads > 0) ? static_cast<double>(recv_calls) / static_cast<double>(reads) : 0.0;

      RCLCPP_INFO(
        rclcpp::get_logger("ContactSensorHardwareInterface"),
        "HI stats(%.1fs): read_hz=%.1f datagrams/read=%.2f recv_calls/read=%.2f max_datagrams/read=%llu "
        "drops=%llu ack_fail=%llu recv_err=%llu",
        elapsed, read_hz, datagrams_per_read, recv_calls_per_read,
        static_cast<unsigned long long>(max_datagrams_in_read),
        static_cast<unsigned long long>(drops),
        static_cast<unsigned long long>(ack_fail),
        static_cast<unsigned long long>(recv_err));

      t0 = now;
      reads = 0;
      recv_calls = 0;
      datagrams = 0;
      drops = 0;
      ack_fail = 0;
      recv_err = 0;
      max_datagrams_in_read = 0;
    }
  };

  static Window w;
  Sample s;

  if (sock_fd_ < 0 && serverPortNum_ > 0) {
    if (!open_udp(serverPortNum_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ContactSensorHardwareInterface"),
        "Failed to (re)open UDP socket on port %d", serverPortNum_);
    } 
    else {
      RCLCPP_WARN(
        rclcpp::get_logger("ContactSensorHardwareInterface"),
        "UDP socket on port %d reopened successfully", serverPortNum_);
    }

    return return_type::OK;
  }

  UdpMsgV1 msg{};
  UdpAck ack_msg{};
  sockaddr_in peer{};
  socklen_t plen = sizeof(peer);

  while (true) {
    s.recv_calls++;

    ssize_t n = ::recvfrom(sock_fd_, &msg, sizeof(msg), 0,
                           reinterpret_cast<sockaddr*>(&peer), &plen);

    if (n < 0) {
      
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      s.recv_err++;

      RCLCPP_ERROR(
        rclcpp::get_logger("ContactSensorHardwareInterface"),
        "recvfrom() failed: %s. Closing UDP socket; will attempt reopen on next read().",
        std::strerror(errno));

      close_udp();
      break;
    }

    if (n != static_cast<ssize_t>(sizeof(msg))) {
      s.drops++;
      RCLCPP_WARN(
        rclcpp::get_logger("ContactSensorHardwareInterface"),
        "Received UDP packet of size %zd (expected %zu) – ignoring.",
        n, sizeof(msg));
      continue;
    }

    auto it = id_to_idx_.find(msg.sensor_id);
    if (it == id_to_idx_.end()) {
      s.drops++;
      RCLCPP_WARN(
        rclcpp::get_logger("ContactSensorHardwareInterface"),
        "Received message from unknown sensor_id=%u – ignoring.",
        static_cast<unsigned int>(msg.sensor_id));
      continue;
    }

    const std::size_t idx = it->second;

    contacts_[idx] = (msg.contact != 0) ? 1.0 : 0.0;

    ack_msg.sensor_id_ack = msg.sensor_id;
    if (::sendto(sock_fd_, &ack_msg, sizeof(ack_msg), 0,
                 reinterpret_cast<sockaddr*>(&peer), plen) < 0) {
      s.ack_fail++;
      RCLCPP_WARN(
        rclcpp::get_logger("ContactSensorHardwareInterface"),
        "sendto() ACK failed for sensor_id=%u: %s",
        static_cast<unsigned int>(msg.sensor_id),
        std::strerror(errno));
    }
    s.datagrams++;
  }

  w.add(s);
  w.maybe_log();

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

  int flags = ::fcntl(sock_fd_, F_GETFL, 0);
  if (flags == -1) flags = 0;
  if (::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "Failed to set non-blocking mode: %s", std::strerror(errno));
    close_udp();
    return false;
  }

  int opt = 1;
  if (::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    RCLCPP_WARN(rclcpp::get_logger("ContactSensorHardwareInterface"),
                "setsockopt(SO_REUSEADDR) failed");
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "bind() failed on port %d: %s", port, std::strerror(errno));
    close_udp();
    return false;
  }

  return true;
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
