#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <chrono>

namespace contact_sensor_hardware_interface
{

class ContactSensorHardwareInterface final : public hardware_interface::SensorInterface
{
public:
  ContactSensorHardwareInterface() = default;
  ~ContactSensorHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;              
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  bool open_udp(int port);
  void close_udp();
  void rx_thread_fn();

  double contact_state_{0.0};                    
  std::atomic<uint8_t> onContact_{0};            

  std::atomic<bool> connected_{false};
  std::atomic<bool> rx_running_{false};
  std::thread rx_thread_;
  std::atomic<uint64_t> state_seq_{0};           

  int sock_fd_{-1};

  int serverPortNum_{0};
};

} 
