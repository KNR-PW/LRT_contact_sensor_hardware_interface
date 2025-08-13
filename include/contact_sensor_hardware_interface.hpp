#pragma once

#include <atomic>
#include <thread>
#include <string>
#include <vector>
#include <optional>
#include <mutex>
#include <chrono>

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "rclcpp/rclcpp.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

namespace pressure_sensor_hardware_interface
{
    class PressureSensor : public hardware_interface::SensorInterface
    {
        public:
        PressureSensor() = default;
        ~PressureSensor() override;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        private:
        bool open_udp(uint16_t port);
        void close_udp();
        void rx_thread_fn();

        int sock_fd_{-1};
        std::atomic<bool> rx_running_{false};
        std::thread rx_thread_;
        uint16_t udp_port_{5005};

        std::atomic<bool> latest_contact_{false};   
        double contact_state_{0.0};              

        std::atomic<bool> last_published_{false};
    };
}