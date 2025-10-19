#include "contact_hardware_interface.hpp"
#include <sstream> 

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

  auto it_ids = hardware_info.hardware_parameters.find("sensor_ids");
  if (it_ids == hardware_info.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "Missing parameter: sensor_ids (comma-separated, e.g. \"1,2,3,4\")");
    return CallbackReturn::ERROR;
  }

  std::vector<uint16_t> ids;
  {
    std::stringstream ss(it_ids->second);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      tok.erase(0, tok.find_first_not_of(" \t"));
      tok.erase(tok.find_last_not_of(" \t") + 1);
      if (tok.empty()) continue;
      uint16_t id = static_cast<uint16_t>(std::stoi(tok));
      ids.push_back(id);
    }  
  }

  if (ids.size() != hardware_info.sensors.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "sensor_ids count (%zu) != number of <sensor> entries (%zu).",
                 ids.size(), hardware_info.sensors.size());
    return CallbackReturn::ERROR;
  }

  contacts_.assign(ids.size(), 0.0);
  on_contacts_.resize(ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i) {
    on_contacts_[i].store(0, std::memory_order_relaxed);
  }

  id_to_idx_.clear();
  for (std::size_t i = 0; i < ids.size(); ++i) {
    id_to_idx_[ids[i]] = i;
  }

  state_seq_.store(0, std::memory_order_relaxed);
  connected_.store(false, std::memory_order_relaxed);

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
  std::vector<StateInterface> result;
  result.reserve(info_.sensors.size());
  for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
    result.emplace_back(info_.sensors[i].name, "contact", &contacts_[i]);
  }
  return result;
}

return_type ContactSensorHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  for (std::size_t i = 0; i < on_contacts_.size(); ++i) {
    contacts_[i] = on_contacts_[i].load(std::memory_order_relaxed) ? 1.0 : 0.0;
  }

  return return_type::OK;
}

bool ContactSensorHardwareInterface::open_udp(int port)
{
  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);                                          // Wywołuje globalną funkcje POSIX z parametrami (IPV4, SOCKDRAM (TYP GNIAZDA DATAGRAMOWEGO UDP), 0 CZYLI DOMYŚLNY)                                                                    
  if (sock_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ContactSensorHardwareInterface"),
                 "socket() failed: %s", std::strerror(errno));
    return false;
  }

  int opt = 1;
  if (::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {         // SO_REUSEADDR - szybkie ponowne użycie adresu/portu, współdzielenie wielu gniazd
    RCLCPP_WARN(rclcpp::get_logger("ContactSensorHardwareInterface"),
                "setsockopt(SO_REUSEADDR) failed: %s", std::strerror(errno));
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;                                                            // Rodzina adresów IPV4
  addr.sin_port   = htons(static_cast<uint16_t>(port));                                 // ODBIÓR DANYCH, HOST TO NETWORK SHORT (16 BITÓW), CZYLI OD BAJTA NAJBARDZIEJ ZNACZACEGO 
  addr.sin_addr.s_addr = htonl(INADDR_ANY);                                             // ODBIÓR  DANYCH (32 BITY), DOWOLNY ADRES INADDR_ANY

  if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {         // BIND -> MÓWI KERNELOWI, UZYWAJ TEGO ADRESU I PORTU W GNIEŹDZIE 
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
    fd_set rfds;                                                                        // BITOWA MASKA DESKRYPTORÓW PLIKÓW (TU GNIAZD), KTÓRE CHCESZ OBSERWOWAĆ POD KĄTEM CZYTANIA
    FD_ZERO(&rfds);                                                                     // ZERUJE MASKE
    FD_SET(sock_fd_, &rfds);                                                            // DODANIE SOCKETU UDP DO CZYTANIA
    timeval tv{0, 100000};                                                              // USTAWIA TIMEOUT DLA SELECT()

    // select() „przycina” przekazane maski i wstawia w nich tylko te deskryptory, które są gotowe.
    int ready = ::select(sock_fd_ + 1, &rfds, nullptr, nullptr, &tv);                   // DESKRYPTOR (+ 1 BO CZYTAMY OD 0 DO -1), MASKA, NULL BO NIE OBSERWUJEMY DESKRYPTORÓW DO PISANIA + TIMEOUT
    if (ready < 0) {
      if (errno == EINTR) continue; 
      break;
    }
    if (ready == 0) {
      continue;
    }

    if (FD_ISSET(sock_fd_, &rfds)) {
      UdpMsgV1 msg{};
      sockaddr_in peer{};
      socklen_t plen = sizeof(peer);

      const ssize_t n = ::recvfrom(
          sock_fd_, &msg, sizeof(msg), 0,
          reinterpret_cast<sockaddr*>(&peer), &plen);

      if (n == static_cast<ssize_t>(sizeof(msg))) {
        auto it = id_to_idx_.find(msg.sensor_id);
        if (it != id_to_idx_.end()) {
          on_contacts_[it->second].store(msg.contact ? 1 : 0, std::memory_order_relaxed);
          state_seq_.fetch_add(1, std::memory_order_relaxed);
        }
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
