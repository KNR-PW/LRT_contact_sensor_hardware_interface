# 🤖 Contact Sensor Hardware Interface

A custom **Hardware Interface** for `ros2_control` designed to integrate **ESP32-based contact sensors** into the ROS 2 ecosystem.

This driver is engineered for real-time performance, utilizing **UDP** with **non-blocking sockets** to ensure minimal latency in the control loop. It includes a custom handshake mechanism to verify hardware connectivity.

## 📦 Tech Stack

- **Framework:** ROS 2 Humble / `ros2_control`
- **Language:** C++ 17
- **Network:** Linux Sockets (UDP, Non-blocking)
- **Hardware:** ESP32 (Firmware)
- **Tools:** Python (Hardware Simulation), Xacro

## ⚡ Key Features

### 🚀 Non-blocking UDP Polling
Instead of relying on multi-threading overhead, this interface utilizes `O_NONBLOCK` sockets. The `read()` method drains the system buffer in every control cycle (100Hz+), ensuring the controller always acts on the most recent sensor state without locking the execution thread.

### 🤝 ACK Handshake (Ping-Pong)
To solve the "fire-and-forget" issue of UDP, the interface implements an **Application-Level ACK**. Upon receiving a packet, the driver immediately echoes the `sensor_id` back to the sender. This allows the ESP32 firmware to confirm that the ROS node is active and listening.

### ⚙️ Per-Sensor Configuration
Hardware mapping is handled directly in the **URDF/Xacro** files. Each `<sensor>` tag contains a specific `sensor_id` parameter, allowing flexible mapping of physical hardware IDs to logical ROS links (e.g., mapping ID `1` to `front_left_link`).

### 📡 ROS 2 Integration
Designed to work out-of-the-box with the standard `contact_sensors_broadcaster`. It automatically publishes `contact_msgs/Contact` messages to topics namespace-d by the robot's TF frames.

## 🚦 Installation & Usage

### 1. Build
Clone the repository into your workspace `src` directory and build.
*Note: We disable testing to speed up the build process.*

```bash
colcon build --symlink-install --packages-up-to contact_sensor_hardware_interface --cmake-args -DBUILD_TESTING=OFF
