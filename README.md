# contact_sensor_hardware_interface

`contact_sensor_hardware_interface` is a `ros2_control` **SensorInterface** that integrates multiple contact sensors over **UDP** (e.g., ESP32-based sensors).

The interface exports one state interface per configured sensor:

- `contact` (type `double`, values `0.0` / `1.0`)

## ROS 2 compatibility

- ROS 2 Humble

## Non-standard (external) components

This repository provides **only** the hardware interface. The controller/broadcaster and message package used for publishing contact data are maintained in separate repositories:

- `contact_sensors_broadcaster` (controller / broadcaster):  
  https://github.com/KNR-PW/LRT_contact_sensor_broadcaster

- `contact_msgs` (message package; provides `contact_msgs/Contact`):  
  https://github.com/BartlomiejK2/contact_msgs

## Tech stack

- C++17
- `ros2_control` / `hardware_interface::SensorInterface`
- POSIX / Linux non-blocking UDP sockets

## Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/KNR-PW/LRT_contact_sensor_hardware_interface.git

# Optional (only if you want to publish ROS messages)
git clone https://github.com/BartlomiejK2/contact_msgs.git
git clone https://github.com/KNR-PW/LRT_contact_sensor_broadcaster.git

cd ~/ros2_ws
rosdep install --ignore-src --from-paths src -y -r
colcon build --symlink-install --packages-up-to contact_sensor_hardware_interface
source install/setup.bash
```

## ros2_control configuration (URDF)

Below is a minimal example containing **only** the `ros2_control` block. Copy this into your robot URDF and adjust values.

```xml
<ros2_control name="ContactSensor" type="sensor">
  <hardware>
    <plugin>contact_sensor_hardware_interface::ContactSensorHardwareInterface</plugin>

    <!-- REQUIRED -->
    <param name="bind_port">5005</param>
  </hardware>

  <!-- One <sensor> entry per physical sensor -->
  <sensor name="front_left">
    <!-- REQUIRED -->
    <state_interface name="contact"/>

    <!-- REQUIRED -->
    <param name="sensor_id">1</param>
  </sensor>

  <sensor name="front_right">
    <state_interface name="contact"/>
    <param name="sensor_id">2</param>
  </sensor>

  <sensor name="rear_left">
    <state_interface name="contact"/>
    <param name="sensor_id">3</param>
  </sensor>

  <sensor name="rear_right">
    <state_interface name="contact"/>
    <param name="sensor_id">4</param>
  </sensor>
</ros2_control>
```

### Required parameters

#### Hardware parameters (inside `<hardware>`)

- `bind_port` (required, integer)  
  UDP port that the interface binds to (`INADDR_ANY`). Must match the destination port used by your sensor firmware.

#### Per-sensor parameters (inside each `<sensor>`)

- `sensor_id` (required, integer / uint16)  
  Hardware ID expected in incoming UDP packets. Must be unique across all configured sensors.

### Required state interfaces

Each `<sensor>` entry must declare:

- `contact` (required, `double`)  
  Semantics:
  - `0.0` = no contact
  - `1.0` = contact

The exported `ros2_control` state interface names follow the standard convention:
- `<sensor_name>/contact`

## UDP protocol contract (firmware)

The driver expects fixed-size datagrams matching the packed C structs:

### Incoming datagram: `UdpMsgV1`

- `uint16_t sensor_id`
- `uint8_t  contact`

### Outgoing ACK datagram: `UdpAck`

- `uint16_t sensor_id_ack` (echo of received `sensor_id`)

### Runtime behavior

- The UDP socket is configured as **non-blocking**.
- Each `read()` call drains the receive buffer until `recvfrom()` returns `EAGAIN` / `EWOULDBLOCK`.
- For each valid incoming datagram, the driver:
  - updates the `contact` state for the matching `sensor_id`
  - sends one ACK (`UdpAck`) back to the sender address/port from which the datagram arrived

### Endianness note

The current implementation uses the `sensor_id` field directly (no `ntohs` / `htons` conversion). Ensure your firmware sends `sensor_id` in the same byte order as the host running this driver (typical x86_64 Linux is little-endian), or update both sides to use network byte order.

## Controller / broadcaster configuration

This hardware interface exports only state interfaces. If you want a ROS topic publishing `contact_msgs/Contact`, use:

- `contact_sensors_broadcaster`: https://github.com/KNR-PW/LRT_contact_sensor_broadcaster
- `contact_msgs`: https://github.com/BartlomiejK2/contact_msgs

This repository includes an example controller configuration:

- `config/contact_sensor_controllers.yaml`

Example snippet:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    contact_sensors_broadcaster:
      type: contact_sensors_broadcaster/ContactSensorsBroadcaster

contact_sensors_broadcaster:
  ros__parameters:
    sensor_names:
      - front_left
      - front_right
      - rear_left
      - rear_right

    frame_ids:
      - front_left_link
      - front_right_link
      - rear_left_link
      - rear_right_link
```

Requirements:

- `sensor_names` must match the `<sensor name="...">` entries in the `ros2_control` block.
- `frame_ids` are broadcaster-specific metadata; see the broadcaster repository README for the exact meaning and published outputs.

## Local bringup (development)

This repository provides a launch file intended for quick local validation:

- `launch/contact_sensor_bringup.launch.py`

Example:

```bash
ros2 launch contact_sensor_hardware_interface contact_sensor_bringup.launch.py bind_port:=5005
```

Notes:

- The launch file is meant for development/testing and starts `ros2_control_node` and spawns:
  - `joint_state_broadcaster`
  - `contact_sensors_broadcaster`
- Refer to `config/contact_sensor_controllers.yaml` if you want to run the same setup manually.
