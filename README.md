# tas_radar
## _TAS Radar Sensor Driver for ROS (ROS1 and ROS2)_

Collection of ROS-based drivers, tools, preprocessing algorithms and visualization nodes for radar sensors used at our institute [TAS](https://www.unibw.de/tas).

https://user-images.githubusercontent.com/2410398/183098526-23277037-e311-43d7-91c9-82d7f9a25e5f.mp4

A crossing scenario in RViz with ego-compensated Doppler visualization (dynamic detections surrounded by color-coded bubbles).

This repository contains only sensor drivers for [smartmicro (SMS)](https://www.smartmicro.com/automotive-radar) so far:
- supporting UMRR and DRVEGRD
- automatic switching to legacy parsers of outdated protocol versions; simultaneous operation of radars with different firmwares

This collection offers the following features:
- Driver for CAN and BroadR-Reach network
- Real-time sensor configurability (Data integration)
- Graphical configuration tool for multiple sensors in the network
- Dynamic query of sensors in the sensor's default IP range 192.168.11.x - no manual configuration needed
- Driver-side publication of available sensors and their detected radar type
- Message definitions for generic detection-based radar sensors
- Visualization nodes, which also visualize the radial-speed measurement (opt. ego-motion compensated with buffered time synchronization)
- Support for the DRVEGRD alive (discovery) service

This collection has been developed since 2020, tested in multiple hardware setups and used in closed-loop applications. It is based on C++, optimized for low CPU usage and direct measurement data access.

The current smartmicro driver/configurator stack supports mixed operation of different smartmicro radar generations in one network:
- legacy UMRR devices
- newer DRVEGRD devices

The repository currently contains the following main packages:
- `driver/smartmicro/smartmicro_driver`: smartmicro driver for CAN and Ethernet
- `driver/smartmicro/smartmicro_configurator`: Qt-based smartmicro configuration GUI
- `messages/radar_msgs`: generic radar detection messages
- `visualization/radar_marker`: RViz marker visualization
- `visualization/radar_pointcloud`: accumulated point cloud visualization
- `launch/radar_launch`: example launch files
- `documentation`: archived protocol documentation used during development and migration

## Dependencies
- [librosqt](https://github.com/1r0b1n0/librosqt)
- [ethernet_bridge](https://github.com/UniBwTAS/ethernet_bridge) (for BroadR-Reach- or network-based drivers)

Depending on your workspace setup, you may also need the usual ROS Noetic build dependencies for Qt, message generation and visualization.

## Installation
- just install dependencies and compile
- see launch file for example startup

All packages build for both **ROS 1** (catkin) and **ROS 2** (ament), selected at build time via `$ROS_VERSION` from a single source tree; the ROS 1 nodes keep the same node and executable names. The Qt/librosqt configuration GUI (`smartmicro_configurator`) remains ROS 1-only.

The repository contains a simple demo launch file for a live smartmicro Ethernet setup:

```bash
# ROS 1
roslaunch radar_launch smartmicro_ethernet.launch

# ROS 2 (without the ROS 1-only configurator)
ros2 launch radar_launch smartmicro_ethernet.launch.xml
```

The demo launch starts:
- the measurement Ethernet bridge on UDP port `55555`
- the DRVEGRD alive bridge on UDP multicast `239.144.0.0:60000`
- the smartmicro driver
- the smartmicro configurator
- marker visualization
- point cloud visualization

Important ROS topics in the demo setup are:
- `/sensor/radar/smartmicro/detections`
- `/sensor/radar/smartmicro/sensors`
- `/sensor/radar/smartmicro/instructions/request`
- `/sensor/radar/smartmicro/instructions/response`

The `sensors` topic provides the currently available smartmicro sensors together with their detected radar type and is used by the configurator for sensor discovery and GUI switching between UMRR and DRVEGRD controls.

Notes:
- The repository currently documents and implements only smartmicro-related drivers.
- Some newer smartmicro firmware/protocol details were integrated pragmatically from real sensor behavior in addition to vendor documentation.
- The `documentation` folder contains old and new protocol references that were used during the migration to newer smartmicro devices.

## Screenshots
Configuration tool:

![sms_configuration_tool](https://user-images.githubusercontent.com/2410398/183097935-61f7c74f-e8b6-4fde-8a3f-336cf9ed472f.png)

Detection accumulation with egomotion-compensated radial speed channel:

![detection_accumulation](https://github.com/UniBwTAS/tas_radar/assets/2410398/dead3581-2660-4b38-85a1-0f3d9de667c8)
