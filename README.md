# tas_radar
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

## Dependencies
- [librosqt](https://github.com/1r0b1n0/librosqt)
- [ethernet_bridge](https://github.com/UniBwTAS/ethernet_bridge) (for BroadR-Reach- or network-based drivers)

## Installation
- just install dependencies and compile
- see launch file for example startup

## Screenshots
Configuration tool:
![sms_configuration_tool](https://user-images.githubusercontent.com/2410398/183097935-61f7c74f-e8b6-4fde-8a3f-336cf9ed472f.png)

Detection accumulation with egomotion-compensated radial speed channel:
![detection_accumulation](https://github.com/UniBwTAS/tas_radar/assets/2410398/dead3581-2660-4b38-85a1-0f3d9de667c8)
