# STM32WL55 Power Consumption Optimization

This repository contains the code and documentation related to the optimization of power consumption for the STM32WL55 Nucleo board. The project focuses on reducing energy usage during various operation modes, particularly for IoT and LoRaWAN applications.

## Project Overview

The goal of this project is to analyze and optimize the energy consumption of the STM32WL55 development board. By using advanced measurement tools and implementing firmware adjustments, this project aims to achieve significant reductions in power consumption.

### Key Features
- Measurement of power consumption in different operational modes.
- Firmware-level optimizations including:
  - Sleep and low-power modes.
  - Disabling of unnecessary debugging traces.
  - Deactivation of unused peripherals and LEDs.
- Power optimization during LoRaWAN transmission phases.

### Tools and Dependencies
- **Hardware**: STM32WL55 Nucleo board, Nordic Power Profiler Kit 2 (PPK2), optional GPS modules, and IKS01A1 sensors.
- **Software**:
  - STM32CubeIDE for firmware development.
  - STM32CubeProgrammer for memory operations.
  - Power Profiler tool for measuring and visualizing energy consumption.
- **Additional Tools**:
  - QOITECH OTII (optional for advanced profiling).

## Hardware Setup

Below is the full hardware setup used for power profiling and optimization:

![Full Hardware Setup](images/full_hardware_setup.jpg)

## Getting Started

### Prerequisites
1. Install STM32CubeIDE and Power Profiler on your PC.
2. Prepare the STM32WL55 Nucleo board and connect it to the PPK2 as described in the documentation.

### Running the Code
1. Flash the firmware onto the STM32WL55 using STM32CubeProgrammer.
2. Set up the PPK2 for energy profiling.
3. Execute the firmware and observe power consumption using the Power Profiler tool.

## License
This project is open-source and free to use, modify, and distribute.

---

Feel free to contact me for any questions or suggestions about this project.
