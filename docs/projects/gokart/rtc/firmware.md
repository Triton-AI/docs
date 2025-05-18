# Gokart Controller Documentation

## Overview

The Triton AI Gokart Controller is an embedded software project built on Mbed OS for controlling a gokart. It supports multiple control modes and integrates various subsystems including communication, sensor reading, state management, actuation via CAN bus, performance profiling, and a watchdog system.

> **Note:** This project is designed to run on hardware such as the Nucleo-H723ZG and relies on Mbed OS APIs.

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Hardware Setup](#hardware-setup)
- [Modules and Components](#modules-and-components)
  - [Main Application](#main-application)
  - [State Machine](#state-machine)
  - [Communication Manager](#communication-manager)
  - [Controller](#controller)
  - [Remote Control (RC) Controller](#remote-control-rc-controller)
  - [Sensor Reader](#sensor-reader)
  - [Actuation Controller & VESC CAN Tools](#actuation-controller--vesc-can-tools)
  - [Logger and Profilers](#logger-and-profilers)
  - [Watchdog & Watchable](#watchdog--watchable)
- [Development Environment](#development-environment)
- [Building and Running](#building-and-running)
- [Testing](#testing)
- [Future Work](#future-work)

## Project Structure

Below is an overview of the key folders and files:
```
src
├── Actuation
│   ├── actuation_controller.cpp
│   ├── actuation_controller.hpp
│   ├── README.md
│   └── vesc_can_tools.hpp
├── Comm
│   ├── comm.cpp
│   └── comm.hpp
├── Controller
│   ├── controller.cpp
│   └── controller.hpp
├── main.cpp
├── RCController
│   ├── rc_controller.cpp
│   └── rc_controller.hpp
├── Sensor
│   ├── brake_pressure_sensor.cpp
│   ├── brake_pressure_sensor.hpp
│   ├── sensor_reader.cpp
│   ├── sensor_reader.hpp
│   ├── wheel_speed_sensor.cpp
│   └── wheel_speed_sensor.hpp
├── StateMachine
│   ├── state_machine.cpp
│   └── state_machine.hpp
├── Tools
│   ├── global_profilers.hpp
│   ├── logger.hpp
│   └── profiler.hpp
├── USBJoystick
│   ├── usb_joystick.cpp
│   └── usb_joystick.hpp
└── Watchdog
    ├── README.md
    ├── watchable.hpp
    ├── watchdog.cpp
    └── watchdog.hpp
```
Each folder contains code that encapsulates one or more aspects of the system's functionality.

## Hardware Setup

### Board Fuses
The breakout board includes protection fuses that may blow during development or if there's a power issue. If a fuse blows, you can bridge it with solder to restore functionality. There are 3 fuses on the breakout board, located next to the 12v, 5v, and 3.3v inputs. Documentation and schematics for the breakout board are located in this repository.

### USB Connection
The controller requires a USB connection to the on-board computer. **Important**: The system waits for this connection before fully initializing. Ensure the USB cable is properly connected between the user USB device port on the board and the computer.

### Serial Connection
For debugging and direct control, connect the Serial-to-USB adapter to the machine you wish to use for testing, not directly to the Nucleo board. This allows communication via `/dev/ttyUSB0` (linux), `/dev/cu.usbmodemXXXX` (MacOS), or similar ports.

## Modules and Components

### Main Application

The entry point is in **main.cpp**. It:
- Sets up the passthrough button (an interrupt on the user button for toggling controller passthrough mode).
- Instantiates the `Controller` object.
- Runs an infinite loop with sleep intervals to keep the thread alive.

> **Example snippet (main.cpp):**
> ```cpp
> #include "PinNames.h"
> #include "mbed.h"
> #include "Controller/controller.hpp"
> 
> bool g_PassthroughEnabled = false;
> 
> // Button callback for toggling passthrough mode
> void TogglePassthrough() {
>     g_PassthroughEnabled = !g_PassthroughEnabled;
> }
> 
> InterruptIn button(BUTTON1);
> 
> int main() {
>     // Use user button to toggle passthrough
>     button.rise(&TogglePassthrough);
>     
>     new tritonai::gkc::Controller();
>     
>     while (true) {
>         ThisThread::sleep_for(3600000ms); // 1 hour
>     };
> }
> ```

### State Machine

The state machine (see **state_machine.hpp** and **state_machine.cpp**) defines the lifecycle of the controller. It manages transitions among states such as:

- **Uninitialized** – the default state.
- **Initializing** – when the system is starting up.
- **Inactive** – when the system is idle.
- **Active** – when the gokart is under normal operation.
- **Emergency** – when an error or safety-critical event occurs.

The abstract base class requires child classes (e.g., the Controller class) to implement state-specific callbacks such as `OnInitialize`, `OnActivate`, and `OnEmergencyStop`.

### Communication Manager

The **CommManager** (declared in **comm.hpp** and implemented in **comm.cpp**) handles packet-based communication. It supports:

- Sending and receiving packets over Serial.
- Converting high-level packets into byte buffers.
- Calling packet callbacks on the Controller.

It uses a send queue and dedicated threads to manage communication asynchronously.

### Controller

The **Controller** class (in **Controller/controller.hpp** and **controller.cpp**) is the central unit. It inherits from several interfaces:

- **GkcPacketSubscriber** – to receive packets.
- **Watchable** – so that it can be monitored by the watchdog.
- **ILogger** – for logging events.
- **GkcStateMachine** – to manage its state.

It integrates the CommManager, SensorReader, ActuationController, RCController, and a local watchdog. The Controller's heartbeat function periodically sends a heartbeat packet and toggles an LED, while also logging state transitions.

> **Key responsibilities include:**
>
> - Handling various packet types (e.g., handshake, control, sensor, and RC packets).
> - Managing state transitions (e.g., from *Inactive* to *Active*).
> - Coordinating actuation commands.
> - Responding to safety events (e.g., lost RC heartbeat triggers an emergency stop).

### Remote Control (RC) Controller

The **RCController** (in **RCController/rc_controller.hpp** and **rc_controller.cpp**) processes remote control inputs. It:
- Reads analog signals from an ELRS receiver.
- Normalizes and maps the signals (using helper functions in a nested `Translation` struct) to throttle, brake, and steering commands.
- Publishes RC packets for consumption by the Controller.
- Monitors heartbeat from the RC to detect disconnections.
- Provides USB joystick passthrough functionality when enabled.

### Sensor Reader

The **SensorReader** (in **Sensor/sensor_reader.hpp** and **sensor_reader.cpp**) is responsible for polling various sensor providers. It:
- Maintains a list of sensor provider objects that implement the `ISensorProvider` interface.
- Periodically reads sensor data and populates a `SensorGkcPacket`.
- Uses a mutex to safely add or remove sensor providers.

### Actuation Controller & VESC CAN Tools

The **ActuationController** (in **Actuation/actuation_controller.hpp** and **actuation_controller.cpp**) commands the vehicle's actuators. It sends commands to the motor controllers and brakes through CAN bus messages.

The **VESC CAN Tools** (in **vesc_can_tools.hpp**) provide a set of helper functions to:
- Format and send CAN messages.
- Convert actuation commands (throttle, brake, steering) into the proper CAN message formats.
- Map steering angles using lookup tables defined in the configuration.

### Logger and Profilers

- **Logger:** The `ILogger` interface (see **Tools/logger.hpp**) provides a common API for logging messages with different severity levels. The Controller implements this interface and prints log messages to `stdout` or `stderr` depending on the severity.
  
- **Profilers:** The profiler classes (in **Tools/profiler.hpp** and **Tools/global_profilers.hpp**) offer functionality to measure code performance. They track elapsed and average times for specific sections (e.g., Control, Sensor, Comm) and can dump profiling information as strings.

### Watchdog & Watchable

To ensure system reliability, the project includes a watchdog system:
- **Watchable:** (in **Watchdog/watchable.hpp**) is an interface for components that should be monitored. It provides methods to increment an activity counter.
- **Watchdog:** (in **Watchdog/watchdog.hpp** and **watchdog.cpp**) monitors a list of `Watchable` objects and triggers a system reset if one of them becomes inactive for too long.

This mechanism is used by critical modules such as the Controller, CommManager, SensorReader, and RCController.

## Development Environment

### Setting Up PlatformIO for Visual Studio Code

1. **Install VS Code and PlatformIO:**
   - Download and install [VS Code](https://code.visualstudio.com/)
   - Open VS Code and install the PlatformIO extension from the marketplace

2. **Open the Project:**
   - In VS Code, go to PlatformIO Home
   - Select "Open Project"
   - Navigate to the gokart controller project folder

3. **Configure PlatformIO:**
   - The project includes a `platformio.ini` file that configures build settings
   - Make sure the correct board is selected (e.g., `nucleo_h723zg`)

4. **Build and Upload:**
   - Use the PlatformIO toolbar to build and upload the project to the Nucleo board

### Platform Compatibility

This project has primarily been tested on Mac and Linux systems. Windows support is experimental:

- **Windows:** Not thoroughly tested. Users have used WSL (Windows Subsystem for Linux) with USB passthrough for development.
- **Mac OS** Usable development platform. USB devices will often have different names on each use.
- **Linux:** Recommended development platform. Tested on Ubuntu and similar distributions.

### Configuration Files

#### platformio.ini
This file controls build instructions and runtime configurations:
```ini
[env:nucleo_h723zg]
platform = ststm32
board = nucleo_h723zg
framework = mbed
build_flags = -DUSBDEVICE
monitor_speed = 115200
```

### mbed_app.json
This file adds extra features to Mbed OS:
```json
{
  "target_overrides": {
    "*": {
      "platform.stdio-baud-rate": 115200,
      "target.usb_device": true,
      "target.printf_lib": "minimal-printf",
      "platform.minimal-printf-enable-floating-point": true,
      "platform.minimal-printf-set-floating-point-max-decimals": 2
    }
  }
}
```

## Building and Running

1. **Requirements:**
   - Mbed OS
   - Nucleo-H723ZG (or compatible) board
   - Appropriate CAN transceivers and wiring for actuators and sensors
   - Configuration parameters set in **config.hpp**

2. **Building:**
   - Use PlatformIO in VS Code as described above
   - Adjust configuration parameters in **config.hpp** as necessary
   - The `platformio.ini` file configures the build process

3. **Deployment:**
   - Flash the binary onto the target board
   - Connect the USB device port to the onboard computer
   - The attached LED will indicate the current state

## Testing

### Serial Control Testing

The project includes a Python script (`serial_test.py`) for testing serial control:

1. Connect the Serial-to-USB adapter to the breakout board's Serial port (not directly to the Nucleo)
2. Run the script with the appropriate port:
   
   python serial_test.py --port /dev/ttyUSB0 --speed 5.0
   
3. The script provides various command-line options:
   - `--port`: Serial port (default: `/dev/ttyUSB0`)
   - `--speed`: Target speed in m/s (default: `5.0`)
   - `--duration`: Test duration in seconds (default: `0` for indefinite)
   - `--debug`: Enable debug logging

## Future Work

The following improvements are recommended for future development:

### Critical Issues
- **Safety Issue:** Very rarely, when the controller dies, the system doesn't stop. This is a critical safety issue that needs to be addressed.
- **Controller Reliability:** Fix controller noise and disconnection issues that occur occasionally.
- **USB Passthrough:** Fix USB passthrough breaking when the onboard computer isn't actively awaiting input.

### Vehicle Performance
- **PID Steering Control:** Implement PID control for steering to improve accuracy and response.
- **Brake Improvements:** Develop a pressure sensor feedback loop to set brake pressure more accurately.
- **ABS Implementation:** Integrate an accelerometer to implement anti-lock braking capabilities.

### Code Structure
- **Smart Pointers:** Replace raw pointers with smart pointers for better memory management.
- **Application Structure:** Refactor the system to follow a cleaner system lifecycle pattern:
  ```cpp
  system->start()
  system->run()
  system->stop()
  ```
- **Thread-safe Logging:** Make the logging system thread-safe to prevent race conditions.
- **Profiler Enhancement:** Implement or fix the profiling system, which is currently untested.

By addressing these issues, the Gokart Controller will become more reliable, safer, and easier to maintain for future users.