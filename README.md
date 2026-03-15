# JSBSim-HITL-PX4

A bridge between Pixhawk (running PX4 autopilot software) and JSBSim flight dynamics model for Hardware-In-The-Loop (HITL) simulation. This project enables realistic flight simulation by integrating JSBSim's physics engine with PX4's autopilot capabilities via MAVLink communication.

Hardware-In-The-Loop (HITL) simulation allows testing of actual flight hardware (like Pixhawk) in a simulated environment, providing a cost-effective way to validate autopilot behavior without risking real aircraft. The bridge acts as an intermediary, translating sensor data from JSBSim to MAVLink messages for Pixhawk and relaying actuator commands back to the simulator.

## Features

- Real-time flight simulation using JSBSim
- MAVLink interface for communication with PX4
- Support for various aircraft models (quadrotor, hexarotor, fixed-wing, etc.)
- Sensor plugins for IMU, GPS, barometer, magnetometer, and airspeed
- Actuator control for servos and motors
- ROS integration for additional tools and visualization

## Prerequisites

Before building and running this project, ensure you have the following dependencies installed:

- **JSBSim**: Flight dynamics simulation software
  - Install from source or package manager (e.g., `sudo apt install libjsbsim-dev` on Ubuntu)
  - Set `JSBSIM_ROOT_DIR` environment variable if installed in a custom location

- **MAVLink**: Communication protocol library
  - Install via package manager or from source

- **Boost**: C++ libraries (version 1.58 or later)
  - Components: system, thread, filesystem

- **TinyXML**: XML parsing library

- **Eigen3**: Linear algebra library

- **ROS (Robot Operating System)**: For ROS integration
  - Recommended: ROS Noetic or Melodic
  - Required packages: roscpp, rospy, std_msgs, mavros

- **CMake**: Build system (version 2.8.11 or later)

- **C++ Compiler**: Supporting C++14 standard

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/JSBSim-HITL-PX4.git
   cd JSBSim-HITL-PX4
   ```

2. Install dependencies:
   ```bash
   # Ubuntu/Debian example
   sudo apt update
   sudo apt install libjsbsim-dev libmavlink-dev libboost-all-dev libtinyxml-dev libeigen3-dev ros-noetic-roscpp ros-noetic-rospy ros-noetic-std-msgs ros-noetic-mavros
   ```

3. Set environment variables if needed:
   ```bash
   export JSBSIM_ROOT_DIR=/path/to/jsbsim/installation
   ```

## Build

### Option 1: Using ROS Catkin (Recommended for ROS users)

1. Source your ROS environment:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```

2. Create a catkin workspace if you don't have one:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ln -s /path/to/JSBSim-HITL-PX4 .
   cd ..
   ```

3. Build with catkin:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

### Option 2: Using CMake

1. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```

2. Configure and build:
   ```bash
   cmake ..
   make
   ```

## Usage

1. Configure your simulation:
   - Edit or create configuration files in the `configs/` directory
   - Select an aircraft model from the `models/` directory

2. Run the bridge:
   ```bash
   # Using ROS
   roslaunch jsbsim_bridge px4_jsbsim_bridge.launch

   # Or directly
   ./build/jsbsim_bridge [config_file]
   ```

3. Connect PX4 to the simulation via MAVLink (typically on UDP port 14560)

## Configuration

- Aircraft models are defined in the `models/` directory
- System configurations (sensors, actuators) are in the `systems/` directory
- Simulation scenarios are in the `scenario/` directory
- Launch files for ROS are in the `launch/` directory

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the BSD-3-Clause License. See the [LICENSE](LICENSE) file for details.

## Authors

- Jaeyoung Lim (jaeyoung@auterion.com)

## Acknowledgments

- Based on the original PX4-JSBSim bridge by Auterion AG
- Built upon JSBSim and PX4 open-source projects