# diff_robot_canopen

This repository provides a comprehensive example of a differential drive robot utilizing the `ros2_canopen` package. It is specifically designed for operation with physical hardware, not for simulation.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)
- [Maintainer](#maintainer)

## Prerequisites

Before you begin, ensure you have met the following requirements:
- You have installed ROS 2.
- You have a CAN controller set up.
- You have installed the `ros2_canopen` package by following [this guide](https://ros-industrial.github.io/ros2_canopen/manual/rolling/quickstart/installation.html). Make sure to switch to the appropriate ROS distribution branch in the repository. For example, if you are using the `humble` distribution, use the following command:

```bash
git clone -b humble https://github.com/ros-industrial/ros2_canopen.git
```

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/mustafa-essam/diff_robot_canopen.git
   ```
2. **Build the package:**
   ```bash
   cd /path/to/your/ros2_ws
   colcon build
   ```

## Configuration

1. **CAN Controller Setup:**
   Follow [this guide](https://ros-industrial.github.io/ros2_canopen/manual/rolling/quickstart/operation.html) to set up your CAN controller:
   ```bash
   sudo modprobe gs_usb
   sudo ip link set can0 up type can bitrate 500000
   sudo ip link set can0 txqueuelen 1000
   sudo ip link set up can0
   ```

2. **Robot Configuration:**
   The robot's configuration files are located in the `config` directory. You can modify the `bus.yml` and `ros2_controllers.yaml` files to match your robot's specific hardware.

## Usage

1. **Launch the Robot Control:**
   ```bash
   ros2 launch diff_robot_canopen robot_control.launch.py
   ```

2. **Start Communication with Drivers:**
   Start communication with the drivers by calling the following services:
   ```bash
   ros2 service call /right_wheel_joint/nmt_start_node std_srvs/srv/Trigger
   ros2 service call /left_wheel_joint/nmt_start_node std_srvs/srv/Trigger
   ```

3. **Initialize the Drivers:**
   ```bash
   ros2 service call /right_wheel_joint/init std_srvs/srv/Trigger
   ros2 service call /left_wheel_joint/init std_srvs/srv/Trigger
   ```

4. **Switch to Velocity Mode:**
   ```bash
   ros2 service call /right_wheel_joint/velocity_mode std_srvs/srv/Trigger
   ros2 service call /left_wheel_joint/velocity_mode std_srvs/srv/Trigger
   ```

5. **Move the Robot:**
   Move the robot by sending `Twist` messages on `/diff_drive_controller/cmd_vel_unstamped`. To stop the movement, send a `0.0` value.
   ```bash
   ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

## Project Structure

```
.
├── CMakeLists.txt
├── CONTRIBUTING.md
├── LICENSE
├── package.xml
├── README.md
├── config
│   └── robot_control
│       ├── bus.yml
│       ├── dchcan.eds
│       └── ros2_controllers.yaml
├── launch
│   └── robot_control.launch.py
├── rviz
│   └── view_robot.rviz
└── urdf
    ├── common_properties.xacro
    ├── gazebo_control.xacro
    ├── inertial_macros.xacro
    ├── robot_core.xacro
    ├── robot.urdf.xacro
    ├── ros2_control.xacro
    ├── sensors.xacro
    └── robot_controller
        ├── robot_controller.ros2_control.xacro
        └── robot_controller.urdf.xacro
```

## Contributing

Contributions are welcome! Please read the [CONTRIBUTING.md](CONTRIBUTING.md) file for details on how to contribute to this project.

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.

## Maintainer

This project is maintained by Mustafa Essam. For any inquiries, please contact [mustafa.e.mohamed@gmail.com](mailto:mustafa.e.mohamed@gmail.com).
