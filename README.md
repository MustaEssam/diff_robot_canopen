# diff_robot_canopen

This repository contains a working example of a differential drive robot using the `ros2_canopen` package. It is designed to work with real drivers, not simulated ones.

## Prerequisites

Before you begin, ensure you have met the following requirements:
- You have installed ROS 2.
- You have a CAN controller set up.
- You have installed the `ros2_canopen` package by following [this guide](https://ros-industrial.github.io/ros2_canopen/manual/rolling/quickstart/installation.html). Make sure to switch to the appropriate ROS distribution branch in the repository. For example, if you are using the `humble` distribution, use the following command:

```bash
git clone -b humble https://github.com/ros-industrial/ros2_canopen.git
```

## Step 1: Setup CAN Controller

Follow [this guide](https://ros-industrial.github.io/ros2_canopen/manual/rolling/quickstart/operation.html) to set up your CAN controller:

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```

## Step 2: Launch the Robot Control

Launch the `robot_control.launch.py` file to start the robot control:

```bash
ros2 launch diff_robot_canopen robot_control.launch.py 
```

## Step 3: Start Communication with Drivers

Start communication with the drivers by calling the following services:

```bash
ros2 service call /right_wheel_joint/nmt_start_node std_srvs/srv/Trigger 
ros2 service call /left_wheel_joint/nmt_start_node std_srvs/srv/Trigger 
```

Initialize the drivers:

```bash
ros2 service call /right_wheel_joint/init std_srvs/srv/Trigger
ros2 service call /left_wheel_joint/init std_srvs/srv/Trigger
```

Switch to velocity mode:

```bash
ros2 service call /right_wheel_joint/velocity_mode std_srvs/srv/Trigger 
ros2 service call /left_wheel_joint/velocity_mode std_srvs/srv/Trigger 
```

## Step 4: Move the Robot

Move the robot by sending **`Twist`** messages on `/diff_drive_controller/cmd_vel_unstamped`. To stop the movement, send a `0.0` value.

```bash
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.

## Maintainer

This project is maintained by Mustafa Essam. For any inquiries, please contact [mustafa.e.mohamed@gmail.com](mailto:mustafa.e.mohamed@gmail.com).