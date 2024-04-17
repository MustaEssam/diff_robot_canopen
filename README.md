# diff_robot_canopen

This is a Working example of a differential drive robot with ros2_canopen package.
Works on a real driver not a fake one.

## Step 1:

you have to setup CAN controller following [this guide](https://ros-industrial.github.io/ros2_canopen/manual/rolling/quickstart/operation.html):

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```

## Step 2:

Launch robot_control.launch.py

```bash
ros2 launch diff_robot_canopen robot_control.launch.py 
```

## Step 3:

Start communication with drivers:

```bash
ros2 service call /right_wheel_joint/nmt_start_node std_srvs/srv/Trigger 
ros2 service call /left_wheel_joint/nmt_start_node std_srvs/srv/Trigger 
```

initialize the drivers:

```bash
ros2 service call /right_wheel_joint/init std_srvs/srv/Trigger
ros2 service call /left_wheel_joint/init std_srvs/srv/Trigger
```

Switch to velocity mode:

```bash
ros2 service call /right_wheel_joint/velocity_mode std_srvs/srv/Trigger 
ros2 service call /left_wheel_joint/velocity_mode std_srvs/srv/Trigger 
```

## Step 4:

Move the robot by sending **`Twist`** messages on `/diff_drive_controller/cmd_vel_unstamped` 

**Note:** You have to send 0.0 to stop the movement.