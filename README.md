# roboteq_motor_controller_driver

### Setting the configuration roboteq motor controller parameters

```yaml
/roboteq_motor_controller_node:
  ros__parameters:
    roboteq_motor_controller:
      port_name: "/dev/ttyACM0" # Port name of left motor controller serial port.
      frame_id: "roboteq_motor_controller" # Frame ID
      baudrate: 115200 # Baudrate of the roboteq motor controller serial port. 
      left_motor_direction: 1 # Left motor direction, it can be 1 or -1
      right_motor_direction: 1 # Right motor direction, it can be 1 or -1
      motor_acceleration: 1.0 # Acceleration of left and right motors.
      motor_deceleration: 1.0 # Deceleration of left and right motors.
```

### Build roboteq_motor_controller package
```bash
cd ~ros2_ws/src && git clone https://github.com/furkansariyildiz/roboteq_motor_controller_driver.git
```
```bash
cd ~ros2_ws && colcon build --symlink-install --packages-select roboteq_motor_controller
```

### Run command via launch
```bash
ros2 launch roboteq_motor_controller roboteq_motor_controller.launch.py
```