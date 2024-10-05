# roboteq_motor_controller_driver (ROS2)   <img src="documents/roboteq-logo.jpg" style="width: 10%; height: 10%"/><img src="documents/ros2.png" style="width: 10%; height: 10%"/>

<p align="center">
  <img src="documents/roboteq.png" style="width: 50%; height: 50%"/>
</p>

### Setting the configuration roboteq motor controller parameters

Roboteq Motor Controller Driver for ROS2 (Humble)

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

# Usage 
## Subscribers 
### /roboteq_motor_controller/motors_rpm
This package subscribes **/roboteq_motor_controller/motors_rpm** topic for getting desired RPM values. After getting this message, package sends RPM values to motor controller for left and right motors.

**roboteq_motor_controller::msg::RPM**
```msg
std_msgs/Header header
std_msgs/Float64 left_motor_rpm
std_msgs/Float64 right_motor_rpm
```

## Publishers
### /roboteq_motor_controller/motors_rpm_output
Package publishes motor RPM values to **/roboteq_motor_controller/motors_rpm_output** topic which are readed from motor controller with **roboteq_motor_controller::msg::RPM** type.

**roboteq_motor_controller::msg::RPM**
```msg
std_msgs/Header header
std_msgs/Float64 left_motor_rpm
std_msgs/Float64 right_motor_rpm
```

### /roboteq_motor_controller/motor_controller_flags
Package publsihes motor controller flags to **/roboteq_motor_controller/motor_controller_flags** topic with **roboteq_motor_controller::msg::MotorControllerFlags** type.

**roboteq_motor_controller::msg::MotorControllerFlags**
```msg
std_msgs/Header header
string[] fault_flags
string[] left_motor_flags
string[] right_motor_flags
string[] status_flags
```

**fault_flags:** String array for motor and motor controller faults. Fault flag can contains following values;
```cpp
{"DefaultConfigLoaded", "MosfetFail", "MotorSensorSetupFault", "EmergencyStop", "ShotCircuit", "UnderVoltage", "OverVoltage", "OverHeat"}
```
**left_motor_flags:** String array for left motor. Motor flags can contain following values;
```cpp
{"AmpsTriggerActivated", "ReverseLimitTriggered", "ForwardLimitTriggered", "SafetyStopActive", "LoopErrorDetected", "MotorStalled", "AmpsLimitCurrentlyActive"}
```

**right_motor_flags:** String array for left motor. Motor flags can contain following values;
```cpp
{"AmpsTriggerActivated", "ReverseLimitTriggered", "ForwardLimitTriggered", "SafetyStopActive", "LoopErrorDetected", "MotorStalled", "AmpsLimitCurrentlyActive"}
```

**status_flags:** String array for motor controller status flags. It can contain following values;
```cpp
{"Setup", "RunScript", "STO", "AtLimit", "StallDetected", "PowerStageOff", "AnalogMode", "PulseMode", "SerialMode"},
```

### /roboteq_motor_controller/motor_controller_status
Package publishes motor RPM values to **/roboteq_motor_controller/motor_controller_status** topic which are readed from motor controller with **roboteq_motor_controller::msg::MotorControllerStatus** type. 

**roboteq_motor_controller::msg::MotorControllerStatus**
```msg
std_msgs/Header header
roboteq_motor_controller/MotorStatus left_motor_status
roboteq_motor_controller/MotorStatus right_motor_status
``` 
+ **roboteq_motor_controller/MotorStatus**
  ```msg
  std_msgs/Header header
  roboteq_motor_controller/MotorCurrent current
  roboteq_motor_controller/MotorTemperature temperature 
  ```

  + **roboteq_motor_controller/MotorCurrent**
    ```msg
    std_msgs/Float64 amper 
    ```

  + **roboteq_motor_controller/MotorTemperature**
    ```msg
    std_msgs/Float64 celsius
    ```

### Contact with me all around the web:
[![LinkedIn Badge](https://img.shields.io/badge/LinkedIn-Profile-informational?style=flat&logo=linkedin&logoColor=white&color=0D76A8)](https://www.linkedin.com/in/furkan-sariyildiz/)


