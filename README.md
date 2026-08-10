# Mecanum ROS2

ROS2 Humble 기반 4WD Mecanum Wheel Mobile Robot 제어 프로젝트.

Raspberry Pi 4에서 정상 구동된 시스템을 기준으로 하며,
Jetson Orin Nano Super로 이전하여 사용하는 것을 목표로 한다.

## Environment

- Ubuntu 22.04
- ROS2 Humble
- ros2_control
- Arduino UNO
- QGPMaker Motor Shield
- Encoder DC Motor x4
- 80mm Mecanum Wheel x4
- WitMotion IMU

## Motor Mapping

| Position | Motor |
|---|---|
| Front Left (FL) | M1 |
| Rear Left (RL) | M2 |
| Rear Right (RR) | M3 |
| Front Right (FR) | M4 |

## ROS2 Packages

### mecanum_serial_hardware

ROS2 ros2_control과 Arduino UNO 사이의 Serial Hardware Interface.

### mecanum_bringup

Robot description, controller configuration 및 Bringup Launch 관리.

## Controllers

- joint_state_broadcaster
- mecanum_drive_controller

## Command Topic

`/mecanum_drive_controller/reference_unstamped`

Type:

`geometry_msgs/msg/Twist`

- linear.x : Forward / Backward
- linear.y : Left / Right
- angular.z : Rotation

## Odometry Topic

`/mecanum_drive_controller/odometry`

## Arduino Firmware

`arduino/mecanum_ros2_control/mecanum_ros2_control.ino`

현재 주요 설정:

- Serial: 115200
- MAX_PWM: 200
- KFF: 25.0
- KP: 6.0
- Encoder: approximately 4300 counts/rev

## Build

```bash
cd ~/mecanum_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
