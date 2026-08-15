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

`/cmd_vel`

Type:

`geometry_msgs/msg/Twist`

The `mecanum_drive_controller` internal `reference_unstamped` input is remapped to `/cmd_vel`.

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
```

## Run

```bash
cd ~/mecanum_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mecanum_bringup mecanum_bringup.launch.py
```

## Controller Check

```bash
ros2 control list_controllers
```

Expected:

```text
joint_state_broadcaster     active
mecanum_drive_controller    active
```

## Baseline Version

### v1.0-rpi-working

Raspberry Pi 4에서 다음 기능이 정상 동작한 기준 버전이다.

- ROS2 Mecanum Control
- Arduino Serial Communication
- Encoder Feedback
- Odometry
- IMU Yaw
- Forward / Backward
- Left / Right Mecanum Motion
- Left / Right 90 degree Rotation


---

# Jetson Current Status

현재 Raspberry Pi 기반 시스템의 Jetson Orin Nano Super 이전이 완료되었다.

## Verified on Jetson

- Ubuntu 22.04 / ROS2 Humble
- ROS_DOMAIN_ID=78
- Arduino UNO Serial Communication
- 4 Mecanum Motor Control
- Encoder Feedback
- Joint State
- Wheel Odometry
- WitMotion IMU approximately 100 Hz
- robot_localization EKF approximately 50 Hz
- odom -> base_link TF
- CAD-based RViz Robot Model
- Individual Wheel Joint Animation

## Mecanum Kinematics

```text
wheel_radius: 0.04 m
sum_of_robot_center_projection_on_X_Y_axis: 0.199546 m
```

CAD measured wheel-center geometry:

```text
Front-Rear: approximately 192.091 mm
Left-Right: approximately 207.001 mm
```

The value `0.199546` was selected after comparison with the previous
`0.205` setting using repeated left/right 90 degree rotation tests.

## Nav2

ROS2 Humble Navigation2 is installed and configured for holonomic Mecanum motion.

Controller:

```text
dwb_core::DWBLocalPlanner
```

Initial velocity limits:

```text
X:   -0.12 ~ +0.12 m/s
Y:   -0.12 ~ +0.12 m/s
Yaw: -0.70 ~ +0.70 rad/s
```

Nav2 command path:

```text
controller_server
      |
      v
/cmd_vel_nav
      |
      v
velocity_smoother
      |
      v
/cmd_vel
      |
      v
MecanumDriveController
      |
      v
Arduino UNO / Motors
```

Actual hardware command tests completed:

- Forward X motion
- Lateral Y motion
- Yaw rotation

Nav2 robot footprint:

```text
[[0.145, 0.123],
 [0.145, -0.123],
 [-0.145, -0.123],
 [-0.145, 0.123]]
```

Footprint padding:

```text
0.01 m
```

## Remaining Nav2 Integration

External camera integration is still required for:

- Global localization
- map -> odom
- Obstacle / object information
- Nav2 Costmap
- Full autonomous navigation

## Stable Versions

```text
v1.0-rpi-working
v2.0-jetson-working
v2.1-jetson-calibrated
v2.2-nav2-ready
```

Current stable baseline:

```text
v2.2-nav2-ready
```
