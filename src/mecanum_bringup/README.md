# Mecanum bringup (ROS 2 Humble)

## 실행

로봇을 바닥에서 띄워 놓고 첫 시험을 진행한다.

```bash
cd ~/mecanum_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mecanum_bringup mecanum_bringup.launch.py
```

위 bringup은 터미널 입력 모드를 변경하지 않는다. 키보드 제어는 **새 터미널**을
열어 다음처럼 실행한다.

```bash
cd ~/mecanum_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run mecanum_bringup mecanum_keyboard
```

장치명이 다르면 안정적인 `by-id` 경로를 전달한다.

```bash
ls -l /dev/serial/by-id/
ros2 launch mecanum_bringup mecanum_bringup.launch.py \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

키 배치는 다음과 같다.

```text
q  w  e    전진 좌대각 / 전진 / 전진 우대각
a  s  d    좌횡이동 / 정지 / 우횡이동
z  x  c    후진 좌대각 / 후진 / 후진 우대각
r  f       반시계 / 시계 방향 제자리 회전
t  y       전진 좌회전 곡선 / 전진 우회전 곡선
g  h       좌/우 횡이동하면서 회전
v  b       후진 좌회전 곡선 / 후진 우회전 곡선
+  -       속도 증가 / 감소
Space      즉시 정지
```

기본 속도는 선속도 0.12 m/s, 각속도 0.7 rad/s이다. 방향 키를 한 번
누르면 그 명령을 유지하므로 이동이 끝나면 반드시 `s` 또는 Space를 누른다.
컨트롤러 watchdog은 키보드 노드가 종료되면 0.3초 후 정지하며, Ctrl-C 종료
시에도 정지 명령을 보낸다.

RViz 없이 실행하려면 `use_rviz:=false`를 붙인다. 키보드 노드는 터미널 입력
모드를 사용하므로 bringup launch에서는 실행하지 않으며 반드시 별도 터미널에서
`ros2 run mecanum_bringup mecanum_keyboard`로 실행한다. RViz의 `odom -> base_link`는 엔코더 속도로 계산한
odometry이고, 각 `wheel_link` 회전은 엔코더 속도를 적분한 joint position이다.

## 점검 명령

```bash
ros2 control list_controllers
ros2 topic echo /mecanum_drive_controller/odometry
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo odom base_link
```

## IMU 융합

bringup은 `/imu/data` (`sensor_msgs/msg/Imu`)와 휠 odometry를 EKF로 융합한다.
IMU 노드는 `header.frame_id: imu_link`로 발행해야 하며, EKF 결과
`/odometry/filtered`가 유일한 `odom -> base_link` TF를 만든다.

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
```

IMU의 orientation covariance 첫 값이 `-1`이면 orientation이 제공되지 않는다는
뜻이므로 차체를 손으로 돌린 자세를 반영할 수 없다. 이 경우 사용하는 IMU
드라이버에서 quaternion orientation과 covariance를 발행하거나 별도의 AHRS
필터를 사용해야 한다. `imu_joint`의 xyz/rpy는 실제 센서 장착 방향에 맞춘다.

컨트롤러 두 개가 모두 `active`이고 `/joint_states`의 네 바퀴 velocity가 실제
회전에 따라 변해야 한다. 방향이 반대이거나 순서가 섞이면 Arduino가 보내는
`E,FL,FR,RL,RR`와 받는 `V,FL,FR,RL,RR` 순서 및 모터 배선을 먼저 확인한다.
