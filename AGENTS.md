# 메카넘휠 로봇 프로젝트

## 1. 프로젝트 개요

이 저장소는 실제 4륜 메카넘 모바일 로봇을 위한 ROS2 Humble 작업공간이다.

주요 플랫폼:

- Jetson Orin Nano Super
- Arduino UNO
- QGPMaker Motor Shield
- 엔코더 DC 모터 ×4
- 80 mm 메카넘 휠 ×4
- WitMotion IMU
- 외부 고정 RGB 카메라
- ROS2 Humble
- Nav2

기본 자율주행 시스템은 이미 실제 로봇에서 구현 및 테스트가 완료된 상태이다.

현재 작업은 검증된 기존 구조를 유지하면서 점진적인 개선, 디버깅, 검증 및 튜닝을 수행하는 것을 기본으로 한다.

이 저장소를 처음부터 다시 설계해야 하는 새로운 로봇 프로젝트로 취급하지 않는다.

---

## 2. 개발 환경

### 메인 환경

```text
OS        : Ubuntu 22.04
ROS2      : Humble
Computer  : Jetson Orin Nano Super
Workspace : ~/mecanum_ros2_ws
```

### 주요 ROS2 패키지

```
mecanum_serial_hardware
mecanum_bringup
```

### ROS2 네트워크

```
ROS_DOMAIN_ID=78
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_LOCALHOST_ONLY=0
```

현재 통신 미들웨어는 CycloneDDS를 사용한다.

### 하드웨어

```
Arduino UNO
QGPMaker Motor Shield
Encoder DC Motor ×4
Mecanum Wheel Ø80 mm ×4
WitMotion IMU
External fixed RGB camera
```

---

## 3. 시스템 구조

### Local Localization

```
Wheel Encoder
+
IMU
↓
Local EKF
↓
odom → base_link
```

Local EKF는 로봇의 로컬 이동 추정을 담당한다.

### Global Localization

```
External RGB Camera
↓
ArUco Localization
↓
/external_camera/pose
↓
Global EKF
↓
map → odom
```

### 최종 TF 구조

```
map
↓
odom
↓
base_link
```

명확한 이유 없이 동일한 핵심 TF를 추가로 발행하는 publisher를 만들지 않는다.

### 자율주행 및 제어

```
RViz / Nav2 Goal
↓
Nav2 Planner
↓
DWBLocalPlanner
↓
/cmd_vel
↓
cmd_vel_bridge
↓
/mecanum_drive_controller/reference_unstamped
↓
mecanum_drive_controller
↓
ros2_control hardware interface
↓
Arduino
↓
Motors
```

`/cmd_vel` bridge는 의도적으로 사용 중인 구조이다.

명시적인 요청이 없는 한 직접 remapping 방식으로 변경하지 않는다.

---

## 4. 검증된 고정 설정

다음 설정들은 실제 로봇에서 정상 동작이 검증되었다.

특별한 이유 없이 변경하지 않는다.

### 모터 매핑

```
FL = M1
RL = M2
RR = M3
FR = M4
```

### 바퀴 / 엔코더 기준

```
Wheel diameter       ≈ 80 mm
Wheel circumference  ≈ 251.3 mm
Gear ratio           ≈ 1:90
Encoder CPR          ≈ 4300
```

이 값들은 일반적인 메카넘 로봇의 기본값이 아니라 현재 프로젝트에서 검증된 기준값이다.

### Controller

현재 검증된 Controller:

```
joint_state_broadcaster
mecanum_drive_controller
```

정상 상태:

```
joint_state_broadcaster   active
mecanum_drive_controller  active
```

Encoder Odometry Topic:

```
/mecanum_drive_controller/odometry
```

이전에 검증된 주기:

```
≈ 50 Hz
```

### 제어 체인

명시적인 요청이 없는 한 다음 구조를 유지한다.

```
/cmd_vel
→ cmd_vel_bridge
→ /mecanum_drive_controller/reference_unstamped
→ mecanum_drive_controller
→ Arduino
```

### Localization 역할

각 EKF의 역할은 다음과 같이 분리한다.

```
Local EKF
→ odom → base_link

Global EKF
→ map → odom
```

### Navigation Controller

현재 Local Controller:

```
DWBLocalPlanner
```

자동으로 다른 Nav2 Controller로 교체하지 않는다.

### 현재 검증된 Git 기준점

```
Branch          : main
Verified commit : 29a9c2b
Remote          : origin/main
```

2026-09-16 Codex 도입 시점 기준:

```
Working tree: clean
main = origin/main
```

사용자가 더 최신의 정상 동작 Commit을 확정하기 전까지 `29a9c2b`를 현재 알려진 정상 기준점으로 취급한다.

---

## 5. 하드웨어 / Serial 규칙

Serial 장치 번호는 변경될 수 있다.

`/dev/ttyUSB0`, `/dev/ttyUSB1` 번호만 보고 장치를 단정하지 않는다.

먼저 현재 시스템을 확인한다.

```bash
ls -l /dev/serial/by-path/
ls -l /dev/ttyUSB*
```

현재 검증된 물리 연결 기준:

```
Arduino:
USB 2.2
/dev/ttyUSB0
/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0

IMU:
USB 2.4
/dev/ttyUSB1
/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0-port0
```

이 값들은 현재 검증된 값이지 영구적으로 고정되는 값은 아니다.

Arduino와 IMU는 이전 확인에서 동일한 USB Serial Chip 정보를 사용했다.

```
VID    = 1a86
PID    = 7523
SERIAL = 1a86_USB_Serial
```

따라서 VID/PID/Serial 값만으로 두 장치를 구분하지 않는다.

Arduino는 Serial Protocol을 이용해 확인할 수 있다.

```
PING
→ PONG
```

정상 Bringup에서는 이전에 다음이 확인되었다.

```
Arduino handshake OK (PONG)
IMU Run!
```

Arduino Serial Baudrate:

```
115200
```

Bringup 실행 중에는 `ros2_control`이 이미 Arduino Serial Port를 사용하고 있을 수 있다.

현재 사용 중인 Serial Port를 확인하지 않고 다른 프로그램에서 동시에 열지 않는다.

사용자가 명시적으로 요청하지 않는 한 Arduino Firmware를 업로드하지 않는다.

모터를 움직이거나 실제 로봇을 동작시킬 수 있는 명령을 실행하기 전에는 사용자에게 먼저 알린다.

---

## 6. 작업 규칙

1. 어떤 것을 수정하기 전에 기존 구현을 먼저 조사한다.
2. 검증된 시스템을 명확한 이유 없이 다시 설계하지 않는다.
3. 설정값을 변경하기 전에 현재 값을 먼저 확인한다.
4. 가능한 한 한 번에 하나의 원인 또는 튜닝 변수만 변경한다.
5. 사용자가 요청한 범위를 넘어 임의로 작업을 확장하지 않는다.
6. 임시 해결책보다 기존 프로젝트 구조와 일관된 해결 방법을 우선한다.
7. 확인 가능한 하드웨어 상태를 추측하지 않는다.
8. 소프트웨어 검증과 실제 하드웨어 검증을 구분한다.
9. Arduino Firmware를 자동으로 업로드하지 않는다.
10. 명시적인 요청 없이 EKF 또는 TF 구조를 변경하지 않는다.
11. 명시적인 요청 없이 Nav2 Controller를 교체하지 않는다.
12. 실제 하드웨어 근거 없이 Motor Mapping을 변경하지 않는다.
13. Serial Device Path를 조용히 임의 변경하지 않는다.
14. 관련 없는 파일을 수정하지 않는다.
15. 수정 후 반드시 변경사항을 확인한다.
16. 코드 수정 후 필요한 Build 또는 문법 검증을 수행한다.
17. 무엇을, 어디서, 왜 변경했는지 명확히 보고한다.
18. 현재 저장소 내용과 이 파일의 내용이 다르면, 어느 한쪽에 맞추기 전에 차이점을 사용자에게 보고한다.
19. 실제 테스트하지 않은 변경사항을 실제 로봇에서 정상 동작한다고 표현하지 않는다.
20. 한 부분을 디버깅하면서 이미 정상 동작하는 다른 부분을 훼손하지 않는다.

---

## 7. Navigation / DWB 규칙

이 로봇은 Holonomic / Omnidirectional 플랫폼이다.

Differential Drive 로봇이 아니다.

지원되는 기본 움직임:

```
전진 / 후진
좌 / 우 횡이동
대각선 이동
제자리 회전
```

Navigation 분석 시 다음을 모두 고려한다.

```
vx
vy
wz
```

Differential Drive를 전제로 한 설정을 메카넘 로봇에 적합한지 검토하지 않고 적용하지 않는다.

현재 Local Controller:

```
DWBLocalPlanner
```

DWB 튜닝 시 다음 절차를 따른다.

1. 현재 Parameter 값을 먼저 확인한다.
2. 기존 값을 기록한다.
3. 변경 이유를 설명한다.
4. 가능한 한 실험당 하나의 Parameter만 변경한다.
5. 여러 Parameter를 동시에 크게 변경하지 않는다.
6. 설정 또는 Build를 검증한다.
7. 실제 로봇 테스트는 사용자가 수행한다.
8. 실제 결과를 비교한 후 다음 튜닝을 진행한다.

현재 개선 대상:

- 경로 진행 방향으로 먼저 회전하려는 현상
- 횡방향 `vy` 사용 부족
- 자연스러운 메카넘 대각 이동 활용 부족
- 일부 Goal 근처에서 지속적으로 회전하거나 Goal 도달에 실패하는 현상

이 항목들은 현재 개선 중인 문제이며 해결 완료된 문제로 취급하지 않는다.

DWB 변경 후 실제 로봇 주행 테스트가 완료되기 전에는 성공했다고 판단하지 않는다.

---

## 8. 카메라 / Localization 규칙

### 카메라

현재 카메라:

```
ARC International Camera
HD Web Camera
VID:PID = 05a3:9331
1920 × 1080
MJPG
30 FPS
```

### ArUco

현재 검증된 Marker 설정:

```
Dictionary    = DICT_4X4_50
Marker ID     = 0
Marker Size   = 18.8 cm
Marker Height = 24.4 cm
```

과거의 Marker Height `16.3 cm`는 사용하지 않는다.

코드 기준:

```python
MARKER_HEIGHT_M = 0.244
PLANE_Z = -MARKER_HEIGHT_M
```

### Position Localization

현재 Localization 방식:

```
ArUco Detection
→ Pixel Position
→ Camera Ray 계산
→ World Ray
→ Marker Plane Intersection
→ World X/Y
```

현재 프로젝트는 Ray–Plane Projection 방식을 사용한다.

명시적인 요청 없이 단순 Homography 기반 위치 계산으로 교체하지 않는다.

### Camera → ROS map 좌표 변환

현재 검증된 변환:

```
X_ros = -X_camera
Y_ros =  Y_camera
```

전체 Map / RViz / Localization 체인을 다시 검증하지 않은 상태에서 이 좌표 기준을 변경하지 않는다.

### Yaw

현재 실제 로봇 기준 Yaw 부호:

```
좌회전 = +
우회전 = -
```

Yaw Zero Offset 저장 파일:

```
yaw_offset.json
```

현재 검증된 Offset 기준:

```
yaw_offset_deg ≈ +0.287°
```

명확한 이유 없이 Yaw Zero를 다시 보정하지 않는다.

### 카메라 화면 방향

외부카메라 화면은 다음을 사용한다.

```python
cv2.flip(frame, -1)
```

Camera View, ROS Map, Static Map, RViz는 이 기준을 중심으로 정렬되어 있다.

### 카메라 위치 변경

실제 주행 공간은 그대로이고 카메라의 위치 또는 각도만 변경된 경우:

```
Camera Intrinsic 유지
Static Map 유지
Camera Extrinsic 재보정
X/Y 검증
Yaw 검증
TF 검증
Nav2 검증
```

카메라 위치가 바뀌었다는 이유만으로 Nav2 Static Map을 다시 만들지 않는다.

### ROS Localization 출력

External Camera Pose:

```
Topic   : /external_camera/pose
Message : geometry_msgs/PoseWithCovarianceStamped
Frame   : map
```

현재 구조:

```
/external_camera/pose
→ Global EKF
→ map → odom
```

다음 Local Localization 구조와 분리해서 유지한다.

```
Encoder + IMU
→ Local EKF
→ odom → base_link
```

---

## 9. 검증 절차

일반적인 수정 작업은 다음 순서로 진행한다.

```
현재 상태 확인
↓
관련 파일 조사
↓
기존 설정값 확인
↓
최소한의 변경 계획 수립
↓
필요한 파일만 수정
↓
git diff
↓
Build / Syntax 검증
↓
필요한 경우 사용자가 실제 로봇 테스트
↓
결과 확인
↓
변경 유지 / 재수정 / 되돌리기 결정
↓
사용자 확인 후 Commit 여부 결정
```

Build 성공만으로 실제 로봇이 정상 동작한다고 판단하지 않는다.

다음 단계들은 서로 별개이다.

```
Syntax 정상
Build 성공
ROS Node 실행 성공
ROS Interface 정상
실제 로봇 동작 정상
```

근거 없이 다음 검증 단계까지 완료된 것으로 판단하지 않는다.

---

## 10. Git 안전 규칙

파일 수정 전:

```bash
git status
```

파일 수정 후:

```bash
git status
git diff
```

규칙:

- 자동으로 `git commit`하지 않는다.
- 자동으로 `git push`하지 않는다.
- 사용자가 요청하지 않은 Branch로 변경하지 않는다.
- `git reset --hard`를 사용하지 않는다.
- `git clean` 등 파괴적인 정리 명령을 사용하지 않는다.
- 사용자 변경사항을 명시적인 승인 없이 삭제하지 않는다.
- Git History를 임의로 다시 작성하지 않는다.
- 수정된 모든 파일을 명확하게 보고한다.
- 실제 하드웨어 테스트가 필요한 변경은 정상 동작 확인 전에 Commit하지 않는다.

현재 알려진 정상 기준점:

```
29a9c2b
Update external camera recalibration and localization
```

---

## 11. Build / Test 규칙

ROS2 Build 전에 적절한 ROS 환경을 설정한다.

기본 Workspace 설정:

```bash
cd ~/mecanum_ros2_ws
source /opt/ros/humble/setup.bash
```

가능하면 전체 Workspace Build보다 변경된 Package만 Build하는 것을 우선 검토한다.

예:

```bash
colcon build --packages-select <package_name> --symlink-install
```

전체 Build가 필요한 경우:

```bash
colcon build --symlink-install
```

Build 성공 후 필요한 경우:

```bash
source install/setup.bash
```

수정한 파일 종류에 맞는 검증도 수행한다.

```
Python
→ 필요 시 Syntax / Import 확인

YAML
→ 문법 및 들여쓰기 확인

Launch File
→ Python 문법 및 ROS2 Launch 호환 확인

ROS Parameter
→ Namespace와 Parameter 위치 확인

Arduino
→ 업로드를 고려하기 전에 먼저 Compile
```

Compile 성공만으로 Arduino Firmware를 업로드하지 않는다.

실제 로봇 주행 테스트를 자동으로 시작하지 않는다.

---

## 12. 유용한 Runtime 확인 명령

Runtime 오류가 발생했을 때 바로 구조를 변경하지 말고 먼저 다음을 확인한다.

### Serial Device

```bash
ls -l /dev/serial/by-path/
ls -l /dev/ttyUSB*
```

### Controller

```bash
ros2 control list_controllers
```

### Odometry

```bash
ros2 topic hz /mecanum_drive_controller/odometry
```

### Local TF

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

### External Camera Pose

```bash
ros2 topic hz /external_camera/pose
```

### Global TF

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Runtime 확인으로 문제 원인을 찾을 수 있다면 먼저 설정을 변경하지 않는다.

---

## 13. 현재 개발 작업

이 항목은 현재 작업 방향이며 고정 설정이 아니다.

현재 주요 작업:

```
1. Nav2 / DWB Path Following 개선
2. 메카넘 횡이동 및 대각이동 활용 개선
3. Goal 근처 회전 문제 분석
4. 점진적인 DWB Parameter 튜닝
5. Arduino Sketch / QGPMaker Library 상태 확인
```

기본 자율주행 통합 자체는 이미 구현된 상태이다.

현재 튜닝 작업을 전체 Navigation Stack을 새로 만드는 작업으로 혼동하지 않는다.

Arduino 관련 현재 작업은 기존에 사용하던 올바른 Firmware와 필요한 Library를 확인하거나 복구하는 작업이다.

기존 구현이 사용할 수 없거나 부적합하다는 것이 확인되지 않는 한 새로운 Arduino 제어 구조를 처음부터 만들지 않는다.

---

## 14. 변경사항 보고 규칙

요청된 수정이 끝나면 최소한 다음 내용을 보고한다.

```
수정된 파일
기존 값 / 동작
변경된 값 / 동작
변경 이유
수행한 검증
아직 수행하지 않은 검증
남아 있는 위험 요소 또는 실제 로봇 테스트 필요 여부
```

튜닝 작업에서는 다음을 명확히 기록한다.

```
Parameter
기존 값
변경 값
변경 이유
예상 효과
사용자 실제 테스트 후 실제 효과
```

관련 없는 변경사항을 큰 수정 작업 안에 숨기지 않는다.

---

## 15. 핵심 원칙

이 프로젝트는 이미 검증된 정상 동작 구조를 가지고 있다.

따라서 기본 작업 원칙은 다음과 같다.

```
먼저 이해
↓
최소한으로 수정
↓
Software 검증
↓
필요 시 실제 Hardware 검증
↓
검증된 개선사항만 유지
```

사용자가 요청한 특정 부분을 개선하는 동안 이미 정상 동작하는 다른 시스템을 최대한 보존한다.
