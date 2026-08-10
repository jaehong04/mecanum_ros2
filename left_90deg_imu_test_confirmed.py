#!/usr/bin/env python3

import struct
import time
import serial
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


PORT = "/dev/ttyUSB1"
BAUD = 115200

TOPIC = "/mecanum_drive_controller/reference_unstamped"

GYRO_SCALE = 2000.0 / 32768.0
ANGLE_SCALE = 180.0 / 32768.0

CALIBRATION_TIME = 3.0

FAST_SPEED = 0.80
SLOW_SPEED = 0.40

SLOW_START_ANGLE = 70.0

# IMU가 약 10Hz이므로 관성 회전을 고려해 조금 일찍 정지
STOP_COMMAND_ANGLE = 88.6

TIMEOUT = 8.0


def wrap_angle(angle):
    return (angle + 180.0) % 360.0 - 180.0


def read_imu(ser):
    while True:
        if ser.read(1) != b"\x55":
            continue

        if ser.read(1) != b"\x61":
            continue

        payload = ser.read(18)

        if len(payload) != 18:
            continue

        values = struct.unpack("<9h", payload)

        gz = values[5] * GYRO_SCALE
        yaw = values[8] * ANGLE_SCALE

        return gz, yaw


def publish_stop(pub):
    stop = Twist()
    pub.publish(stop)


def main():
    rclpy.init()

    node = rclpy.create_node("left_90deg_imu_test")

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT

    pub = node.create_publisher(Twist, TOPIC, qos)

    print("ROS2 컨트롤러 연결 대기...")

    wait_start = time.monotonic()

    while pub.get_subscription_count() == 0:
        if time.monotonic() - wait_start > 5.0:
            print("오류: 명령 토픽 구독자를 찾지 못했습니다.")
            node.destroy_node()
            rclpy.shutdown()
            return

        time.sleep(0.1)

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        ser.reset_input_buffer()

        print("자이로 바이어스 측정 중... 3초 동안 움직이지 마세요.")

        samples = []
        calibration_end = time.monotonic() + CALIBRATION_TIME
        start_yaw = 0.0

        while time.monotonic() < calibration_end:
            gz, yaw = read_imu(ser)
            samples.append(gz)
            start_yaw = yaw

        gyro_bias = sum(samples) / len(samples)

        print(f"바이어스: {gyro_bias:+.5f}도/s")
        print(f"샘플 수: {len(samples)}")
        print("IMU 기준 좌회전 90도 시작")

        integrated_angle = 0.0

        previous_gz, start_yaw = read_imu(ser)
        previous_gz -= gyro_bias
        previous_time = time.monotonic()

        test_start = previous_time
        last_print = previous_time
        stop_reason = "시간 초과"

        try:
            while True:
                current_angle = abs(integrated_angle)

                command = Twist()

                if current_angle < SLOW_START_ANGLE:
                    command.angular.z = FAST_SPEED
                else:
                    command.angular.z = SLOW_SPEED

                pub.publish(command)

                raw_gz, yaw = read_imu(ser)

                current_time = time.monotonic()
                corrected_gz = raw_gz - gyro_bias
                dt = current_time - previous_time

                if 0.0 < dt < 0.3:
                    integrated_angle += (
                        previous_gz + corrected_gz
                    ) * 0.5 * dt

                previous_time = current_time
                previous_gz = corrected_gz

                if current_time - last_print >= 0.2:
                    print(
                        f"\r자이로 누적각: {integrated_angle:+7.2f}도 | "
                        f"속도 명령: {command.angular.z:.2f}rad/s | "
                        f"Z 각속도: {corrected_gz:+7.2f}도/s",
                        end="",
                        flush=True,
                    )
                    last_print = current_time

                if abs(integrated_angle) >= STOP_COMMAND_ANGLE:
                    stop_reason = "정지 기준 도달"
                    break

                if current_time - test_start >= TIMEOUT:
                    break

        finally:
            publish_stop(pub)

        command_stop_angle = integrated_angle
        settle_start = time.monotonic()
        stationary_start = None

        # 정지 명령 후 관성으로 회전한 각도까지 측정
        while time.monotonic() - settle_start < 1.5:
            publish_stop(pub)

            raw_gz, final_yaw = read_imu(ser)

            current_time = time.monotonic()
            corrected_gz = raw_gz - gyro_bias
            dt = current_time - previous_time

            if 0.0 < dt < 0.3:
                integrated_angle += (
                    previous_gz + corrected_gz
                ) * 0.5 * dt

            previous_time = current_time
            previous_gz = corrected_gz

            if abs(corrected_gz) <= 2.0:
                if stationary_start is None:
                    stationary_start = current_time
                elif current_time - stationary_start >= 0.4:
                    break
            else:
                stationary_start = None

        relative_yaw = wrap_angle(final_yaw - start_yaw)

        print()
        print(f"정지 명령 시 자이로각: {command_stop_angle:+.2f}도")
        print(f"최종 자이로 누적각: {integrated_angle:+.2f}도")
        print(f"절대 Yaw 변화: {relative_yaw:+.2f}도")
        print(f"정지 후 관성 회전: {integrated_angle - command_stop_angle:+.2f}도")
        print(f"종료 원인: {stop_reason}")
        print("정지 완료")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
