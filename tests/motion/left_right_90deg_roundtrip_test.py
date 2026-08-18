#!/usr/bin/env python3

import struct
import time
import serial
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


PORT = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0"
BAUD = 115200

TOPIC = "/cmd_vel"

GYRO_SCALE = 2000.0 / 32768.0

CALIBRATION_TIME = 3.0

FAST_SPEED = 0.80
SLOW_SPEED = 0.40
SLOW_START_ANGLE = 70.0

LEFT_STOP_ANGLE = 88.6
RIGHT_STOP_ANGLE = 87.5

TIMEOUT = 8.0

ROUNDTRIP_COUNT = 5
INTER_TEST_DELAY = 3.0
SETTLE_TIMEOUT = 1.5


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

        return gz


def publish_stop(pub):
    stop = Twist()
    pub.publish(stop)


def calibrate_gyro(ser):
    print("자이로 바이어스 측정 중... 3초 동안 움직이지 마세요.")

    samples = []
    end_time = time.monotonic() + CALIBRATION_TIME

    while time.monotonic() < end_time:
        gz = read_imu(ser)
        samples.append(gz)

    gyro_bias = sum(samples) / len(samples)

    print(f"바이어스: {gyro_bias:+.5f}도/s")
    print(f"샘플 수: {len(samples)}")

    return gyro_bias


def rotate_90(
    ser,
    pub,
    gyro_bias,
    direction,
    stop_angle,
    label
):
    print()
    print(f"===== {label} 시작 =====")

    integrated_angle = 0.0

    previous_gz = read_imu(ser) - gyro_bias
    previous_time = time.monotonic()

    test_start = previous_time
    stop_reason = "시간 초과"

    try:
        while True:
            current_angle = abs(integrated_angle)

            command = Twist()

            if current_angle < SLOW_START_ANGLE:
                speed = FAST_SPEED
            else:
                speed = SLOW_SPEED

            command.angular.z = direction * speed

            pub.publish(command)

            raw_gz = read_imu(ser)

            current_time = time.monotonic()
            corrected_gz = raw_gz - gyro_bias
            dt = current_time - previous_time

            if 0.0 < dt < 0.3:
                integrated_angle += (
                    previous_gz + corrected_gz
                ) * 0.5 * dt

            previous_time = current_time
            previous_gz = corrected_gz

            if abs(integrated_angle) >= stop_angle:
                stop_reason = "정지 기준 도달"
                break

            if current_time - test_start >= TIMEOUT:
                break

    finally:
        publish_stop(pub)

    command_stop_angle = integrated_angle

    settle_start = time.monotonic()
    stationary_start = None

    while time.monotonic() - settle_start < SETTLE_TIMEOUT:
        publish_stop(pub)

        raw_gz = read_imu(ser)

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

    inertia_angle = integrated_angle - command_stop_angle

    print(f"정지 명령 시 각도 : {command_stop_angle:+.2f}°")
    print(f"최종 자이로 각도   : {integrated_angle:+.2f}°")
    print(f"정지 후 관성 회전 : {inertia_angle:+.2f}°")
    print(f"90° 기준 오차      : {abs(integrated_angle) - 90.0:+.2f}°")
    print(f"종료 원인          : {stop_reason}")
    print(f"===== {label} 완료 =====")

    return integrated_angle


def main():
    rclpy.init()

    node = rclpy.create_node("left_right_90deg_roundtrip_test")

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT

    pub = node.create_publisher(
        Twist,
        TOPIC,
        qos
    )

    print("ROS2 컨트롤러 연결 대기...")

    wait_start = time.monotonic()

    while pub.get_subscription_count() == 0:
        if time.monotonic() - wait_start > 5.0:
            print("오류: /cmd_vel 구독자를 찾지 못했습니다.")
            node.destroy_node()
            rclpy.shutdown()
            return

        time.sleep(0.1)

    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            ser.reset_input_buffer()

            print()
            print("======================================")
            print(f"좌 90° → 우 90° 자동 왕복 {ROUNDTRIP_COUNT}회")
            print("======================================")
            print("외부카메라 자동 측정 대기...")
            time.sleep(1.0)

            for test_num in range(1, ROUNDTRIP_COUNT + 1):

                print()
                print("######################################")
                print(f"왕복 테스트 #{test_num} / {ROUNDTRIP_COUNT}")
                print("######################################")

                # ------------------------------
                # 좌회전
                # ------------------------------
                left_bias = calibrate_gyro(ser)

                left_angle = rotate_90(
                    ser,
                    pub,
                    left_bias,
                    direction=+1.0,
                    stop_angle=LEFT_STOP_ANGLE,
                    label=f"#{test_num} 좌회전 90도"
                )

                print()
                print("좌회전 완료 - 우회전 전 안정화")
                time.sleep(1.0)

                # ------------------------------
                # 우회전
                # ------------------------------
                right_bias = calibrate_gyro(ser)

                right_angle = rotate_90(
                    ser,
                    pub,
                    right_bias,
                    direction=-1.0,
                    stop_angle=RIGHT_STOP_ANGLE,
                    label=f"#{test_num} 우회전 90도"
                )

                publish_stop(pub)

                residual = left_angle + right_angle

                print()
                print("--------------------------------------")
                print(f"왕복 #{test_num} 결과")
                print(f"좌회전 최종각 : {left_angle:+.2f}°")
                print(f"우회전 최종각 : {right_angle:+.2f}°")
                print(f"IMU 잔여각    : {residual:+.2f}°")
                print("--------------------------------------")

                # 카메라 FINAL 기록 시간 확보
                time.sleep(1.5)

                if test_num < ROUNDTRIP_COUNT:
                    print(
                        f"다음 테스트까지 "
                        f"{INTER_TEST_DELAY:.1f}초 대기..."
                    )
                    time.sleep(INTER_TEST_DELAY)

            print()
            print("======================================")
            print(f"자동 왕복 {ROUNDTRIP_COUNT}회 완료")
            print("======================================")

    except KeyboardInterrupt:
        print()
        print("사용자 중단")

    finally:
        publish_stop(pub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
