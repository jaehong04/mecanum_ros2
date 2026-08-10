#!/usr/bin/env python3

import struct
import time
import serial

PORT = "/dev/ttyUSB1"
BAUD = 115200
ANGLE_SCALE = 180.0 / 32768.0
TEST_TIME = 60.0


def read_yaw(ser):
    while True:
        if ser.read(1) != b"\x55":
            continue

        if ser.read(1) != b"\x61":
            continue

        payload = ser.read(18)

        if len(payload) == 18:
            values = struct.unpack("<9h", payload)
            return values[8] * ANGLE_SCALE


def angle_diff(current, start):
    return (current - start + 180.0) % 360.0 - 180.0


with serial.Serial(PORT, BAUD, timeout=1) as ser:
    ser.reset_input_buffer()

    print("IMU 안정화 중...")
    time.sleep(3)

    start_yaw = read_yaw(ser)
    start_time = time.time()

    minimum = 0.0
    maximum = 0.0

    print(f"시작 Yaw: {start_yaw:.2f}도")
    print("60초 동안 로봇을 움직이지 마세요.")

    while True:
        elapsed = time.time() - start_time

        if elapsed >= TEST_TIME:
            break

        yaw = read_yaw(ser)
        drift = angle_diff(yaw, start_yaw)

        minimum = min(minimum, drift)
        maximum = max(maximum, drift)

        print(
            f"\r시간: {elapsed:5.1f}초 | "
            f"Yaw: {yaw:8.2f}도 | "
            f"변화: {drift:+7.2f}도",
            end="",
            flush=True
        )

    final_yaw = read_yaw(ser)
    final_drift = angle_diff(final_yaw, start_yaw)

    print()
    print(f"최종 변화: {final_drift:+.2f}도")
    print(f"최솟값: {minimum:+.2f}도")
    print(f"최댓값: {maximum:+.2f}도")
    print(f"전체 변동폭: {maximum - minimum:.2f}도")
