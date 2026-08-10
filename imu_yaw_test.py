#!/usr/bin/env python3

import struct
import time
import serial

PORT = "/dev/ttyUSB1"
BAUD = 115200
ANGLE_SCALE = 180.0 / 32768.0


def angle_difference(current, start):
    return (current - start + 180.0) % 360.0 - 180.0


def read_yaw(ser):
    while True:
        if ser.read(1) != b"\x55":
            continue

        if ser.read(1) != b"\x61":
            continue

        payload = ser.read(18)

        if len(payload) != 18:
            continue

        values = struct.unpack("<9h", payload)
        return values[8] * ANGLE_SCALE


with serial.Serial(PORT, BAUD, timeout=1) as ser:
    ser.reset_input_buffer()

    print("IMU 안정화 중...")
    time.sleep(1)

    start_yaw = read_yaw(ser)

    print(f"시작 Yaw: {start_yaw:.2f}도")
    print("현재 방향을 상대각도 0도로 설정")
    print("종료: Ctrl+C")

    try:
        while True:
            yaw = read_yaw(ser)
            relative_yaw = angle_difference(yaw, start_yaw)

            print(
                f"절대 Yaw: {yaw:8.2f}도 | "
                f"상대 회전각: {relative_yaw:8.2f}도"
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("종료")
