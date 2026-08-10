#!/usr/bin/env python3

import struct
import time
import serial

PORT = "/dev/ttyUSB1"
BAUD = 115200

GYRO_SCALE = 2000.0 / 32768.0
ANGLE_SCALE = 180.0 / 32768.0


def wrap_angle(angle):
    return (angle + 180.0) % 360.0 - 180.0


def read_data(ser):
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


with serial.Serial(PORT, BAUD, timeout=1) as ser:
    ser.reset_input_buffer()

    print("자이로 바이어스 측정 중... 로봇을 움직이지 마세요.")

    gyro_samples = []
    start_yaw = None

    for _ in range(30):
        gz, yaw = read_data(ser)
        gyro_samples.append(gz)
        start_yaw = yaw

    gyro_bias = sum(gyro_samples) / len(gyro_samples)

    print(f"자이로 바이어스: {gyro_bias:.3f}도/s")
    print(f"시작 Yaw: {start_yaw:.2f}도")
    print("로봇을 정확히 90도 회전하세요.")
    print("종료: Ctrl+C")

    integrated_angle = 0.0
    previous_time = time.monotonic()
    last_print = previous_time

    try:
        while True:
            gz, yaw = read_data(ser)

            current_time = time.monotonic()
            dt = current_time - previous_time
            previous_time = current_time

            corrected_gz = gz - gyro_bias
            integrated_angle += corrected_gz * dt

            if current_time - last_print >= 0.1:
                relative_yaw = wrap_angle(yaw - start_yaw)

                print(
                    f"\r절대 Yaw 기준: {relative_yaw:+8.2f}도 | "
                    f"자이로 누적각: {integrated_angle:+8.2f}도 | "
                    f"Z 각속도: {corrected_gz:+8.2f}도/s",
                    end="",
                    flush=True
                )

                last_print = current_time

    except KeyboardInterrupt:
        print("\n종료")
