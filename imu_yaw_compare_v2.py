#!/usr/bin/env python3

import struct
import time
import serial

PORT = "/dev/ttyUSB1"
BAUD = 115200

GYRO_SCALE = 2000.0 / 32768.0
ANGLE_SCALE = 180.0 / 32768.0

BIAS_TIME = 3.0
START_THRESHOLD = 5.0
START_CONFIRM_SAMPLES = 3
STOP_THRESHOLD = 2.0
STOP_HOLD_TIME = 0.5


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

    print("자이로 바이어스 측정 중... 3초 동안 움직이지 마세요.")

    gyro_samples = []
    last_yaw = 0.0
    calibration_end = time.monotonic() + BIAS_TIME

    while time.monotonic() < calibration_end:
        gz, yaw = read_data(ser)
        gyro_samples.append(gz)
        last_yaw = yaw

    if not gyro_samples:
        raise RuntimeError("IMU 데이터를 받지 못했습니다.")

    gyro_bias = sum(gyro_samples) / len(gyro_samples)

    print(f"바이어스 샘플 수: {len(gyro_samples)}")
    print(f"자이로 바이어스: {gyro_bias:+.5f}도/s")
    print("회전 대기 중...")
    print("별도 터미널에서 90도 회전 코드를 실행하세요.")

    state = "waiting"

    last_stationary_yaw = last_yaw
    start_candidates = []

    integrated_angle = 0.0
    motion_start_yaw = None
    motion_start_time = None

    previous_time = None
    previous_gz = None

    stop_start_time = None
    last_print = time.monotonic()
    maximum_gz = 0.0
    motion_samples = 0

    try:
        while True:
            raw_gz, yaw = read_data(ser)
            current_time = time.monotonic()
            corrected_gz = raw_gz - gyro_bias

            if state == "waiting":
                if abs(corrected_gz) >= START_THRESHOLD:
                    start_candidates.append(
                        (current_time, corrected_gz, yaw)
                    )

                    if len(start_candidates) >= START_CONFIRM_SAMPLES:
                        state = "moving"

                        motion_start_yaw = last_stationary_yaw
                        motion_start_time = start_candidates[0][0]

                        integrated_angle = 0.0

                        previous_time = start_candidates[0][0]
                        previous_gz = start_candidates[0][1]

                        maximum_gz = abs(previous_gz)
                        motion_samples = 1

                        for sample_time, sample_gz, _ in start_candidates[1:]:
                            dt = sample_time - previous_time

                            if 0.0 < dt < 0.2:
                                integrated_angle += (
                                    previous_gz + sample_gz
                                ) * 0.5 * dt

                            previous_time = sample_time
                            previous_gz = sample_gz
                            maximum_gz = max(
                                maximum_gz, abs(sample_gz)
                            )
                            motion_samples += 1

                        print("\n회전 시작 감지")

                else:
                    start_candidates.clear()
                    last_stationary_yaw = yaw

                continue

            dt = current_time - previous_time

            if 0.0 < dt < 0.2:
                integrated_angle += (
                    previous_gz + corrected_gz
                ) * 0.5 * dt

            previous_time = current_time
            previous_gz = corrected_gz

            maximum_gz = max(maximum_gz, abs(corrected_gz))
            motion_samples += 1

            relative_yaw = wrap_angle(yaw - motion_start_yaw)

            if abs(corrected_gz) <= STOP_THRESHOLD:
                if stop_start_time is None:
                    stop_start_time = current_time

                elif current_time - stop_start_time >= STOP_HOLD_TIME:
                    motion_duration = current_time - motion_start_time

                    print("\n\n회전 정지 감지")
                    print(f"절대 Yaw 기준: {relative_yaw:+.2f}도")
                    print(f"자이로 누적각: {integrated_angle:+.2f}도")
                    print(
                        f"Yaw-자이로 차이: "
                        f"{relative_yaw - integrated_angle:+.2f}도"
                    )
                    print(f"회전 측정 시간: {motion_duration:.3f}초")
                    print(f"최대 Z 각속도: {maximum_gz:.2f}도/s")
                    print(f"회전 중 샘플 수: {motion_samples}")
                    break

            else:
                stop_start_time = None

            if current_time - last_print >= 0.1:
                line = (
                    f"\r절대 Yaw: {relative_yaw:+8.2f}도 | "
                    f"자이로 누적: {integrated_angle:+8.2f}도 | "
                    f"Z 각속도: {corrected_gz:+8.2f}도/s"
                )

                print(line.ljust(90), end="", flush=True)
                last_print = current_time

    except KeyboardInterrupt:
        print("\n사용자 종료")
