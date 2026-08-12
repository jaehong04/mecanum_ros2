#!/usr/bin/env python3

import struct
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

ACC_SCALE = 16.0 / 32768.0
GYRO_SCALE = 2000.0 / 32768.0
ANGLE_SCALE = 180.0 / 32768.0


def read_packet(ser):
    while True:
        first = ser.read(1)

        if first != b"\x55":
            continue

        second = ser.read(1)

        if second != b"\x61":
            continue

        payload = ser.read(18)

        if len(payload) == 18:
            return payload


def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        ser.reset_input_buffer()

        print(f"IMU 연결: {PORT}, {BAUD}bps")
        print("종료: Ctrl+C")

        while True:
            payload = read_packet(ser)

            raw = struct.unpack("<9h", payload)

            ax, ay, az = [v * ACC_SCALE for v in raw[0:3]]
            gx, gy, gz = [v * GYRO_SCALE for v in raw[3:6]]
            roll, pitch, yaw = [v * ANGLE_SCALE for v in raw[6:9]]

            print(
                f"\r"
                f"가속도[g] X:{ax:7.3f} Y:{ay:7.3f} Z:{az:7.3f} | "
                f"각속도[°/s] X:{gx:7.2f} Y:{gy:7.2f} Z:{gz:7.2f} | "
                f"각도[°] Roll:{roll:7.2f} Pitch:{pitch:7.2f} Yaw:{yaw:7.2f}",
                end="",
                flush=True,
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n종료")
    except serial.SerialException as error:
        print(f"\n시리얼 오류: {error}")
