import json
from pathlib import Path
from collections import deque

import cv2
import numpy as np

DEVICE = "/dev/v4l/by-id/usb-HD_Web_Camera_HD_Web_Camera_Ucamera001-video-index0"
MARKER_LENGTH_M = 0.188

# -----------------------------
# Intrinsic
# -----------------------------
calib = json.loads(
    Path("calibration_result.json").read_text()
)

K = np.array(
    calib["camera_matrix"],
    dtype=np.float64
)

dist = np.array(
    calib["distortion_coefficients_k1_k2_p1_p2_k3"],
    dtype=np.float64
)

width = int(calib["image_width"])
height = int(calib["image_height"])

# -----------------------------
# Camera Extrinsic
# -----------------------------
ext = np.load("camera_extrinsic.npz")

R_camera_to_world = ext["R_camera_to_world"]
camera_position_world = ext["camera_position_world"].reshape(3, 1)

# -----------------------------
# ArUco 실제 3D 모서리
# -----------------------------
L = MARKER_LENGTH_M

marker_points = np.array([
    [-L/2,  L/2, 0],
    [ L/2,  L/2, 0],
    [ L/2, -L/2, 0],
    [-L/2, -L/2, 0]
], dtype=np.float32)

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

parameters = cv2.aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise SystemExit("카메라 열기 실패")

print("======================================")
print("ArUco World Position Test")
print("q / ESC : 종료")
print("======================================")

WINDOW = "ArUco World Position Test"

cv2.namedWindow(
    WINDOW,
    cv2.WINDOW_NORMAL
)

cv2.resizeWindow(
    WINDOW,
    1280,
    720
)

fullscreen = False

# 최근 7프레임의 World Pose 중앙값 사용
FILTER_WINDOW = 7
pose_history = deque(maxlen=FILTER_WINDOW)

while True:
    ok, frame = cap.read()

    if not ok:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters
    )

    if ids is not None and 0 in ids.flatten():

        idx = int(
            np.where(ids.flatten() == 0)[0][0]
        )

        marker_corners = corners[idx]

        solved, rvec, tvec = cv2.solvePnP(
            marker_points,
            marker_corners.reshape(4, 2),
            K,
            dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if solved:

            # ArUco 중심: Camera frame -> World/Floor frame
            p_camera = tvec.reshape(3, 1)

            p_world = (
                R_camera_to_world @ p_camera
                + camera_position_world
            )

            # Raw World 좌표
            x = float(p_world[0, 0])
            y = float(p_world[1, 0])
            z = float(p_world[2, 0])

            # Median Filter
            pose_history.append([x, y, z])

            filtered_pose = np.median(
                np.array(pose_history),
                axis=0
            )

            fx = float(filtered_pose[0])
            fy = float(filtered_pose[1])
            fz = float(filtered_pose[2])

            cv2.aruco.drawDetectedMarkers(
                frame,
                [marker_corners],
                np.array([[0]], dtype=np.int32)
            )

            text = (
                f"Filtered X:{fx*100:.1f}cm "
                f"Y:{fy*100:.1f}cm "
                f"Z:{fz*100:.1f}cm"
            )

            cv2.putText(
                frame,
                text,
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )

            print(
                f"RAW X={x*100:.2f} Y={y*100:.2f} Z={z*100:.2f} cm"
                f"  |  FILTERED "
                f"X={fx*100:.2f} Y={fy*100:.2f} Z={fz*100:.2f} cm",
                flush=True
            )

    cv2.imshow(WINDOW, frame)

    key = cv2.waitKey(1) & 0xFF

    if key in (ord("q"), 27):
        break

    if key in (ord("f"), ord("F")):
        fullscreen = not fullscreen

        if fullscreen:
            cv2.setWindowProperty(
                WINDOW,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN
            )
        else:
            cv2.setWindowProperty(
                WINDOW,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_NORMAL
            )
            cv2.resizeWindow(
                WINDOW,
                1280,
                720
            )

cap.release()
cv2.destroyAllWindows()
print()
