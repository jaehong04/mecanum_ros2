import json
from pathlib import Path
from collections import deque

import cv2
import numpy as np

DEVICE = "/dev/v4l/by-id/usb-HD_Web_Camera_HD_Web_Camera_Ucamera001-video-index0"

# 바닥에서 ArUco 평면까지 실제 높이
MARKER_HEIGHT_M = 0.163

# 현재 World 좌표계에서는 위쪽이 -Z
PLANE_Z = -MARKER_HEIGHT_M

FILTER_WINDOW = 7
position_history = deque(maxlen=FILTER_WINDOW)

# --------------------------------------------------
# Intrinsic
# --------------------------------------------------
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

# --------------------------------------------------
# Extrinsic
# --------------------------------------------------
ext = np.load("camera_extrinsic.npz")

R_camera_to_world = ext["R_camera_to_world"]
camera_position_world = (
    ext["camera_position_world"].reshape(3)
)

# --------------------------------------------------
# 60x60cm 바닥 기준 영역 표시
# --------------------------------------------------
FLOOR_H_FILE = "homography_60cm.npy"
TILE_SIZE_CM = 60.0

floor_points = []

if Path(FLOOR_H_FILE).exists():

    floor_H = np.load(FLOOR_H_FILE)

    world_corners = np.array([
        [[0.0, 0.0]],
        [[TILE_SIZE_CM, 0.0]],
        [[TILE_SIZE_CM, TILE_SIZE_CM]],
        [[0.0, TILE_SIZE_CM]]
    ], dtype=np.float32)

    inv_H = np.linalg.inv(floor_H)

    # Homography 기준은 undistorted pixel
    undistorted_points = cv2.perspectiveTransform(
        world_corners,
        inv_H
    ).reshape(-1, 2)

    # 현재 화면은 raw 영상이므로 raw pixel로 다시 변환
    ones = np.ones(
        (len(undistorted_points), 1),
        dtype=np.float64
    )

    homogeneous = np.hstack([
        undistorted_points.astype(np.float64),
        ones
    ])

    normalized = (
        np.linalg.inv(K) @ homogeneous.T
    ).T

    normalized_xy = (
        normalized[:, :2] /
        normalized[:, 2:3]
    )

    object_points = np.column_stack([
        normalized_xy,
        np.ones(len(normalized_xy))
    ])

    raw_points, _ = cv2.projectPoints(
        object_points,
        np.zeros(3),
        np.zeros(3),
        K,
        dist
    )

    raw_points = raw_points.reshape(-1, 2)

    floor_points = [
        (int(round(x)), int(round(y)))
        for x, y in raw_points
    ]

    print("바닥 기준점 1~4:", floor_points)

else:
    print("homography_60cm.npy 없음")

# --------------------------------------------------
# ArUco
# --------------------------------------------------
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

parameters = cv2.aruco.DetectorParameters_create()

parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 7
parameters.cornerRefinementMaxIterations = 50

parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 53
parameters.adaptiveThreshWinSizeStep = 4

parameters.minMarkerPerimeterRate = 0.015

# 조명 변화 대응용 CLAHE
clahe = cv2.createCLAHE(
    clipLimit=2.0,
    tileGridSize=(8, 8)
)

# --------------------------------------------------
# Camera
# --------------------------------------------------
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)

cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*"MJPG")
)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise SystemExit("카메라 열기 실패")

WINDOW = "ArUco Ray Plane Position"

cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW, 1280, 720)

fullscreen = False

print("======================================")
print("ArUco Ray-Plane World Position")
print(f"Marker height = {MARKER_HEIGHT_M*100:.1f} cm")
print("F : 전체화면")
print("Q / ESC : 종료")
print("======================================")

while True:

    ok, frame = cap.read()

    if not ok:
        break

    # ArUco 검출용 원본 영상 보존
    detect_frame = frame.copy()

    # 60x60cm 바닥 기준 영역 표시
    if len(floor_points) == 4:

        polygon = np.array(
            floor_points,
            dtype=np.int32
        )

        cv2.polylines(
            frame,
            [polygon],
            True,
            (0, 255, 0),
            3
        )

        for i, point in enumerate(floor_points):

            cv2.circle(
                frame,
                point,
                6,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                frame,
                str(i + 1),
                (point[0] + 10, point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

    # ArUco 검출은 초록 경계선이 그려지지 않은
    # 깨끗한 원본 영상에서 수행
    gray = cv2.cvtColor(
        detect_frame,
        cv2.COLOR_BGR2GRAY
    )

    # 1차: 원본 grayscale에서 검출
    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters
    )

    # 2차: ID 0을 못 찾으면 CLAHE 적용 후 다시 검출
    id0_found = (
        ids is not None
        and 0 in ids.flatten()
    )

    if not id0_found:
        gray_clahe = clahe.apply(gray)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray_clahe,
            dictionary,
            parameters=parameters
        )

    if ids is not None and 0 in ids.flatten():

        idx = int(
            np.where(ids.flatten() == 0)[0][0]
        )

        marker_corners = corners[idx]

        pts = marker_corners[0]

        raw_cx = float(pts[:, 0].mean())
        raw_cy = float(pts[:, 1].mean())

        # ------------------------------------------
        # 픽셀 → 왜곡 보정된 카메라 광선
        # ------------------------------------------
        pixel = np.array(
            [[[raw_cx, raw_cy]]],
            dtype=np.float64
        )

        normalized = cv2.undistortPoints(
            pixel,
            K,
            dist
        )[0][0]

        ray_camera = np.array([
            normalized[0],
            normalized[1],
            1.0
        ])

        # Camera frame → World frame
        ray_world = (
            R_camera_to_world @ ray_camera
        )

        camera_z = camera_position_world[2]
        ray_z = ray_world[2]

        if abs(ray_z) > 1e-9:

            # --------------------------------------
            # 광선과 ArUco 높이 평면의 교차점
            # --------------------------------------
            t = (
                PLANE_Z - camera_z
            ) / ray_z

            if t > 0:

                p_world = (
                    camera_position_world
                    + t * ray_world
                )

                x = float(p_world[0])
                y = float(p_world[1])

                position_history.append([x, y])

                filtered = np.median(
                    np.array(position_history),
                    axis=0
                )

                fx = float(filtered[0])
                fy = float(filtered[1])

                cv2.aruco.drawDetectedMarkers(
                    frame,
                    [marker_corners],
                    np.array([[0]], dtype=np.int32)
                )

                cv2.circle(
                    frame,
                    (
                        int(round(raw_cx)),
                        int(round(raw_cy))
                    ),
                    5,
                    (0, 0, 255),
                    -1
                )

                text = (
                    f"X:{fx*100:.1f}cm "
                    f"Y:{fy*100:.1f}cm "
                    f"H:{MARKER_HEIGHT_M*100:.1f}cm"
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
                    f"RAW X={x*100:.2f} Y={y*100:.2f} cm"
                    f" | FILTERED "
                    f"X={fx*100:.2f} Y={fy*100:.2f} cm"
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
