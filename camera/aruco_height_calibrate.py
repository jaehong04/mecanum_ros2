import cv2
import numpy as np
from collections import deque
from pathlib import Path
import json

DEVICE = "/dev/v4l/by-id/usb-HD_Web_Camera_HD_Web_Camera_Ucamera001-video-index0"
CALIBRATION_FILE = "calibration_result.json"

# Camera Intrinsic Calibration 불러오기
calib_path = Path(CALIBRATION_FILE)

if not calib_path.exists():
    raise SystemExit(f"{CALIBRATION_FILE} 파일이 없습니다.")

calib = json.loads(calib_path.read_text())

CAMERA_WIDTH = int(calib["image_width"])
CAMERA_HEIGHT = int(calib["image_height"])

camera_matrix = np.array(
    calib["camera_matrix"],
    dtype=np.float64
)

dist_coeffs = np.array(
    calib["distortion_coefficients_k1_k2_p1_p2_k3"],
    dtype=np.float64
)

print(
    f"Intrinsic calibration 적용: "
    f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}"
)

# 기존 60x60cm 바닥 Homography는 화면 기준선 표시용으로만 사용
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

    restored = cv2.perspectiveTransform(
        world_corners,
        inv_H
    ).reshape(-1, 2)

    # homography_60cm.npy는 undistorted pixel 좌표계 기준.
    # 현재 화면은 raw 영상이므로 표시용 경계만 raw pixel로 되돌린다.
    ones = np.ones((len(restored), 1), dtype=np.float64)
    homogeneous = np.hstack([
        restored.astype(np.float64),
        ones
    ])

    normalized = (
        np.linalg.inv(camera_matrix) @ homogeneous.T
    ).T

    normalized_xy = normalized[:, :2] / normalized[:, 2:3]

    object_points = np.column_stack([
        normalized_xy,
        np.ones(len(normalized_xy), dtype=np.float64)
    ])

    raw_restored, _ = cv2.projectPoints(
        object_points,
        np.zeros(3),
        np.zeros(3),
        camera_matrix,
        dist_coeffs
    )

    raw_restored = raw_restored.reshape(-1, 2)

    floor_points = [
        (int(round(x)), int(round(y)))
        for x, y in raw_restored
    ]

    print("기존 60cm 타일 경계 복원:", floor_points)
else:
    print("homography_60cm.npy 없음 - 타일 경계 표시 안 함")

# 실제 로봇(ArUco 중심)을 놓을 위치 [cm]
TARGETS = [
    (10.0, 10.0),
    (30.0, 10.0),
    (50.0, 10.0),
    (10.0, 30.0),
    (30.0, 30.0),
    (50.0, 30.0),
    (10.0, 50.0),
    (30.0, 50.0),
    (50.0, 50.0),
]

image_points = []
world_points = []

# 최근 검출값 중앙값 사용
center_history = deque(maxlen=10)

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("카메라 열기 실패")
    raise SystemExit

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 7
parameters.cornerRefinementMaxIterations = 50
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 53
parameters.adaptiveThreshWinSizeStep = 4
parameters.minMarkerPerimeterRate = 0.015

window = "ArUco Height Homography Calibration"

cv2.namedWindow(window, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window, 1280, 720)

print("====================================")
print("ArUco 높이 기준 Homography Calibration")
print("====================================")
print("c : 현재 위치 저장")
print("z : 마지막 위치 취소")
print("q : 종료")
print()

while True:

    ret, frame = cap.read()

    if not ret:
        print("영상 읽기 실패")
        break

    # 기존 60x60cm 타일 경계 표시
    if len(floor_points) == 4:
        polygon = np.array(floor_points, dtype=np.int32)

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
                4,
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

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters
    )

    detected_center = None

    if ids is not None:

        for marker_corners, marker_id in zip(corners, ids.flatten()):

            if marker_id != 0:
                continue

            pts = marker_corners[0]

            # ArUco 검출은 선명한 원본(raw) 영상에서 수행
            raw_cx = float(pts[:, 0].mean())
            raw_cy = float(pts[:, 1].mean())

            # Homography와 동일한 undistorted pixel 좌표계로 변환
            raw_center = np.array(
                [[[raw_cx, raw_cy]]],
                dtype=np.float32
            )

            undistorted_center = cv2.undistortPoints(
                raw_center,
                camera_matrix,
                dist_coeffs,
                P=camera_matrix
            )[0][0]

            cx = float(undistorted_center[0])
            cy = float(undistorted_center[1])

            detected_center = (cx, cy)
            center_history.append([cx, cy])

            cv2.aruco.drawDetectedMarkers(
                frame,
                [marker_corners],
                np.array([[0]], dtype=np.int32)
            )

            # 화면 표시 위치는 raw 영상의 마커 중심
            cv2.circle(
                frame,
                (int(round(raw_cx)), int(round(raw_cy))),
                4,
                (0, 0, 255),
                -1
            )

            break

    index = len(image_points)

    if index < len(TARGETS):

        tx, ty = TARGETS[index]

        cv2.putText(
            frame,
            f"Target {index + 1}/9 : X={tx:.0f}cm Y={ty:.0f}cm",
            (30, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 255, 255),
            3
        )

        cv2.putText(
            frame,
            "Place ArUco center at target -> press C",
            (30, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

    if detected_center is not None:

        cv2.putText(
            frame,
            f"Undist Pixel: ({detected_center[0]:.1f}, {detected_center[1]:.1f})",
            (30, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

    cv2.imshow(window, frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

    elif key == ord("z"):

        if image_points:
            print(
                "취소:",
                world_points[-1],
                image_points[-1]
            )

            image_points.pop()
            world_points.pop()
            center_history.clear()

    elif key == ord("c"):

        if len(image_points) >= len(TARGETS):
            continue

        if len(center_history) < 5:
            print("검출값이 충분하지 않습니다. 잠시 기다린 후 다시 c를 누르세요.")
            continue

        # 최근 값들의 중앙값
        median_center = np.median(
            np.array(center_history),
            axis=0
        )

        cx = float(median_center[0])
        cy = float(median_center[1])

        target = TARGETS[len(image_points)]

        image_points.append((cx, cy))
        world_points.append(target)

        print(
            f"{len(image_points)}/9 저장 | "
            f"실제=({target[0]:.1f}, {target[1]:.1f}) cm | "
            f"Pixel=({cx:.2f}, {cy:.2f})"
        )

        center_history.clear()

        if len(image_points) == 9:

            src = np.array(
                image_points,
                dtype=np.float32
            )

            dst = np.array(
                world_points,
                dtype=np.float32
            )

            H, mask = cv2.findHomography(
                src,
                dst,
                method=0
            )

            np.save(
                "homography_aruco_height.npy",
                H
            )

            np.save(
                "aruco_calib_image_points.npy",
                src
            )

            np.save(
                "aruco_calib_world_points.npy",
                dst
            )

            predicted = cv2.perspectiveTransform(
                src.reshape(-1, 1, 2),
                H
            ).reshape(-1, 2)

            errors = np.linalg.norm(
                predicted - dst,
                axis=1
            )

            rmse = np.sqrt(
                np.mean(errors ** 2)
            )

            print()
            print("====================================")
            print("Homography 생성 완료")
            print("파일: homography_aruco_height.npy")
            print("------------------------------------")
            print(H)
            print("------------------------------------")
            print(f"Calibration RMSE = {rmse:.3f} cm")
            print("====================================")
            print("q를 눌러 종료하세요.")

cap.release()
cv2.destroyAllWindows()
