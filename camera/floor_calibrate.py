import cv2
import numpy as np
from collections import deque
from pathlib import Path
import json

DEVICE = "/dev/v4l/by-id/usb-HD_Web_Camera_HD_Web_Camera_Ucamera001-video-index0"
TILE_SIZE_CM = 60.0
CALIBRATION_FILE = "calibration_result.json"

# Camera Intrinsic Calibration
calib = json.loads(Path(CALIBRATION_FILE).read_text())

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

print(f"Intrinsic calibration 적용: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")

HOLD_FRAMES = 15       # 약 0.5초간 마지막 좌표 유지
SMOOTH_ALPHA = 0.25    # 낮을수록 좌표가 부드러움
MEDIAN_WINDOW = 5      # 최근 5회 측정값 사용

clicked_points = []

HOMOGRAPHY_FILE = "homography_60cm.npy"

# 카메라 위치가 변경되었고 Intrinsic 보정 좌표계를 사용하므로
# 기존 Homography는 자동으로 불러오지 않고 새로 지정한다.
homography = None
print("새 바닥 Homography를 생성합니다.")
print("실제 60x60cm 영역의 모서리 1→2→3→4를 다시 클릭하세요.")

last_marker_corners = None
last_center_pixel = None
filtered_world = None
world_history = deque(maxlen=MEDIAN_WINDOW)
missed_frames = HOLD_FRAMES + 1


def mouse_callback(event, x, y, flags, param):
    global homography

    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        print(f"{len(clicked_points)}번 점: ({x}, {y})")

        if len(clicked_points) == 4:
            image_points = np.array(clicked_points, dtype=np.float32)

            world_points = np.array([
                [0.0, 0.0],
                [TILE_SIZE_CM, 0.0],
                [TILE_SIZE_CM, TILE_SIZE_CM],
                [0.0, TILE_SIZE_CM]
            ], dtype=np.float32)

            homography = cv2.getPerspectiveTransform(
                image_points,
                world_points
            )

            print("좌표 변환 완료")
            print("s: 저장 / r: 다시 선택 / q: 종료")


cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("카메라 열기 실패")
    raise SystemExit

undistort_map1, undistort_map2 = cv2.initUndistortRectifyMap(
    camera_matrix,
    dist_coeffs,
    None,
    camera_matrix,
    (CAMERA_WIDTH, CAMERA_HEIGHT),
    cv2.CV_32FC1
)

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 7
parameters.cornerRefinementMaxIterations = 50
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 53
parameters.adaptiveThreshWinSizeStep = 4
parameters.minMarkerPerimeterRate = 0.015

window_name = "Stable 60cm Floor Calibration"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 1280, 720)
cv2.setMouseCallback(window_name, mouse_callback)

print("타일 모서리 클릭 순서")
print("1. 왼쪽 위")
print("2. 오른쪽 위")
print("3. 오른쪽 아래")
print("4. 왼쪽 아래")

while True:
    ret, frame = cap.read()

    if not ret:
        print("영상 읽기 실패")
        break

    # 렌즈 왜곡 보정된 영상 사용
    frame = cv2.remap(
        frame,
        undistort_map1,
        undistort_map2,
        cv2.INTER_LINEAR
    )

    display = frame.copy()

    for index, point in enumerate(clicked_points):
        cv2.circle(display, point, 10, (0, 0, 255), -1)
        cv2.putText(
            display,
            str(index + 1),
            (point[0] + 15, point[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            3
        )

    if len(clicked_points) == 4:
        polygon = np.array(clicked_points, dtype=np.int32)
        cv2.polylines(display, [polygon], True, (0, 255, 0), 4)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters
    )

    detected = False

    if ids is not None:
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if marker_id != 0:
                continue

            points = marker_corners[0]

            center_x = float(points[:, 0].mean())
            center_y = float(points[:, 1].mean())

            last_marker_corners = marker_corners.copy()
            last_center_pixel = (int(center_x), int(center_y))
            missed_frames = 0
            detected = True

            if homography is not None:
                center_pixel = np.array(
                    [[[center_x, center_y]]],
                    dtype=np.float32
                )

                raw_world = cv2.perspectiveTransform(
                    center_pixel,
                    homography
                )[0][0]

                world_history.append(raw_world)

                median_world = np.median(
                    np.array(world_history),
                    axis=0
                )

                if filtered_world is None:
                    filtered_world = median_world
                else:
                    filtered_world = (
                        (1.0 - SMOOTH_ALPHA) * filtered_world
                        + SMOOTH_ALPHA * median_world
                    )

            break

    if not detected:
        missed_frames += 1

    if (
        last_marker_corners is not None
        and missed_frames <= HOLD_FRAMES
    ):
        cv2.aruco.drawDetectedMarkers(
            display,
            [last_marker_corners],
            np.array([[0]], dtype=np.int32)
        )

        cv2.circle(
            display,
            last_center_pixel,
            4,
            (0, 0, 255),
            -1
        )

        status = "LIVE" if detected else "HOLD"
        status_color = (0, 255, 0) if detected else (0, 255, 255)

        if filtered_world is not None:
            world_x = float(filtered_world[0])
            world_y = float(filtered_world[1])

            cv2.putText(
                display,
                f"{status}  X:{world_x:.1f}cm  Y:{world_y:.1f}cm",
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                status_color,
                3
            )
    else:
        cv2.putText(
            display,
            "MARKER LOST",
            (30, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 0, 255),
            3
        )

    cv2.imshow(window_name, display)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("c"):
        if filtered_world is not None:
            world_x = float(filtered_world[0])
            world_y = float(filtered_world[1])
            print(
                f"Pixel=({last_center_pixel[0]}, {last_center_pixel[1]})  "
                f"현재 Homography X={world_x:.2f}cm, Y={world_y:.2f}cm"
            )
        else:
            print("현재 검출된 좌표가 없습니다.")

    if key == ord("q"):
        break

    if key == ord("r"):
        clicked_points.clear()
        homography = None
        filtered_world = None
        world_history.clear()
        print("기준점 초기화")

    if key == ord("s") and homography is not None:
        np.save("homography_60cm.npy", homography)
        print("저장 완료: homography_60cm.npy")

cap.release()
cv2.destroyAllWindows()
