import cv2
import math

DEVICE = "/dev/v4l/by-id/usb-HD_Web_Camera_HD_Web_Camera_Ucamera001-video-index0"
HOLD_FRAMES = 5       # 최대 5프레임 동안 마지막 검출값 유지
SMOOTH_ALPHA = 0.3    # 위치 흔들림 완화

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("카메라 열기 실패")
    raise SystemExit

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# 작은 마커와 기울어진 마커 검출 개선
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 53
parameters.adaptiveThreshWinSizeStep = 4
parameters.minMarkerPerimeterRate = 0.02

window_name = "ArUco Stable Detection"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 960, 540)

last_center = None
last_angle = None
last_corners = None
missed_frames = HOLD_FRAMES + 1

print("안정화 검출 시작 - 종료하려면 q")

while True:
    ret, frame = cap.read()

    if not ret:
        print("영상 읽기 실패")
        break

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

            top_x = float((points[0][0] + points[1][0]) / 2)
            top_y = float((points[0][1] + points[1][1]) / 2)

            angle = math.degrees(
                math.atan2(center_y - top_y, top_x - center_x)
            )

            if last_center is not None:
                center_x = (
                    SMOOTH_ALPHA * center_x
                    + (1 - SMOOTH_ALPHA) * last_center[0]
                )
                center_y = (
                    SMOOTH_ALPHA * center_y
                    + (1 - SMOOTH_ALPHA) * last_center[1]
                )

            last_center = (center_x, center_y)
            last_angle = angle
            last_corners = marker_corners
            missed_frames = 0
            detected = True
            break

    if not detected:
        missed_frames += 1

    if last_center is not None and missed_frames <= HOLD_FRAMES:
        center_x = int(last_center[0])
        center_y = int(last_center[1])

        cv2.aruco.drawDetectedMarkers(
            frame,
            [last_corners],
            ids=None
        )

        length = 80
        rad = math.radians(last_angle)

        arrow_x = int(center_x + length * math.cos(rad))
        arrow_y = int(center_y - length * math.sin(rad))

        cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
        cv2.arrowedLine(
            frame,
            (center_x, center_y),
            (arrow_x, arrow_y),
            (255, 0, 0),
            5,
            tipLength=0.25
        )

        text = (
            f"ID:0  Center:({center_x},{center_y})  "
            f"Angle:{last_angle:.1f}"
        )

        cv2.putText(
            frame,
            text,
            (center_x + 20, center_y - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    cv2.imshow(window_name, frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
