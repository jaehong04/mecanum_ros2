import json
import math
import time
from pathlib import Path
from collections import deque

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


# ==================================================
# 기본 설정
# ==================================================
DEVICE = "/dev/v4l/by-id/usb-HD_Web_Camera_HD_Web_Camera_Ucamera001-video-index0"

MARKER_HEIGHT_M = 0.163
PLANE_Z = -MARKER_HEIGHT_M

FILTER_WINDOW = 7
PRINT_INTERVAL = 0.1

YAW_OFFSET_FILE = Path("yaw_offset.json")


# ==================================================
# Intrinsic
# ==================================================
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


# ==================================================
# Extrinsic
# ==================================================
ext = np.load("camera_extrinsic.npz")

R_camera_to_world = np.array(
    ext["R_camera_to_world"],
    dtype=np.float64
)

camera_position_world = np.array(
    ext["camera_position_world"],
    dtype=np.float64
).reshape(3)


# ==================================================
# Yaw offset
# ==================================================
yaw_offset = 0.0

if YAW_OFFSET_FILE.exists():
    try:
        data = json.loads(YAW_OFFSET_FILE.read_text())
        yaw_offset = math.radians(
            float(data["yaw_offset_deg"])
        )
        print(
            f"저장된 Yaw offset 로드: "
            f"{math.degrees(yaw_offset):.2f} deg"
        )
    except Exception:
        print("Yaw offset 파일 로드 실패 - 0 deg 사용")


# ==================================================
# Camera World -> ROS map 좌표 변환
# ==================================================
# Camera World:
#   +X = 180도 회전 화면의 왼쪽
#   +Y = 180도 회전 화면의 위쪽
#   +Z = 실제 아래쪽
#
# ROS map:
#   +X = 180도 회전 화면의 오른쪽
#   +Y = 180도 회전 화면의 위쪽
#   +Z = 실제 위쪽
#
# 3D 기준 Y축 180도 회전:
#   X_ros = -X_cam
#   Y_ros =  Y_cam
#   Z_ros = -Z_cam
# ==================================================
def camera_world_to_ros_map_xy(x_cam, y_cam):
    return -float(x_cam), float(y_cam)


def camera_direction_to_ros_map(dx_cam, dy_cam):
    return -float(dx_cam), float(dy_cam)


# ==================================================
# Nav2 실제 주행 가능 영역 표시
# ==================================================
# 최종 직사각형 drive area
#
# X: -0.607456 ~ +0.585724 m
# Y: -0.645901 ~ +1.701045 m
#
# 이 값은 drive_area 측정값의 서로 마주보는 변을 평균하여
# 만든 최종 Nav2 직사각형 주행 가능 영역과 동일하다.
# ==================================================

DRIVE_MIN_X = -0.607456
DRIVE_MAX_X =  0.585724
DRIVE_MIN_Y = -0.645901
DRIVE_MAX_Y =  1.701045

# 바닥 평면이므로 Z = 0
drive_world_corners = np.array([
    [DRIVE_MIN_X, DRIVE_MIN_Y, 0.0],
    [DRIVE_MAX_X, DRIVE_MIN_Y, 0.0],
    [DRIVE_MAX_X, DRIVE_MAX_Y, 0.0],
    [DRIVE_MIN_X, DRIVE_MAX_Y, 0.0],
], dtype=np.float64)

# World -> Camera
R_world_to_camera = R_camera_to_world.T

camera_points = np.array([
    R_world_to_camera @ (
        world_point - camera_position_world
    )
    for world_point in drive_world_corners
], dtype=np.float64)

# Camera 좌표 -> 원본 1920x1080 이미지 픽셀
raw_points, _ = cv2.projectPoints(
    camera_points.reshape(-1, 1, 3),
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

print("Nav2 drive area pixel points:", floor_points)

# ==================================================
# Pixel -> World plane
# ==================================================
def pixel_to_world(u, v):

    pixel = np.array(
        [[[u, v]]],
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

    ray_world = (
        R_camera_to_world
        @ ray_camera
    )

    if abs(ray_world[2]) < 1e-9:
        return None

    t = (
        PLANE_Z
        - camera_position_world[2]
    ) / ray_world[2]

    if t <= 0:
        return None

    return (
        camera_position_world
        + t * ray_world
    )


def wrap_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


# ==================================================
# ArUco
# ==================================================
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

parameters = cv2.aruco.DetectorParameters_create()

parameters.cornerRefinementMethod = (
    cv2.aruco.CORNER_REFINE_SUBPIX
)

parameters.cornerRefinementWinSize = 7
parameters.cornerRefinementMaxIterations = 50

parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 53
parameters.adaptiveThreshWinSizeStep = 4

parameters.minMarkerPerimeterRate = 0.015

clahe = cv2.createCLAHE(
    clipLimit=2.0,
    tileGridSize=(8, 8)
)


# ==================================================
# Filter
# ==================================================
position_history = deque(
    maxlen=FILTER_WINDOW
)

yaw_history = deque(
    maxlen=FILTER_WINDOW
)


# ==================================================
# Camera
# ==================================================
cap = cv2.VideoCapture(
    DEVICE,
    cv2.CAP_V4L2
)

cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*"MJPG")
)

cap.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    width
)

cap.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    height
)

cap.set(
    cv2.CAP_PROP_FPS,
    30
)

if not cap.isOpened():
    raise SystemExit("카메라 열기 실패")


WINDOW = "ArUco Localization X Y Yaw"

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
last_print_time = 0.0
last_raw_yaw = None

# ==================================================
# ROS2 External Camera Localization
# ==================================================
rclpy.init()

ros_node = rclpy.create_node(
    "external_camera_localization"
)

pose_pub = ros_node.create_publisher(
    PoseWithCovarianceStamped,
    "/external_camera/pose",
    10
)

# 초기 covariance
# X/Y 약 2 cm, Yaw 약 3 deg 기준으로 보수적으로 설정
POSITION_STD_M = 0.02
YAW_STD_RAD = math.radians(3.0)


print("======================================")
print("ArUco Localization : X / Y / Yaw")
print("Marker height = 16.3 cm")
print("--------------------------------------")
print("C : 현재 로봇 방향을 Yaw 0도로 설정")
print("F : 전체화면")
print("Q / ESC : 종료")
print("======================================")


while True:

    ok, frame = cap.read()

    if not ok:
        break

    text1 = None
    text2 = None

    # ----------------------------------------------
    # 검출용 영상은 overlay 전에 보존
    # ----------------------------------------------
    detect_frame = frame.copy()

    # ----------------------------------------------
    # ArUco detection ROI
    # 바닥 캘리브레이션 영역 주변만 검출하여 처리 속도 향상
    # ----------------------------------------------
    ROI_X1, ROI_Y1 = 509, 34
    ROI_X2, ROI_Y2 = 1382, 1080

    detect_roi = detect_frame[
        ROI_Y1:ROI_Y2,
        ROI_X1:ROI_X2
    ]

    gray = cv2.cvtColor(
        detect_roi,
        cv2.COLOR_BGR2GRAY
    )

    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters
    )

    # ROI 좌표 → 원본 1920x1080 영상 좌표로 복원
    if ids is not None:
        for marker_corner in corners:
            marker_corner[:, :, 0] += ROI_X1
            marker_corner[:, :, 1] += ROI_Y1

    id0_found = (
        ids is not None
        and 0 in ids.flatten()
    )

    # CLAHE 재검출 비활성화
    # 현재 환경 테스트에서 재검출 성공 0회로 확인되어
    # 불필요한 연산 부하를 줄이기 위해 원본 영상 검출만 사용


    # ----------------------------------------------
    # 바닥 1~4 표시
    # ----------------------------------------------
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

        for i, point in enumerate(
            floor_points
        ):

            cv2.circle(
                frame,
                point,
                6,
                (0, 0, 255),
                -1
            )



    # ----------------------------------------------
    # ID 0 Localization
    # ----------------------------------------------
    if (
        ids is not None
        and 0 in ids.flatten()
    ):

        idx = int(
            np.where(
                ids.flatten() == 0
            )[0][0]
        )

        marker_corners = corners[idx]

        pts = marker_corners[0]

        # 마커 중심
        center_pixel = np.mean(
            pts,
            axis=0
        )

        # 마커 canonical top edge 중심
        top_mid_pixel = (
            pts[0] + pts[1]
        ) / 2.0

        center_world = pixel_to_world(
            float(center_pixel[0]),
            float(center_pixel[1])
        )

        top_world = pixel_to_world(
            float(top_mid_pixel[0]),
            float(top_mid_pixel[1])
        )

        if (
            center_world is not None
            and top_world is not None
        ):

            # --------------------------------------
            # X / Y
            # --------------------------------------
            # Camera World 좌표
            x_cam = float(center_world[0])
            y_cam = float(center_world[1])

            position_history.append(
                [x_cam, y_cam]
            )

            filtered_xy = np.median(
                np.array(position_history),
                axis=0
            )

            filtered_cam_x = float(filtered_xy[0])
            filtered_cam_y = float(filtered_xy[1])

            # Camera World -> ROS map
            fx, fy = camera_world_to_ros_map_xy(
                filtered_cam_x,
                filtered_cam_y
            )


            # --------------------------------------
            # Yaw
            # --------------------------------------
            # Camera World 방향 벡터
            direction_cam = (
                top_world[:2]
                - center_world[:2]
            )

            # Camera World -> ROS map 방향 벡터
            dx_map, dy_map = camera_direction_to_ros_map(
                direction_cam[0],
                direction_cam[1]
            )

            # ROS map 기준 raw yaw
            raw_yaw = math.atan2(
                dy_map,
                dx_map
            )

            last_raw_yaw = raw_yaw

            # ROS 표준:
            # +Yaw = CCW
            # Yaw 0 = map +X
            yaw = wrap_angle(
                raw_yaw - yaw_offset
            )

            yaw_history.append(yaw)

            sin_mean = np.mean([
                math.sin(a)
                for a in yaw_history
            ])

            cos_mean = np.mean([
                math.cos(a)
                for a in yaw_history
            ])

            filtered_yaw = math.atan2(
                sin_mean,
                cos_mean
            )

            yaw_deg = math.degrees(
                filtered_yaw
            )

            # --------------------------------------
            # ROS2 PoseWithCovarianceStamped Publish
            # --------------------------------------
            pose_msg = PoseWithCovarianceStamped()

            pose_msg.header.stamp = (
                ros_node.get_clock().now().to_msg()
            )

            pose_msg.header.frame_id = "map"

            pose_msg.pose.pose.position.x = fx
            pose_msg.pose.pose.position.y = fy
            pose_msg.pose.pose.position.z = 0.0

            pose_msg.pose.pose.orientation.x = 0.0
            pose_msg.pose.pose.orientation.y = 0.0
            pose_msg.pose.pose.orientation.z = math.sin(
                filtered_yaw / 2.0
            )
            pose_msg.pose.pose.orientation.w = math.cos(
                filtered_yaw / 2.0
            )

            covariance = [0.0] * 36

            covariance[0] = POSITION_STD_M ** 2
            covariance[7] = POSITION_STD_M ** 2

            covariance[14] = 999.0
            covariance[21] = 999.0
            covariance[28] = 999.0

            covariance[35] = YAW_STD_RAD ** 2

            pose_msg.pose.covariance = covariance

            pose_pub.publish(pose_msg)


            # --------------------------------------
            # 화면 표시
            # --------------------------------------
            cv2.aruco.drawDetectedMarkers(
                frame,
                [marker_corners],
                np.array(
                    [[0]],
                    dtype=np.int32
                )
            )

            center_draw = (
                int(round(center_pixel[0])),
                int(round(center_pixel[1]))
            )

            top_draw = (
                int(round(top_mid_pixel[0])),
                int(round(top_mid_pixel[1]))
            )

            cv2.circle(
                frame,
                center_draw,
                5,
                (0, 0, 255),
                -1
            )

            cv2.arrowedLine(
                frame,
                center_draw,
                top_draw,
                (255, 0, 255),
                3,
                tipLength=0.25
            )

            text1 = (
                f"X:{fx*100:.2f}cm "
                f"Y:{fy*100:.2f}cm"
            )

            text2 = (
                f"Yaw:{yaw_deg:.2f} deg "
                f"({filtered_yaw:.3f} rad)"
            )



            # 실시간 값은 OpenCV 화면에만 표시
            # 터미널 출력은 P 키를 눌렀을 때만 수행


    # ----------------------------------------------
    # 화면 표시 방향: 180도 회전
    # 위치 계산 / ROS 좌표계에는 영향 없음
    # ----------------------------------------------
    display = cv2.flip(frame, -1)

    # --------------------------------------------------
    # 바닥 기준점 1~4 번호
    #
    # polygon / 빨간 점은 원본 frame에 그린 뒤 영상과 함께
    # 180도 회전되므로 위치는 이미 맞는다.
    #
    # 숫자만 회전 후 display 위에 다시 그려서
    # 글자가 거꾸로 보이지 않도록 한다.
    #
    # 번호 자체는 기존 calibration point ID를 그대로 유지한다.
    # --------------------------------------------------
    if 'floor_points' in locals() and len(floor_points) == 4:

        display_h, display_w = display.shape[:2]

        for i, point in enumerate(floor_points):

            display_point = (
                display_w - 1 - int(point[0]),
                display_h - 1 - int(point[1])
            )

            cv2.putText(
                display,
                str(i + 1),
                (
                    display_point[0] + 10,
                    display_point[1] - 10
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

    if text1 is not None:
        cv2.putText(
            display,
            text1,
            (30, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

    if text2 is not None:
        cv2.putText(
            display,
            text2,
            (30, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2
        )

    cv2.imshow(
        WINDOW,
        display
    )

    key = cv2.waitKey(1) & 0xFF

    # ----------------------------------------------
    # 현재 X / Y / Yaw 터미널 출력
    # ----------------------------------------------
    if key in (
        ord("p"),
        ord("P")
    ):
        if 'fx' in locals() and 'fy' in locals() and 'yaw_deg' in locals():
            print(
                f"X={fx*100:.2f} cm | "
                f"Y={fy*100:.2f} cm | "
                f"Yaw={yaw_deg:.2f} deg | "
                f"{filtered_yaw:.3f} rad"
            )
        else:
            print("ArUco가 검출된 후 P를 눌러주세요.")


    # ----------------------------------------------
    # 종료
    # ----------------------------------------------
    if key in (
        ord("q"),
        27
    ):
        break


    # ----------------------------------------------
    # 전체화면
    # ----------------------------------------------
    if key in (
        ord("f"),
        ord("F")
    ):

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


    # ----------------------------------------------
    # 현재 방향을 Yaw = 0도로 저장
    # ----------------------------------------------
    if key in (
        ord("c"),
        ord("C")
    ):

        if last_raw_yaw is not None:

            yaw_offset = last_raw_yaw

            YAW_OFFSET_FILE.write_text(
                json.dumps(
                    {
                        "yaw_offset_deg":
                            math.degrees(
                                yaw_offset
                            )
                    },
                    indent=2
                )
            )

            yaw_history.clear()

            print()
            print(
                "Yaw 0도 설정 완료: "
                f"{math.degrees(yaw_offset):.2f} deg"
            )
            print(
                "저장:",
                YAW_OFFSET_FILE
            )

        else:

            print(
                "ArUco가 검출된 후 "
                "C를 눌러주세요."
            )


cap.release()
cv2.destroyAllWindows()

ros_node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()
