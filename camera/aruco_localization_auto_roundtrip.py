import json
import math
import time
from pathlib import Path
from collections import deque

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


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
# 바닥 60x60 영역 표시
# ==================================================
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

    undistorted_points = cv2.perspectiveTransform(
        world_corners,
        inv_H
    ).reshape(-1, 2)

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
        normalized[:, :2]
        / normalized[:, 2:3]
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
# ROS2 /cmd_vel 자동 왕복 측정
# ==================================================
CMD_THRESHOLD = 0.05
SETTLE_TIME = 0.8
POSE_MAX_AGE = 0.5

latest_pose = None
cmd_z = 0.0

round_state = "WAIT_LEFT_START"
stop_since = None

start_pose = None
left_pose = None
final_pose = None

test_count = 0


def cmd_vel_callback(msg):
    global cmd_z
    cmd_z = float(msg.angular.z)


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def pose_string(name, pose):
    x, y, yaw, _ = pose

    return (
        f"{name}: "
        f"X={x*100:.2f} cm | "
        f"Y={y*100:.2f} cm | "
        f"Yaw={yaw:.2f} deg"
    )


rclpy.init()

ros_node = rclpy.create_node(
    "aruco_auto_roundtrip_monitor"
)

cmd_qos = QoSProfile(depth=1)
cmd_qos.reliability = ReliabilityPolicy.BEST_EFFORT

cmd_sub = ros_node.create_subscription(
    Twist,
    "/cmd_vel",
    cmd_vel_callback,
    cmd_qos
)


print("======================================")
print("ArUco Localization : X / Y / Yaw")
print("Marker height = 16.3 cm")
print("--------------------------------------")
print("C : 현재 로봇 방향을 Yaw 0도로 설정")
print("F : 전체화면")
print("Q / ESC : 종료")
print("======================================")


while True:

    # ROS2 /cmd_vel 처리
    rclpy.spin_once(
        ros_node,
        timeout_sec=0.0
    )

    now_auto = time.monotonic()

    pose_valid = (
        latest_pose is not None
        and now_auto - latest_pose[3] <= POSE_MAX_AGE
    )

    # ----------------------------------------------
    # 좌회전 시작 감지
    # ----------------------------------------------
    if (
        round_state == "WAIT_LEFT_START"
        and cmd_z > CMD_THRESHOLD
        and pose_valid
    ):
        start_pose = latest_pose
        test_count += 1

        print()
        print("======================================")
        print(f"AUTO 왕복 측정 #{test_count} 시작")
        print(pose_string("START", start_pose))

        round_state = "LEFT_MOVING"
        stop_since = None

    # ----------------------------------------------
    # 좌회전 정지 감지
    # ----------------------------------------------
    elif round_state == "LEFT_MOVING":

        if abs(cmd_z) <= CMD_THRESHOLD:

            if stop_since is None:
                stop_since = now_auto

            elif (
                now_auto - stop_since >= SETTLE_TIME
                and pose_valid
            ):
                left_pose = latest_pose

                print(pose_string("LEFT ", left_pose))

                left_camera_angle = wrap_deg(
                    left_pose[2] - start_pose[2]
                )

                print(
                    f"Camera 좌회전량: "
                    f"{left_camera_angle:+.2f} deg"
                )

                round_state = "WAIT_RIGHT_START"
                stop_since = None

        else:
            stop_since = None

    # ----------------------------------------------
    # 우회전 시작 감지
    # ----------------------------------------------
    elif (
        round_state == "WAIT_RIGHT_START"
        and cmd_z < -CMD_THRESHOLD
    ):
        print("우회전 명령 감지")

        round_state = "RIGHT_MOVING"
        stop_since = None

    # ----------------------------------------------
    # 우회전 정지 + 최종 결과
    # ----------------------------------------------
    elif round_state == "RIGHT_MOVING":

        if abs(cmd_z) <= CMD_THRESHOLD:

            if stop_since is None:
                stop_since = now_auto

            elif (
                now_auto - stop_since >= SETTLE_TIME
                and pose_valid
            ):
                final_pose = latest_pose

                print(pose_string("FINAL", final_pose))

                left_camera_angle = wrap_deg(
                    left_pose[2] - start_pose[2]
                )

                right_camera_angle = wrap_deg(
                    final_pose[2] - left_pose[2]
                )

                residual_yaw = wrap_deg(
                    final_pose[2] - start_pose[2]
                )

                dx_cm = (
                    final_pose[0] - start_pose[0]
                ) * 100.0

                dy_cm = (
                    final_pose[1] - start_pose[1]
                ) * 100.0

                position_error_cm = math.hypot(
                    dx_cm,
                    dy_cm
                )

                print("--------------------------------------")
                print(
                    f"Camera 좌회전량 : "
                    f"{left_camera_angle:+.2f} deg"
                )
                print(
                    f"Camera 우회전량 : "
                    f"{right_camera_angle:+.2f} deg"
                )
                print(
                    f"왕복 Yaw 잔여각 : "
                    f"{residual_yaw:+.2f} deg"
                )
                print(
                    f"왕복 ΔX         : "
                    f"{dx_cm:+.2f} cm"
                )
                print(
                    f"왕복 ΔY         : "
                    f"{dy_cm:+.2f} cm"
                )
                print(
                    f"왕복 위치 오차  : "
                    f"{position_error_cm:.2f} cm"
                )
                print("======================================")

                print(f"AUTO 왕복 측정 #{test_count} 완료")
                print("다음 좌회전 테스트 대기 중...")
                print()

                round_state = "WAIT_LEFT_START"
                stop_since = None

                start_pose = None
                left_pose = None
                final_pose = None

    ok, frame = cap.read()

    if not ok:
        break

    # ----------------------------------------------
    # 검출용 영상은 overlay 전에 보존
    # ----------------------------------------------
    detect_frame = frame.copy()

    gray = cv2.cvtColor(
        detect_frame,
        cv2.COLOR_BGR2GRAY
    )

    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters
    )

    id0_found = (
        ids is not None
        and 0 in ids.flatten()
    )

    # 기본 영상에서 실패하면 CLAHE 재검출
    if not id0_found:

        gray_clahe = clahe.apply(gray)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray_clahe,
            dictionary,
            parameters=parameters
        )


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

            cv2.putText(
                frame,
                str(i + 1),
                (
                    point[0] + 10,
                    point[1] - 10
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
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
            x = float(center_world[0])
            y = float(center_world[1])

            position_history.append(
                [x, y]
            )

            filtered_xy = np.median(
                np.array(position_history),
                axis=0
            )

            fx = float(filtered_xy[0])
            fy = float(filtered_xy[1])


            # --------------------------------------
            # Yaw
            # --------------------------------------
            direction = (
                top_world[:2]
                - center_world[:2]
            )

            raw_yaw = math.atan2(
                direction[1],
                direction[0]
            )

            last_raw_yaw = raw_yaw

            yaw = wrap_angle(
                -(raw_yaw - yaw_offset)
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

            # 자동 측정용 최신 Pose
            latest_pose = (
                fx,
                fy,
                yaw_deg,
                time.monotonic()
            )


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

            cv2.putText(
                frame,
                text1,
                (30, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                text2,
                (30, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2
            )


            # 실시간 값은 OpenCV 화면에만 표시
            # 터미널 출력은 P 키를 눌렀을 때만 수행


    cv2.imshow(
        WINDOW,
        frame
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
