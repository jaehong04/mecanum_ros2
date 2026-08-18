import json
from pathlib import Path

import cv2
import numpy as np

# --------------------------------------------------
# Intrinsic Calibration
# --------------------------------------------------
calib = json.loads(Path("calibration_result.json").read_text())

K = np.array(
    calib["camera_matrix"],
    dtype=np.float64
)

# 바닥 4점은 이미 undistorted 영상에서 선택했으므로
# solvePnP에서는 distortion = 0 으로 사용
dist = np.zeros((5, 1), dtype=np.float64)

# --------------------------------------------------
# 실제 바닥 좌표 [m]
#
# 1 (0,0) ---------- 2 (0.6,0)
#   |                    |
#   |                    |
# 4 (0,0.6) -------- 3 (0.6,0.6)
# --------------------------------------------------
world_points = np.array([
    [0.0, 0.0, 0.0],
    [0.6, 0.0, 0.0],
    [0.6, 0.6, 0.0],
    [0.0, 0.6, 0.0]
], dtype=np.float64)

# Intrinsic 보정 영상에서 방금 클릭한 픽셀 좌표
image_points = np.array([
    [952.0, 283.0],
    [1205.0, 284.0],
    [1231.0, 485.0],
    [951.0, 489.0]
], dtype=np.float64)

# --------------------------------------------------
# World -> Camera 자세 계산
# --------------------------------------------------
success, rvec, tvec = cv2.solvePnP(
    world_points,
    image_points,
    K,
    dist,
    flags=cv2.SOLVEPNP_ITERATIVE
)

if not success:
    raise SystemExit("solvePnP 실패")

R_world_to_camera, _ = cv2.Rodrigues(rvec)

# Camera -> World
R_camera_to_world = R_world_to_camera.T
camera_position_world = -R_camera_to_world @ tvec

T_world_camera = np.eye(4, dtype=np.float64)
T_world_camera[:3, :3] = R_camera_to_world
T_world_camera[:3, 3] = camera_position_world.ravel()

# --------------------------------------------------
# Reprojection 확인
# --------------------------------------------------
projected, _ = cv2.projectPoints(
    world_points,
    rvec,
    tvec,
    K,
    dist
)

projected = projected.reshape(-1, 2)

errors = np.linalg.norm(
    projected - image_points,
    axis=1
)

rmse = np.sqrt(np.mean(errors ** 2))

# --------------------------------------------------
# 저장
# --------------------------------------------------
np.savez(
    "camera_extrinsic.npz",
    rvec=rvec,
    tvec=tvec,
    R_world_to_camera=R_world_to_camera,
    R_camera_to_world=R_camera_to_world,
    camera_position_world=camera_position_world,
    T_world_camera=T_world_camera
)

print("====================================")
print("Camera Extrinsic 계산 완료")
print("====================================")
print()
print("Camera position in floor/world frame [m]")
print(
    f"X = {camera_position_world[0,0]:.3f} m\n"
    f"Y = {camera_position_world[1,0]:.3f} m\n"
    f"Z = {camera_position_world[2,0]:.3f} m"
)
print()
print(f"Reprojection RMSE = {rmse:.3f} px")
print()
print("각 기준점 reprojection error:")
for i, e in enumerate(errors, start=1):
    print(f"Point {i}: {e:.3f} px")

print()
print("저장 완료: camera_extrinsic.npz")
print("====================================")
