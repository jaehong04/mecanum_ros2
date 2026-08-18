#!/bin/bash

CAM="/dev/v4l/by-id/usb-HD_Web_Camera_HD_Web_Camera_Ucamera001-video-index0"

if [ ! -e "$CAM" ]; then
    echo "카메라를 찾을 수 없습니다:"
    echo "$CAM"
    exit 1
fi

v4l2-ctl -d "$CAM" \
  --set-fmt-video=width=1920,height=1080,pixelformat=MJPG \
  --set-parm=30

v4l2-ctl -d "$CAM" \
  --set-ctrl=brightness=95

echo "================================="
echo "Camera setup 완료"
echo "Resolution : 1920x1080"
echo "FPS        : 30"
echo "Format     : MJPG"
echo "Brightness : 95"
echo "================================="
