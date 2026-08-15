#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node('right_90deg_test')

publisher = node.create_publisher(
    Twist,
    '/mecanum_drive_controller/reference_unstamped',
    10
)

state = {
    'received': False,
    'x': 0.0,
    'y': 0.0,
    'yaw': 0.0,
}

def odom_callback(msg):
    state['x'] = msg.pose.pose.position.x
    state['y'] = msg.pose.pose.position.y

    q = msg.pose.pose.orientation

    state['yaw'] = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

    state['received'] = True


subscription = node.create_subscription(
    Odometry,
    '/mecanum_drive_controller/odometry',
    odom_callback,
    10
)

print('오도메트리 수신 대기...')

wait_start = time.time()

while rclpy.ok() and not state['received']:
    rclpy.spin_once(node, timeout_sec=0.1)

    if time.time() - wait_start > 5.0:
        print('오도메트리 수신 실패')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

start_x = state['x']
start_y = state['y']
previous_yaw = state['yaw']

accumulated_angle = 0.0

target = math.radians(90.0)
slow_start = math.radians(70.0)
timeout = 8.0

fast = Twist()
fast.angular.z = -0.80

slow = Twist()
slow.angular.z = -0.40

stop = Twist()

print('우회전 90도 시작')

start_time = time.time()
last_print = 0.0
reason = '시간 초과'

while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.01)

    current_yaw = state['yaw']

    delta_yaw = math.atan2(
        math.sin(current_yaw - previous_yaw),
        math.cos(current_yaw - previous_yaw)
    )

    accumulated_angle += delta_yaw
    previous_yaw = current_yaw

    if accumulated_angle <= -target:
        reason = '목표 도달'
        break

    if time.time() - start_time >= timeout:
        break

    if abs(accumulated_angle) < slow_start:
        command = fast
        speed_text = '-0.80rad/s'
    else:
        command = slow
        speed_text = '-0.40rad/s'

    publisher.publish(command)

    if time.time() - last_print >= 0.2:
        print(
            f'\r우회전 각도: '
            f'{math.degrees(accumulated_angle):.2f}도 '
            f'속도: {speed_text}',
            end='',
            flush=True
        )
        last_print = time.time()

for _ in range(10):
    publisher.publish(stop)
    rclpy.spin_once(node, timeout_sec=0.02)

final_x = state['x'] - start_x
final_y = state['y'] - start_y

print(
    f'\n최종 오도메트리 회전각: '
    f'{math.degrees(accumulated_angle):.2f}도'
)
print(
    f'회전 중 위치 변화: '
    f'{math.hypot(final_x, final_y) * 100:.2f}cm'
)
print(f'종료 원인: {reason}')
print('정지 완료')

node.destroy_node()
rclpy.shutdown()
