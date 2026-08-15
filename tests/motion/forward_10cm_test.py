import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


rclpy.init()
node = rclpy.create_node('forward_10cm_test')

pub = node.create_publisher(
    Twist,
    '/cmd_vel',
    10
)

state = {'x': None, 'y': None, 'yaw': None}


def odom_callback(msg):
    state['x'] = msg.pose.pose.position.x
    state['y'] = msg.pose.pose.position.y

    q = msg.pose.pose.orientation
    state['yaw'] = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


node.create_subscription(
    Odometry,
    '/mecanum_drive_controller/odometry',
    odom_callback,
    10
)

fast = Twist()
fast.linear.x = 0.10

slow = Twist()
slow.linear.x = 0.07

stop = Twist()

print('오도메트리 수신 대기...')

wait_start = time.monotonic()

while state['x'] is None:
    rclpy.spin_once(node, timeout_sec=0.05)

    if time.monotonic() - wait_start > 5.0:
        print('오류: 오도메트리 수신 실패')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

start_x = state['x']
start_y = state['y']
start_yaw = state['yaw']

target = 0.101
slow_start = 0.080
run_start = time.monotonic()
timed_out = False

print('10cm 전진 시작')

while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.005)

    dx = state['x'] - start_x
    dy = state['y'] - start_y

    distance = (
        dx * math.cos(start_yaw)
        + dy * math.sin(start_yaw)
    )

    if distance >= target:
        break

    if time.monotonic() - run_start > 8.0:
        timed_out = True
        break

    if distance < slow_start:
        pub.publish(fast)
        speed_text = '0.10m/s'
    else:
        pub.publish(slow)
        speed_text = '0.07m/s'

    print(
        f'\r전진 거리: {distance * 100:.2f}cm '
        f'속도: {speed_text}',
        end='',
        flush=True
    )

    time.sleep(0.005)

for _ in range(30):
    pub.publish(stop)
    rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(0.01)

for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.01)

dx = state['x'] - start_x
dy = state['y'] - start_y

final_distance = (
    dx * math.cos(start_yaw)
    + dy * math.sin(start_yaw)
)

print(f'\n최종 오도메트리 전진거리: {final_distance * 100:.2f}cm')
print('종료 원인:', '제한시간 초과' if timed_out else '목표 도달')
print('정지 완료')

node.destroy_node()
rclpy.shutdown()
