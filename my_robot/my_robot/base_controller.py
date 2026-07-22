#!/usr/bin/env python3
"""
AMR Base Controller
- /cmd_vel 구독 → 아두이노로 바퀴 속도 명령 전송
- 아두이노 엔코더 피드백 → /odom 발행 + TF
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class BaseController(Node):

    def __init__(self):
        super().__init__('base_controller')

        # ── 파라미터 ────────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_separation', 0.185)   # m
        self.declare_parameter('wheel_diameter', 0.065)     # m
        self.declare_parameter('encoder_cpr', 1320.0)
        self.declare_parameter('cmd_rate', 50.0)            # Hz
        self.declare_parameter('publish_tf', True)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_dia = self.get_parameter('wheel_diameter').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.cmd_rate = self.get_parameter('cmd_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.wheel_circum = math.pi * self.wheel_dia
        self.meters_per_count = self.wheel_circum / self.encoder_cpr

        # ── 시리얼 연결 ─────────────────────────
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {self.port}: {e}')
            raise

        self.get_logger().info(f'Connected to {self.port} @ {self.baudrate}')

        # 아두이노 리셋 대기
        import time
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        # 엔코더 리셋 명령
        self.ser.write(b'r\n')

        # ── 상태 변수 ───────────────────────────
        self.target_vl = 0.0
        self.target_vr = 0.0

        # ── cmd_vel 워치독 ──────────────────────
        self.declare_parameter('cmd_vel_timeout', 1.0)   # 초
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_timed_out = True

        self.enc_l = 0
        self.enc_r = 0
        self.prev_enc_l = None
        self.prev_enc_r = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_odom_time = self.get_clock().now()

        self.lock = threading.Lock()

        # ── ROS2 인터페이스 ─────────────────────
        # cmd_vel: 최신 값만 중요 → BEST_EFFORT + depth 1
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # odom: 센서 데이터 → BEST_EFFORT
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, cmd_qos)

        self.odom_pub = self.create_publisher(Odometry, 'odom', odom_qos)

        self.tf_broadcaster = TransformBroadcaster(self)
        # ── 타이머 ──────────────────────────────
        self.create_timer(1.0 / self.cmd_rate, self.send_command)

        # ── 시리얼 수신 스레드 ──────────────────
        self.running = True
        self.rx_thread = threading.Thread(target=self.serial_rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info('Base controller ready')

    # ========================================
    # /cmd_vel → 좌/우 바퀴 속도 (역기구학)
    # ========================================
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x       # m/s
        w = msg.angular.z      # rad/s

        # 차동구동 역기구학
        vl = v - (w * self.wheel_sep / 2.0)
        vr = v + (w * self.wheel_sep / 2.0)

        with self.lock:
            self.target_vl = vl
            self.target_vr = vr
            self.last_cmd_vel_time = self.get_clock().now()   # ← 추가
            if self.cmd_vel_timed_out:                        # ← 추가
                self.cmd_vel_timed_out = False                # ← 추가
                self.get_logger().info('cmd_vel resumed')     # ← 추가
    # ========================================
    # 50Hz로 아두이노에 명령 전송
    # ========================================
    def send_command(self):
        now = self.get_clock().now()

        with self.lock:
            elapsed = (now - self.last_cmd_vel_time).nanoseconds / 1e9

            # /cmd_vel 이 끊기면 목표를 0으로
            if elapsed > self.cmd_vel_timeout:
                if not self.cmd_vel_timed_out:
                    self.cmd_vel_timed_out = True
                    self.get_logger().warn('cmd_vel timeout -> stopping')
                self.target_vl = 0.0
                self.target_vr = 0.0

            vl = self.target_vl
            vr = self.target_vr

        cmd = f'v {vl:.4f} {vr:.4f}\n'
        try:
            self.ser.write(cmd.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    # ========================================
    # 시리얼 수신 루프 (별도 스레드)
    # ========================================
    def serial_rx_loop(self):
        while self.running and rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException:
                continue

            if not line:
                continue

            if line.startswith('e '):
                parts = line.split()
                if len(parts) == 3:
                    try:
                        el = int(parts[1])
                        er = int(parts[2])
                    except ValueError:
                        continue
                    self.update_odometry(el, er)

            elif line.startswith('#'):
                self.get_logger().debug(f'arduino: {line}')

    # ========================================
    # 엔코더 → 오도메트리 (정기구학)
    # ========================================
    def update_odometry(self, el: int, er: int):
        now = self.get_clock().now()

        if self.prev_enc_l is None:
            self.prev_enc_l = el
            self.prev_enc_r = er
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        d_left  = (el - self.prev_enc_l) * self.meters_per_count
        d_right = (er - self.prev_enc_r) * self.meters_per_count

        self.prev_enc_l = el
        self.prev_enc_r = er
        self.last_odom_time = now

        # 차동구동 정기구학
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.wheel_sep

        # 위치 갱신 (중점 적분)
        if abs(d_theta) < 1e-6:
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        else:
            self.x += d_center * math.cos(self.theta + d_theta / 2.0)
            self.y += d_center * math.sin(self.theta + d_theta / 2.0)

        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        vx = d_center / dt
        vth = d_theta / dt

        self.publish_odom(now, vx, vth)

    # ========================================
    # /odom + TF 발행
    # ========================================
    def publish_odom(self, stamp, vx: float, vth: float):
        q = self.yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        # 공분산 (경험적 값)
        odom.pose.covariance[0]  = 0.001   # x
        odom.pose.covariance[7]  = 0.001   # y
        odom.pose.covariance[35] = 0.01    # yaw
        odom.twist.covariance[0]  = 0.001
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # ========================================
    def destroy_node(self):
        self.running = False
        try:
            self.ser.write(b's\n')
            self.ser.flush()
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
