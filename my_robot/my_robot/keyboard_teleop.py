#!/usr/bin/env python3
"""
AMR 전용 키보드 조종 노드
- 마지막 명령을 20Hz로 계속 발행 (워치독 대응)
- 키를 떼도 값이 유지됨. 정지는 스페이스바 또는 k
"""

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

HELP = """
─────────────────────────────
   AMR Keyboard Teleop
─────────────────────────────
        w
   a    s    d
        x

  w : 전진 (linear +)
  x : 후진 (linear -)
  a : 좌회전 (angular +)
  d : 우회전 (angular -)
  s : 즉시 정지
  space : 즉시 정지

  q / z : 최대 선속도 증가 / 감소
  e / c : 최대 각속도 증가 / 감소

  Ctrl+C : 종료
─────────────────────────────
"""


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('linear_step', 0.05)    # m/s
        self.declare_parameter('angular_step', 0.3)    # rad/s
        self.declare_parameter('max_linear', 0.26)     # m/s (Nav2와 동일)
        self.declare_parameter('max_angular', 1.82)    # rad/s
        self.declare_parameter('publish_rate', 20.0)   # Hz

        self.linear_step  = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value
        self.max_linear   = self.get_parameter('max_linear').value
        self.max_angular  = self.get_parameter('max_angular').value
        rate              = self.get_parameter('publish_rate').value

        self.lin = 0.0
        self.ang = 0.0
        self.lock = threading.Lock()

        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)
        self.create_timer(1.0 / rate, self.publish_cmd)

        self.settings = termios.tcgetattr(sys.stdin)

        print(HELP)
        self.print_status()

        self.running = True
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

    # ── 20Hz로 계속 발행 ────────────────────────
    def publish_cmd(self):
        with self.lock:
            lin = self.lin
            ang = self.ang

        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    # ── 키 입력 (논블로킹) ──────────────────────
    def get_key(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        while self.running and rclpy.ok():
            key = self.get_key()
            if not key:
                continue

            with self.lock:
                if key == 'w':
                    self.lin = min(self.lin + self.linear_step, self.max_linear)
                elif key == 'x':
                    self.lin = max(self.lin - self.linear_step, -self.max_linear)
                elif key == 'a':
                    self.ang = min(self.ang + self.angular_step, self.max_angular)
                elif key == 'd':
                    self.ang = max(self.ang - self.angular_step, -self.max_angular)
                elif key == 's' or key == ' ':
                    self.lin = 0.0
                    self.ang = 0.0
                elif key == 'q':
                    self.max_linear = min(self.max_linear + 0.05, 1.0)
                elif key == 'z':
                    self.max_linear = max(self.max_linear - 0.05, 0.05)
                elif key == 'e':
                    self.max_angular = min(self.max_angular + 0.2, 3.0)
                elif key == 'c':
                    self.max_angular = max(self.max_angular - 0.2, 0.2)
                elif key == '\x03':   # Ctrl+C
                    self.lin = 0.0
                    self.ang = 0.0
                    self.running = False
                    rclpy.shutdown()
                    return
                else:
                    continue

            self.print_status()

    def print_status(self):
        print(f'\rlinear: {self.lin:+.3f} m/s   '
              f'angular: {self.ang:+.3f} rad/s   '
              f'(max {self.max_linear:.2f} / {self.max_angular:.2f})    ',
              end='', flush=True)

    def destroy_node(self):
        self.running = False
        # 종료 전 정지 명령
        msg = Twist()
        self.pub.publish(msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
