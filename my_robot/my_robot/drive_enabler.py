#!/usr/bin/env python3
"""
drive_enabler — 런치 직후 CiA 402 드라이브를 한 번 활성화하는 노드

하는 일 (한 번만 실행하고 종료):
  1) 채널별 init / velocity_mode 서비스가 뜰 때까지 기다린다
  2) diff_drive_controller 가 active 가 될 때까지 기다린다
     → 활성화 순간에 바퀴 명령이 0 으로 들어가 있게 하려는 것
  3) 모든 채널 init        (CiA 402: 06 → 07 → 0F)
     → 두 축이 모두 Switched On 이 되는 순간 펌웨어가 릴레이를 붙인다
  4) 모든 채널 velocity_mode (Profile Velocity = 3)
     → 두 축 모두 끝나야 펌웨어가 PWM 을 내보낸다

모든 채널에 init 을 정확히 한 번씩 부른다 (한 채널이 실패해도 나머지도 부른다):
  upstream Motor402 는 동작 모드 객체(Profile Velocity 등)를 init 이 처음 불릴 때 만든다.
  recover 에는 이 과정이 없어서, init 을 한 번도 거치지 않은 채널은 나중에
  recover → velocity_mode 로 복구할 때 velocity_mode 가 False 가 된다.

하지 않는 일:
  - 실패한 init 을 재시도하지 않는다.
    upstream 의 init 은 내부에서 Fault Reset 도 함께 보낸다. 재시도를 반복하면
    E-Stop 을 푸는 순간 사람 확인 없이 다시 켜져 버린다 (= recover 자동화).
    재시도는 "서비스/컨트롤러가 아직 안 떴을 때" 기다리는 것뿐이다.
  - recover 를 부르지 않는다. 고장 이후 복귀는 사람이 원인을 확인하고 직접 한다.
  - 런치 이후 상태를 감시하지 않는다. 활성화가 끝나면 노드는 종료한다.

실패하면: 이미 켠 채널은 disable 로 되돌리고, 무엇이 실패했는지 로그를 남긴 뒤
          종료 코드 1 로 끝난다. (펌웨어도 두 축이 모두 준비되지 않으면 출력하지 않는다)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers


class DriveEnabler(Node):
    def __init__(self):
        super().__init__('drive_enabler')

        self.declare_parameter('channels', ['right_wheel', 'left_wheel'])
        self.declare_parameter('controller', 'diff_drive_controller')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('wait_timeout_s', 60.0)   # 서비스/컨트롤러 대기 한도
        self.declare_parameter('call_timeout_s', 10.0)   # 서비스 한 번 응답 한도 (드라이버 전이 한도 5s 보다 길게)

        self.channels = list(self.get_parameter('channels').value)
        self.controller = self.get_parameter('controller').value
        self.cm = self.get_parameter('controller_manager').value
        self.wait_timeout = float(self.get_parameter('wait_timeout_s').value)
        self.call_timeout = float(self.get_parameter('call_timeout_s').value)

        self.enabled = []   # init 에 성공한 채널 (실패 시 되돌리기용)

    # ── 서비스 한 번 호출 (동기) ──────────────────────────────
    def call_trigger(self, name):
        client = self.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=self.wait_timeout):
            self.get_logger().error(f'{name}: 서비스가 {self.wait_timeout:.0f}초 안에 나타나지 않음')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'{name}: {self.call_timeout:.0f}초 안에 응답 없음')
            return False
        ok = future.result().success
        (self.get_logger().info if ok else self.get_logger().error)(
            f'{name}: success={ok}')
        return ok

    # ── 컨트롤러가 active 가 될 때까지 대기 ──────────────────
    def wait_controller_active(self):
        client = self.create_client(ListControllers, f'{self.cm}/list_controllers')
        deadline = time.monotonic() + self.wait_timeout
        if not client.wait_for_service(timeout_sec=self.wait_timeout):
            self.get_logger().error(f'{self.cm}/list_controllers 서비스 없음')
            return False
        while time.monotonic() < deadline:
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                for c in future.result().controller:
                    if c.name == self.controller and c.state == 'active':
                        self.get_logger().info(f'{self.controller}: active')
                        return True
            time.sleep(0.5)
        self.get_logger().error(f'{self.controller} 가 {self.wait_timeout:.0f}초 안에 active 가 되지 않음')
        return False

    # ── 실패 시 이미 켠 채널 끄기 ────────────────────────────
    def rollback(self):
        for ch in reversed(self.enabled):
            self.get_logger().warn(f'/{ch}/disable 로 되돌림')
            self.call_trigger(f'/{ch}/disable')

    def run(self):
        self.get_logger().info(f'드라이브 활성화 시작: channels={self.channels}')

        if not self.wait_controller_active():
            return False

        # 1) 모든 채널 init — 두 축이 모두 Switched On 이 되는 순간 릴레이 ON
        #    한 채널이 실패해도 나머지 채널 init 도 한 번은 부른다 (모드 객체 생성 때문, 위 설명)
        failed = []
        for ch in self.channels:
            if self.call_trigger(f'/{ch}/init'):
                self.enabled.append(ch)
            else:
                failed.append(ch)
        if failed:
            self.get_logger().error(
                f'init 실패 채널: {failed}. 재시도하지 않습니다. '
                'E-Stop·고장 여부를 확인한 뒤 J-7 의 recover 순서로 직접 복귀하세요.')
            self.rollback()
            return False

        # 2) 모든 채널 velocity_mode — 두 축 모두 끝나야 PWM 출력
        for ch in self.channels:
            if not self.call_trigger(f'/{ch}/velocity_mode'):
                self.get_logger().error(f'/{ch}/velocity_mode 실패')
                self.rollback()
                return False

        self.get_logger().info('드라이브 활성화 완료 — /cmd_vel_out 명령을 따릅니다')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = DriveEnabler()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()