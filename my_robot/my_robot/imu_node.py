#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import math

# ── MPU6050 레지스터 주소 ──────────────────────
MPU_ADDR      = 0x68   # i2cdetect에서 확인한 주소
PWR_MGMT_1    = 0x6B
CONFIG        = 0x1A   # DLPF 설정
GYRO_CONFIG   = 0x1B
ACCEL_CONFIG  = 0x1C
ACCEL_XOUT_H  = 0x3B
GYRO_XOUT_H   = 0x43

# ── 측정 범위에 따른 스케일 팩터 ──────────────────
# 자이로 ±250°/s 설정 시: raw 값 / 131 = °/s
GYRO_SCALE  = 131.0
# 가속도 ±2g 설정 시: raw 값 / 16384 = g
ACCEL_SCALE = 16384.0
GRAVITY     = 9.80665  # g → m/s²


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.bus = SMBus(1)   # I2C 버스 1번
        self.init_mpu6050()

        # ── 자이로 바이어스 캘리브레이션 ──────────────
        self.gyro_bias_z = 0.0
        self.calibrate_gyro()

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # 50Hz로 발행
        self.timer = self.create_timer(0.02, self.publish_imu)
        self.get_logger().info('MPU6050 IMU node started (50Hz)')

    def calibrate_gyro(self, samples=500):
        """정지 상태에서 자이로 Z의 평균(바이어스)을 측정"""
        self.get_logger().info(
            'Calibrating gyro... 로봇을 완전히 정지시켜 주세요 (약 10초)')

        # 온도 안정화를 위해 잠깐 대기 (선택)
        import time
        time.sleep(1.0)

        total_z = 0.0
        for _ in range(samples):
            gz_raw = self.read_word(GYRO_XOUT_H + 4) / GYRO_SCALE  # °/s
            total_z += math.radians(gz_raw)                        # rad/s
            time.sleep(0.005)  # 200Hz로 샘플링

        self.gyro_bias_z = total_z / samples
        self.get_logger().info(
            f'Gyro Z bias = {self.gyro_bias_z:.6f} rad/s')

    def init_mpu6050(self):
        # 1. 잠자기 해제 (이걸 안 하면 값이 안 나옴)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)

        # 2. DLPF 설정: 0x03 = 대역폭 약 44Hz
        #    라이다 진동(고주파)을 칩 레벨에서 걸러줌
        self.bus.write_byte_data(MPU_ADDR, CONFIG, 0x03)

        # 3. 자이로 범위 ±250°/s (0x00) — 저속 로봇이라 충분, 분해능 최고
        self.bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0x00)

        # 4. 가속도 범위 ±2g (0x00)
        self.bus.write_byte_data(MPU_ADDR, ACCEL_CONFIG, 0x00)

    def read_word(self, reg):
        """레지스터 2개(상위/하위 바이트)를 읽어 16비트 부호값으로 변환"""
        high = self.bus.read_byte_data(MPU_ADDR, reg)
        low  = self.bus.read_byte_data(MPU_ADDR, reg + 1)
        value = (high << 8) | low
        # 16비트 2의 보수 → 부호 있는 값
        if value >= 0x8000:
            value -= 0x10000
        return value

    def publish_imu(self):
        # 자이로 (°/s → rad/s)
        gx = self.read_word(GYRO_XOUT_H)     / GYRO_SCALE
        gy = self.read_word(GYRO_XOUT_H + 2) / GYRO_SCALE
        gz = self.read_word(GYRO_XOUT_H + 4) / GYRO_SCALE

        # 가속도 (g → m/s²)
        ax = self.read_word(ACCEL_XOUT_H)     / ACCEL_SCALE * GRAVITY
        ay = self.read_word(ACCEL_XOUT_H + 2) / ACCEL_SCALE * GRAVITY
        az = self.read_word(ACCEL_XOUT_H + 4) / ACCEL_SCALE * GRAVITY

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # 자이로 (rad/s)
        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        # ── Z에서 바이어스 빼기 ──────────────
        msg.angular_velocity.z = math.radians(gz) - self.gyro_bias_z

        # 가속도 (m/s²)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # covariance — EKF가 신뢰도 판단에 사용
        # orientation(자세)은 이 드라이버가 계산 안 하므로 -1로 "없음" 표시
        msg.orientation_covariance[0] = -1.0
        # 자이로 covariance (대각선만, 적당한 값)
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        # 가속도 covariance
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()