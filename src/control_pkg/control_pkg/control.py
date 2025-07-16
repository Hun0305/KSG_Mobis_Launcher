#!/usr/bin/env python3

import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import MotionCommand
import serial

from geometry_msgs.msg import Twist      # /cmd_vel 토픽을 위한 Twist 메시지
from sensor_msgs.msg import Range      # 초음파 센서 데이터 발행을 위한 Range 메시지
import numpy as np

# --- 튜닝 파라미터 ---
WHEEL_BASE = 0.3  # 차량 축거 (m)
MAX_ARDUINO_SPEED = 255  # 아두이노로 보낼 속도 값의 최대치
MAX_STEER_ANGLE_DEG = 30 # 차량의 최대 조향각 (degree)
SERVO_CENTER_ANGLE = 90 # 서보모터 중앙값
SERVO_MAX_ANGLE_OFFSET = 30 # 중앙에서 최대로 움직이는 각도

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        # 1. vehicle_controller로부터 /cmd_vel 토픽을 구독
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 2. 초음파 데이터를 발행할 퍼블리셔
        self.ultrasonic_publisher = self.create_publisher(Range, '/ultrasonic/sensor_0', 10)

        # 4. 아두이노로부터 데이터를 주기적으로 읽기 위한 타이머
        self.timer = self.create_timer(0.1, self.read_from_arduino)
        self.get_logger().info("Control node has been started and is handling all Arduino I/O.")


        # serial port
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.get_logger().info("Connected to Arduino on /dev/ttyACM0 @9600")

        # 토글 플래그: True면 모션 전송, False면 제로 전송
        self.send_motion = False

        # 구독
        self.create_subscription(
            MotionCommand,
            'motion_command',
            self.motion_command_callback,
            10
        )

        # 스페이스바로 send_motion 토글
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

    def motion_command_callback(self, msg: MotionCommand):
        # 모드에 따라 즉시 전송
        if self.send_motion:
            cmd = f"s{msg.steering}r{msg.right_speed}l{msg.left_speed}\n"
        else:
            cmd = "s0r0l0\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent: {cmd.strip()}")

    def _keyboard_loop(self):
        """stdin에서 스페이스바를 감지해 모드를 토글."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self.send_motion = not self.send_motion
                    mode = "MOTION" if self.send_motion else "ZERO"
                    self.get_logger().info(f"Toggled to {mode} mode")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def cmd_vel_callback(self, msg):
        """/cmd_vel 메시지를 아두이노 명령으로 변환하여 전송"""
        # Twist 메시지에서 선속도(v)와 각속도(w) 추출
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # 선속도(m/s)를 모터 PWM 값으로 변환
        motor_speed = int(linear_velocity * MAX_ARDUINO_SPEED) # 차량 최대 속도 1m/s 가정
        motor_speed = np.clip(motor_speed, -MAX_ARDUINO_SPEED, MAX_ARDUINO_SPEED)
        left_speed = motor_speed
        right_speed = motor_speed

        # 각속도(rad/s)를 서보모터 각도(degree)로 변환
        if abs(linear_velocity) > 0.05:
            steering_rad = np.arctan(WHEEL_BASE * angular_velocity / linear_velocity)
            steering_deg = np.degrees(steering_rad)
        else:
            steering_deg = 0.0
        
        servo_angle = SERVO_CENTER_ANGLE - (steering_deg / MAX_STEER_ANGLE_DEG) * SERVO_MAX_ANGLE_OFFSET
        servo_angle = int(np.clip(servo_angle, SERVO_CENTER_ANGLE - SERVO_MAX_ANGLE_OFFSET, SERVO_CENTER_ANGLE + SERVO_MAX_ANGLE_OFFSET))
        
        # 아두이노로 보낼 데이터 형식으로 변환
        command = f"s{servo_angle}r{right_speed}l{left_speed}\n"
        self.ser.write(command.encode())

    def read_from_arduino(self):
        """아두이노로부터 초음파 데이터를 읽어 ROS 토픽으로 발행"""
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if ',' in line:
                    parts = line.split(',')
                    distance_cm = float(parts[1].replace(';', ''))

                    range_msg = Range()
                    range_msg.header.stamp = self.get_clock().now().to_msg()
                    range_msg.header.frame_id = 'ultrasonic_sensor_0_frame'
                    range_msg.radiation_type = Range.ULTRASOUND
                    range_msg.field_of_view = 0.26
                    range_msg.min_range = 0.02
                    range_msg.max_range = 4.0
                    range_msg.range = distance_cm / 100.0
                    self.ultrasonic_publisher.publish(range_msg)
            except Exception as e:
                self.get_logger().warn(f"Could not parse ultrasonic data: '{line}'. Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
