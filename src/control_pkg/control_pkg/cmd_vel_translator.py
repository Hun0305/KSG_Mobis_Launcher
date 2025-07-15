import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces_pkg.msg import MotionCommand  # MotionCommand 메시지 임포트
import numpy as np

# --- 튜닝 파라미터 (control.py에서 가져옴) ---
WHEEL_BASE = 0.3  # 차량 축거 (m)
MAX_SPEED_MPS = 0.5  # 차량의 최대 속도 (m/s)
MAX_ARDUINO_SPEED = 255  # 아두이노로 보낼 속도 값의 최대치
MAX_STEER_ANGLE_DEG = 30 # 차량의 최대 조향각 (degree)
SERVO_CENTER_ANGLE = 90 # 서보모터 중앙값
SERVO_MAX_ANGLE_OFFSET = 30 # 중앙에서 최대로 움직이는 각도

class CmdVelTranslator(Node):
    def __init__(self):
        super().__init__('cmd_vel_translator')

        # /cmd_vel 토픽을 구독
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # /motion_command 토픽으로 발행
        self.motion_cmd_publisher = self.create_publisher(MotionCommand, 'motion_command', 10)
        
        self.get_logger().info('CmdVel Translator has been started.')

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # 선속도(m/s) -> 아두이노 모터 PWM 값으로 변환
        motor_speed = int((linear_velocity / MAX_SPEED_MPS) * MAX_ARDUINO_SPEED)
        motor_speed = np.clip(motor_speed, -MAX_ARDUINO_SPEED, MAX_ARDUINO_SPEED)
        left_speed = motor_speed
        right_speed = motor_speed

        # 각속도(rad/s) -> 서보모터 각도(degree)로 변환
        if abs(linear_velocity) > 0.05:
            steering_rad = np.arctan(WHEEL_BASE * angular_velocity / linear_velocity)
            steering_deg = np.degrees(steering_rad)
        else:
            steering_deg = 0.0

        servo_angle = SERVO_CENTER_ANGLE - (steering_deg / MAX_STEER_ANGLE_DEG) * SERVO_MAX_ANGLE_OFFSET
        servo_angle = int(np.clip(servo_angle, SERVO_CENTER_ANGLE - SERVO_MAX_ANGLE_OFFSET, SERVO_CENTER_ANGLE + SERVO_MAX_ANGLE_OFFSET))
        
        # 변환된 값으로 MotionCommand 메시지 생성
        motion_cmd = MotionCommand()
        motion_cmd.steering = servo_angle
        motion_cmd.left_speed = left_speed
        motion_cmd.right_speed = right_speed
        
        # /motion_command 토픽으로 발행
        self.motion_cmd_publisher.publish(motion_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()