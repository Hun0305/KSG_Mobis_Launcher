import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import MotionCommand  # MotionCommand 메시지 임포트
import serial

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        # MotionCommand 메시지를 구독
        self.create_subscription(MotionCommand, 'motion_command', self.motion_command_callback, 10)

        # 아두이노 시리얼 포트 설정
        self.ser = serial.Serial('/dev/ttyACM0', 9600)  # 아두이노 포트
        self.get_logger().info("Connected to Arduino.")

    def motion_command_callback(self, msg):
        # 수신된 MotionCommand 메시지에서 steering, left_speed, right_speed 값을 추출
        steering = msg.steering
        left_speed = msg.left_speed
        right_speed = msg.right_speed

        # 아두이노로 보낼 데이터 형식으로 변환 (s(steering), r(right_speed), l(left_speed))
        command = f"s{steering}r{right_speed}l{left_speed}\n"
        
        # 시리얼 포트를 통해 아두이노로 보냄
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent to Arduino: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ControlNode has been interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



