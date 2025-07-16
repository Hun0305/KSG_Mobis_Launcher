# 기존 내용을 모두 지우고 아래 코드로 완전히 교체하세요.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class ParkingControlNode(Node):
    def __init__(self):
        super().__init__('parking_control_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        try:
            self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)
            self.get_logger().info('Connected to Arduino on /dev/ttyACM0 @9600')
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

        self.get_logger().info('Parking Control node has been started.')

    def listener_callback(self, msg):
        if self.arduino is None:
            self.get_logger().warn('Arduino not connected. Cannot send command.')
            return

        speed = msg.linear.x
        steer = msg.angular.z

        # <<< ❗ 이 부분이 가장 중요합니다. speed가 양수(+)일 때 
        # speed_val도 양수(+)가 되어야 합니다. ❗>>>
        speed_val = int(speed * 100)
        
        steer_val = int(steer * (180 / 3.141592) + 90)
        
        command = f"s{speed_val},{steer_val}#"
        
        try:
            self.arduino.write(command.encode())
            self.get_logger().info(f'Sent to Arduino: "{command}"')
        except Exception as e:
            self.get_logger().error(f"Failed to write to Arduino: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ParkingControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()