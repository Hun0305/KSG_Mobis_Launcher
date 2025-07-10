import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import LaneInfo, MotionCommand
from std_msgs.msg import String  # ← 추가

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion')

        # LaneInfo 메시지 구독
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)

        # traffic_light_result 구독 추가
        self.create_subscription(String, 'traffic_light_result', self.traffic_callback, 10)
        self.traffic_light_status = "None"  # 초기 상태 저장용

        # MotionCommand 퍼블리셔
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 가중치 설정
        self.angle_weight = 0.7
        self.position_weight = -0.05

        self.max_speed = 255
        self.min_speed = -255

    def traffic_callback(self, msg):
        self.traffic_light_status = msg.data

    def lane_info_callback(self, msg):
        steering_angle = msg.steering_angle
        vehicle_position_x = msg.vehicle_position_x

        steering_value = (
            (steering_angle * self.angle_weight) +
            (vehicle_position_x * self.position_weight)
        )
        steering_value = max(-10, min(steering_value, 10))

        speed_factor = 60
        left_speed = speed_factor
        right_speed = speed_factor

        motion_command = MotionCommand()
        motion_command.steering = -int(steering_value)
        motion_command.left_speed = int(left_speed)
        motion_command.right_speed = int(right_speed)

        self.motion_pub.publish(motion_command)

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MotionNode has been interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
