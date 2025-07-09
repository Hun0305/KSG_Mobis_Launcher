import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import LaneInfo, MotionCommand  # MotionCommand 메시지 임포트

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion')

        # LaneInfo 메시지를 구독
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)

        # MotionCommand 메시지를 발행할 퍼블리셔 생성
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 가중치 설정 (각도, 차량 위치에 대한 가중치)
        self.angle_weight = 0.7  # 중앙선 각도에 대한 가중치
        self.position_weight = -0.05  # 차량 위치에 대한 가중치

        # 속도 제한
        self.max_speed = 255
        self.min_speed = -255

    def lane_info_callback(self, msg):
        # LaneInfo 메시지에서 값 추출
        steering_angle = msg.steering_angle
        vehicle_position_x = msg.vehicle_position_x

        # 각 값에 대한 가중치를 곱해서 최종 조향값 계산
        steering_value = (
            (steering_angle * self.angle_weight) +  # 각도에 대한 가중치 적용
            (vehicle_position_x * self.position_weight)  # 차량 위치에 대한 가중치 적용
        )

        # 계산된 조향값을 제한 (예: -10에서 10 사이)
        steering_value = max(-10, min(steering_value, 10))

        # 속도 계산 (여기서는 간단한 예시로, 실제 속도 계산 로직을 추가해야 합니다)
        #speed_factor = max(-255, min(255, vehicle_position_x))  # 차량의 위치에 따른 속도 조정
        speed_factor = 60
        left_speed = speed_factor  # 예시: 왼쪽 속도
        right_speed = speed_factor  # 예시: 오른쪽 속도

        # MotionCommand 메시지 생성
        motion_command = MotionCommand()
        motion_command.steering = -int(steering_value)  # 조향각
        motion_command.left_speed = int(left_speed)  # 왼쪽 속도
        motion_command.right_speed = int(right_speed)  # 오른쪽 속도

        # MotionCommand 메시지 발행
        self.motion_pub.publish(motion_command)
        self.get_logger().info(f"Published Motion Command: Steering: {steering_value}, Left Speed: {left_speed}, Right Speed: {right_speed}")

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


