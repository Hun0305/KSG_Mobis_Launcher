import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from interfaces_pkg.msg import MotionCommand
from enum import Enum

class ParkingState(Enum):
    SEARCH = 1
    BACK_INTO_SPOT = 2
    ADJUST_FORWARD = 3
    GO_OUT_TURN = 4
    GO_OUT_STRAIGHT = 5

class ParkingNode(Node):
    def __init__(self):
        super().__init__('motion_parking')
        self.state = ParkingState.SEARCH
        self.state_start_time = None

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 장애물 관련 상태 변수
        self.obs_detect_count = 0
        self.obs_clear_count = 0
        self.obstacle_state = 'no_obstacle'  # or 'passed_first', 'between', 'passed_second'

    def lidar_callback(self, msg: LaserScan):
        if self.state == ParkingState.SEARCH:
            self.handle_search(msg)
        elif self.state == ParkingState.BACK_INTO_SPOT:
            self.handle_back_into_spot(msg)
        elif self.state == ParkingState.ADJUST_FORWARD:
            self.handle_adjust_forward(msg)
        elif self.state == ParkingState.GO_OUT_TURN:
            self.handle_go_out_turn(msg)
        elif self.state == ParkingState.GO_OUT_STRAIGHT:
            self.handle_go_out_straight(msg)

    def handle_search(self, msg: LaserScan):
        # 265~275도 범위 평균 거리 계산
        angle_min_idx = int(len(msg.ranges) * (265 / 360))
        angle_max_idx = int(len(msg.ranges) * (275 / 360))
        section = msg.ranges[angle_min_idx:angle_max_idx]
        filtered = [r for r in section if 0.0 < r < float('inf')]
        avg_distance = sum(filtered) / len(filtered) if filtered else float('inf')

        if avg_distance <= 2.0:
            self.obs_detect_count += 1
            self.obs_clear_count = 0
        else:
            self.obs_clear_count += 1
            self.obs_detect_count = 0

        if self.obstacle_state == 'no_obstacle' and self.obs_detect_count >= 3:
            self.obstacle_state = 'passed_first'
            self.get_logger().info("🚧 첫 번째 장애물 감지 완료")

        elif self.obstacle_state == 'passed_first' and self.obs_clear_count >= 3:
            self.obstacle_state = 'between'
            self.get_logger().info("🟩 첫 번째와 두 번째 장애물 사이")

        elif self.obstacle_state == 'between' and self.obs_detect_count >= 3:
            self.obstacle_state = 'passed_second'
            self.get_logger().info("🚧 두 번째 장애물 감지 완료 — 정지")
            self.publish_motion_command(0, 0, 0)
            # 다음 상태로 넘어가고 싶다면 여기에 상태 전이 추가
            # self.state = ParkingState.BACK_INTO_SPOT

        else:
            # 장애물 통과 중 직진
            self.publish_motion_command(150, 150, 0)

    def handle_back_into_spot(self, msg: LaserScan):
        pass

    def handle_adjust_forward(self, msg: LaserScan):
        pass

    def handle_go_out_turn(self, msg: LaserScan):
        pass

    def handle_go_out_straight(self, msg: LaserScan):
        pass

    def publish_motion_command(self, left_speed: int, right_speed: int, steering: int):
        cmd = MotionCommand()
        cmd.left_speed = left_speed
        cmd.right_speed = right_speed
        cmd.steering = steering
        self.motion_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
