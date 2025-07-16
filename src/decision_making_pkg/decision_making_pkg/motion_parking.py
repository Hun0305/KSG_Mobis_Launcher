import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from interfaces_pkg.msg import MotionCommand
from enum import Enum

class ParkingState(Enum):
    SEARCH = 1                     # 차량 탐색
    REVERSE_RIGHT = 2              # 바퀴 오른쪽 최대 조향 후 후진
    REVERSE_STRAIGHT = 3           # 바퀴 정방향 후 똑바로 후진
    REVERSE_PAUSE = 4              # 후진 완료 후 3초 정지
    ADJUST_FORWARD = 5             # 똑바로 전진 
    GO_OUT_TURN = 6                # 탈출 회전
    GO_OUT_STRAIGHT = 7            # 탈출 직진

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
        elif self.state == ParkingState.REVERSE_RIGHT:
            self.handle_reverse_right(msg)
        elif self.state == ParkingState.REVERSE_STRAIGHT:
            self.handle_reverse_straight(msg)
        elif self.state == ParkingState.REVERSE_PAUSE:
            self.handle_reverse_pause(msg)
        elif self.state == ParkingState.ADJUST_FORWARD:
            self.handle_adjust_forward(msg)
        elif self.state == ParkingState.GO_OUT_TURN:
            self.handle_go_out_turn(msg)
        elif self.state == ParkingState.GO_OUT_STRAIGHT:
            self.handle_go_out_straight(msg)

    # 첫번째 단계, 직진
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
            self.get_logger().info("첫 번째 장애물 감지 완료")

        elif self.obstacle_state == 'passed_first' and self.obs_clear_count >= 3:
            self.obstacle_state = 'between'
            self.get_logger().info("첫 번째와 두 번째 장애물 사이")

        elif self.obstacle_state == 'between' and self.obs_detect_count >= 3:
            self.obstacle_state = 'passed_second'
            self.get_logger().info("두 번째 장애물 감지 완료 — 정지")
            self.publish_motion_command(0, 0, 0)            
            self.state = ParkingState.REVERSE_RIGHT

        else:
            # 장애물 통과 중 직진
            self.publish_motion_command(150, 150, 0)


    # 두번째 단계, 우측 조향 후 후진
    def handle_reverse_right(self, msg: LaserScan):
        # 라이더 감지 영역 설정
        right_indices = self.get_range_indices(msg, 265, 275)
        left_indices = self.get_range_indices(msg, 85, 95)

        right_detected = self.has_obstacle(msg, right_indices)
        left_detected = self.has_obstacle(msg, left_indices)

        if right_detected and left_detected:
            self.state = ParkingState.REVERSE_STRAIGHT
            self.get_logger().info("양쪽 장애물 감지됨 → REVERSE_STRAIGHT 상태로 전이")
            self.publish_motion_command(0, 0, 0)
            return

        else:
            self.publish_motion_command(-130, -130, 30)
    

    # 세번째 단계, 똑바로 후진
    def handle_reverse_straight(self, msg: LaserScan):
        right_indices = self.get_range_indices(msg, 265, 275)
        left_indices = self.get_range_indices(msg, 85, 95)

        right_detected = self.has_obstacle(msg, right_indices)
        left_detected = self.has_obstacle(msg, left_indices)

        if not right_detected and not left_detected:
            self.state = ParkingState.REVERSE_PAUSE
            self.get_logger().info("후진 완료: 양쪽 물체 감지 없음 → 3초 정지 시작 (REVERSE_PAUSE)")
            self.publish_motion_command(0, 0, 0)
            return

        else:
            self.publish_motion_command(-130, -130, 0)


    # 네번째 단계, 정지
    def handle_reverse_pause(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds

        # 정지 시작 시간 기록이 없다면 지금으로 설정
        if self.state_start_time is None:
            self.state_start_time = now
            self.get_logger().info("후진 완료 후 정지 시작 (3초)")

        # 3초가 지나면 다음 상태로 전이
        elif now - self.state_start_time >= 3e9:
            self.state = ParkingState.ADJUST_FORWARD
            self.state_start_time = now  # 다음 상태 시작 시간 갱신
            self.get_logger().info("3초 정지 완료 → ADJUST_FORWARD 진입")

        else:
            self.publish_motion_command(0, 0, 0)


    # 다섯번째 단계, 똑바로 전진
    # 오류 발생 주의점 : 처음 출발할 때 장애물이 양옆에 안잡히는 상태일터라 처음에 직진을 잘 할까? 에 대한 의문이 남아 있긴 함 > 확인 필요..
    def handle_adjust_forward(self, msg: LaserScan):
        right_indices = self.get_range_indices(msg, 265, 275)
        left_indices = self.get_range_indices(msg, 85, 95)

        right_detected = self.has_obstacle(msg, right_indices)
        left_detected = self.has_obstacle(msg, left_indices)

        if not left_detected and not right_detected:
            # 좌우 모두 감지되지 않으면 탈출 회전으로 이동
            self.state = ParkingState.GO_OUT_TURN
            self.get_logger().info("양쪽 장애물 사라짐 → GO_OUT_TURN 진입")

        else:
            self.publish_motion_command(150, 150, 0)
        

    # 여섯번째 단계, 우측 조향 후 전진
    # 오류 발생 주의점 : 의외로 조금만 돌아도 라이더에 우측 차량이 잡힐 가능성이 높다고 생각함 
    # -> 각도를 조정하거나 최악의 경우 time을 사용해 보는 것을 고려, 아니면 카메라를 사용해서 평행주차 라인을 따서 그걸 타고 나가기..?
    def handle_go_out_turn(self, msg: LaserScan):
        right_indices = self.get_range_indices(msg, 265, 275)
        right_detected = self.has_obstacle(msg, right_indices)

        if right_detected:
            self.state = ParkingState.GO_OUT_STRAIGHT
            self.get_logger().info("차량이 다시 감지됨 → GO_OUT_STRAIGHT 전이")
            return

        else:
            self.publish_motion_command(150, 150, 30)


    # 일곱번째 단계, 똑바로 전진
    def handle_go_out_straight(self, msg: LaserScan):
        self.publish_motion_command(150, 150, 0)  
        

    def get_range_indices(self, msg, angle_start, angle_end):
        total_len = len(msg.ranges)
        start_idx = int((angle_start / 360.0) * total_len)
        end_idx = int((angle_end / 360.0) * total_len)
        return range(start_idx, end_idx)

    def has_obstacle(self, msg, indices, threshold=1.5):
        for i in indices:
            r = msg.ranges[i]
            if 0.05 < r < threshold:
                return True
        return False

    # 조향 관련
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
