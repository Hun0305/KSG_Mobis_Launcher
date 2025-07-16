import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from interfaces_pkg.msg import MotionCommand
from enum import Enum

class ParkingState(Enum):
    SEARCH = 1                     # 차량 탐색
    FIRST_STOP = 2                 # 첫 번째 정지
    REVERSE_RIGHT = 3              # 바퀴 오른쪽 최대 조향 후 후진
    REVERSE_STRAIGHT = 4           # 바퀴 정방향 후 직진 후진
    REVERSE_PAUSE = 5              # 후진 완료 후 3초 정지
    ADJUST_FORWARD = 6             # 전진 조정 (필요 시)
    GO_OUT_TURN = 7                # 탈출 회전
    GO_OUT_STRAIGHT = 8            # 탈출 직진

class ParkingNode(Node):
    def __init__(self):
        super().__init__('motion_parking')
        self.state = ParkingState.SEARCH # 시작 상태
        self.state_start_time = None

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 장애물 관련 상태 변수
        self.obs_detect_count = 0
        self.obs_clear_count = 0
        self.obstacle_state = 'no_obstacle'  # or 'passed_first', 'between', 'passed_second'

        self.left_obs_detect_count = 0
        self.left_obs_clear_count = 0
        self.left_obstacle_state = 'no_obstacle'  # or 'passing' or 'out'
        self.right_obs_detect_count = 0
        self.right_obs_clear_count = 0
        self.right_obstacle_state = 'no_obstacle'  # or 'passing' or 'out'

    def lidar_callback(self, msg: LaserScan):
        if self.state == ParkingState.SEARCH:
            self.handle_search(msg)
        elif self.state == ParkingState.FIRST_STOP:
            self.handle_first_stop(msg)
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
        angle_min_idx = int(len(msg.ranges) * (95 / 360))
        angle_max_idx = int(len(msg.ranges) * (105 / 360))
        section = msg.ranges[angle_min_idx:angle_max_idx]
        filtered = [r for r in section if 0.0 < r < float('inf')]
        avg_distance = sum(filtered) / len(filtered) if filtered else float('inf')

        # 조금 가까운 곳에서 시작했을 땐 장애물과의 거리가 1.4m정도였고
        # 조금 멀리서 시작하면 1.6-1.7m정도로 감지됨. 이에 따라서 후진 스피드를 다르게 주는 것도 고려.
        if avg_distance <= 2.0:
            self.get_logger().info(f"🚧 265~275도 범위에서 물체 감지! 거리: {avg_distance:.2f}m")

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
            self.get_logger().info("🚧 두 번째 장애물 감지 완료")
            # self.get_logger().info(f"time: {self.get_clock().now().to_msg().sec} s")

        elif self.obstacle_state == 'passed_second' and self.obs_clear_count >= 3:
            self.get_logger().info("🟩 두 번째 장애물 통과 - 정지")
            self.publish_motion_command(0, 0, 30)
            self.state = ParkingState.FIRST_STOP
            # self.get_logger().info(f"time: {self.get_clock().now().to_msg().sec} s")
            
            self.state_start_time = self.get_clock().now()

        else:
            # 장애물 통과 중 직진
            self.publish_motion_command(100, 100, 0)

    def handle_first_stop(self, msg: LaserScan):
        self.publish_motion_command(0, 0, 30)
        # 첫 번째 정지 상태에서 1초 후 REVERSE_RIGHT 상태로 전이
        if (self.get_clock().now() - self.state_start_time).nanoseconds >= 2e9:
            self.state_start_time = None
            self.get_logger().info("🔁 1초 정지 완료 → REVERSE_RIGHT 시작")
            self.state = ParkingState.REVERSE_RIGHT
        
    def handle_reverse_right(self, msg: LaserScan):
        # 오른쪽으로 후진
        self.publish_motion_command(-70, -30, 30)
        self.get_logger().info("🔁 오른쪽으로 후진 중...")

        #-------------------------------------------------------
        # 라이다의 좌우에 장애물이 잡히는지를 통해
        # REVERSE_STRAIGHT 상태로의 전이 여부를 판단

        # 왼쪽 장애물 탐지
        left_angle_min_idx = int(len(msg.ranges) * (265 / 360))
        left_angle_max_idx = int(len(msg.ranges) * (275 / 360))
        left_section = msg.ranges[left_angle_min_idx:left_angle_max_idx]
        left_filtered = [r for r in left_section if 0.0 < r < float('inf')]
        left_avg_distance = sum(left_filtered) / len(left_filtered) if left_filtered else float('inf')

        if left_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 왼쪽에서 물체 감지! 거리: {left_avg_distance:.2f}m")

            self.left_obs_detect_count += 1
            self.left_obs_clear_count = 0
        else:
            self.left_obs_clear_count += 1
            self.left_obs_detect_count = 0

        # ---
        if self.left_obstacle_state == 'no_obstacle' and self.left_obs_detect_count >= 3:
            self.left_obstacle_state = 'passing'
            self.get_logger().info("🚧 좌측 주차 차량 탐지 완료 - 지나는 중")

        # elif self.left_obstacle_state == 'passing' and self.left_obs_clear_count >= 3:
        #     self.left_obstacle_state = 'out'
        #     self.get_logger().info("🟩 좌측 주차 차량 탐지 안됨")


        # --------------------------------------------------------
        # 오른쪽 장애물 탐지
        right_angle_min_idx = int(len(msg.ranges) * (95 / 360))
        right_angle_max_idx = int(len(msg.ranges) * (105 / 360))
        right_section = msg.ranges[right_angle_min_idx:right_angle_max_idx]
        right_filtered = [r for r in right_section if 0.0 < r < float('inf')]
        right_avg_distance = sum(right_filtered) / len(right_filtered) if right_filtered else float('inf')

        if right_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 오른쪽에서 물체 감지! 거리: {right_avg_distance:.2f}m")

            self.right_obs_detect_count += 1
            self.right_obs_clear_count = 0
        else:
            self.right_obs_clear_count += 1
            self.right_obs_detect_count = 0

        # ---
        if self.right_obstacle_state == 'no_obstacle' and self.right_obs_detect_count >= 3:
            self.right_obstacle_state = 'passing'
            self.get_logger().info("🚧 우측 주차 차량 탐지 완료 - 지나는 중")

        # elif self.right_obstacle_state == 'passing' and self.right_obs_clear_count >= 3:
        #     self.right_obstacle_state = 'out'
        #     self.get_logger().info("🟩 우측 주차 차량 탐지 안됨")

        # REVERSE_STRAIGHT 상태 전이
        if self.left_obstacle_state == 'passing' and self.right_obstacle_state == 'passing':
            self.state = ParkingState.REVERSE_STRAIGHT

    def handle_reverse_straight(self, msg: LaserScan):
        # 똑바로 후진
        self.publish_motion_command(-70, -70, 0)
        self.get_logger().info("🔁 똑바로 후진 중...")

        #-------------------------------------------------------
        # 라이다의 좌우에 장애물이 잡히는지를 통해
        # 주차 완료를 확힌

        # 왼쪽 장애물 탐지
        left_angle_min_idx = int(len(msg.ranges) * (265 / 360))
        left_angle_max_idx = int(len(msg.ranges) * (275 / 360))
        left_section = msg.ranges[left_angle_min_idx:left_angle_max_idx]
        left_filtered = [r for r in left_section if 0.0 < r < float('inf')]
        left_avg_distance = sum(left_filtered) / len(left_filtered) if left_filtered else float('inf')

        if left_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 왼쪽에서 물체 감지! 거리: {left_avg_distance:.2f}m")

            self.left_obs_detect_count += 1
            self.left_obs_clear_count = 0
        else:
            self.left_obs_clear_count += 1
            self.left_obs_detect_count = 0

        # ---

        if self.left_obstacle_state == 'passing' and self.left_obs_clear_count >= 3:
            self.left_obstacle_state = 'out'
            self.get_logger().info("🟩 좌측 주차 차량 탐지 안됨")

        # --------------------------------------------------------
        # 오른쪽 장애물 탐지
        right_angle_min_idx = int(len(msg.ranges) * (95 / 360))
        right_angle_max_idx = int(len(msg.ranges) * (105 / 360))
        right_section = msg.ranges[right_angle_min_idx:right_angle_max_idx]
        right_filtered = [r for r in right_section if 0.0 < r < float('inf')]
        right_avg_distance = sum(right_filtered) / len(right_filtered) if right_filtered else float('inf')

        if right_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 오른쪽에서 물체 감지! 거리: {right_avg_distance:.2f}m")

            self.right_obs_detect_count += 1
            self.right_obs_clear_count = 0
        else:
            self.right_obs_clear_count += 1
            self.right_obs_detect_count = 0

        # ---
        if self.right_obstacle_state == 'passing' and self.right_obs_clear_count >= 3:
            self.right_obstacle_state = 'out'
            self.get_logger().info("🟩 우측 주차 차량 탐지 안됨")

        # REVERSE_PAUSE 상태 전이
        if self.left_obstacle_state == 'out' and self.right_obstacle_state == 'out':
            self.state_start_time = self.get_clock().now()
            self.state = ParkingState.REVERSE_PAUSE

    def handle_reverse_pause(self, msg: LaserScan):
        # 후진 완료 후 3초 정지
        self.publish_motion_command(0, 0, 0)
        self.get_logger().info("🔁 후진 완료 - 3초 정지 중...")

        if (self.get_clock().now() - self.state_start_time).nanoseconds >= 4e9:
            self.state_start_time = self.get_clock().now()
            self.get_logger().info("🔁 3초 정지 완료 → ADJUST_FORWARD 시작")
            self.state = ParkingState.ADJUST_FORWARD

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
        # 탈출 직진
        self.publish_motion_command(70, 70, 0)
        self.get_logger().info("🔁 탈출 직진 중...")

        # --------------------------------------------------------
        # 오른쪽 장애물 탐지
        right_angle_min_idx = int(len(msg.ranges) * (95 / 360))
        right_angle_max_idx = int(len(msg.ranges) * (105 / 360))
        right_section = msg.ranges[right_angle_min_idx:right_angle_max_idx]
        right_filtered = [r for r in right_section if 0.0 < r < float('inf')]
        right_avg_distance = sum(right_filtered) / len(right_filtered) if right_filtered else float('inf')

        if right_avg_distance <= 1.0:
            self.get_logger().info(f"🚧 오른쪽에서 물체 감지! 거리: {right_avg_distance:.2f}m")

            self.right_obs_detect_count += 1
            self.right_obs_clear_count = 0
        else:
            self.right_obs_clear_count += 1
            self.right_obs_detect_count = 0

        # ---
        if (self.get_clock().now() - self.state_start_time).nanoseconds >= 3e9 and self.right_obstacle_state == 'out' and self.right_obs_clear_count >= 3:
            self.state_start_time = self.get_clock().now()
            self.right_obstacle_state = 'passing'
            self.state = ParkingState.GO_OUT_TURN
            self.get_logger().info("🟩 우측 주차 차량 탐지 안됨 - 우회전 시작")


    # 여섯번째 단계, 우측 조향 후 전진
    # 오류 발생 주의점 : 의외로 조금만 돌아도 라이더에 우측 차량이 잡힐 가능성이 높다고 생각함 
    # -> 각도를 조정하거나 최악의 경우 time을 사용해 보는 것을 고려, 아니면 카메라를 사용해서 평행주차 라인을 따서 그걸 타고 나가기..?
    def handle_go_out_turn(self, msg: LaserScan):
        # 탈출 우회전
        self.publish_motion_command(50, 20, 30)
        self.get_logger().info("🔁 탈출 우회전 중...")

        # --------------------------------------------------------
        # 오른쪽 장애물 탐지
        right_angle_min_idx = int(len(msg.ranges) * (95 / 360))
        right_angle_max_idx = int(len(msg.ranges) * (105 / 360))
        right_section = msg.ranges[right_angle_min_idx:right_angle_max_idx]
        right_filtered = [r for r in right_section if 0.0 < r < float('inf')]
        right_avg_distance = sum(right_filtered) / len(right_filtered) if right_filtered else float('inf')

        if right_avg_distance <= 1.5:
            self.get_logger().info(f"🚧 오른쪽에서 물체 감지! 거리: {right_avg_distance:.2f}m")

            self.right_obs_detect_count += 1
            self.right_obs_clear_count = 0
        else:
            self.right_obs_clear_count += 1
            self.right_obs_detect_count = 0

        # ---
        if (self.get_clock().now() - self.state_start_time).nanoseconds >= 25e9 and self.right_obstacle_state == 'passing' and self.right_obs_clear_count >= 3:
            self.right_obstacle_state = 'out'
            self.state = ParkingState.GO_OUT_STRAIGHT
            self.get_logger().info("🟩 우측 주차 차량 탐지 안됨 - 직진 시작")

    def handle_go_out_straight(self, msg: LaserScan):
        # 탈출 우회전
        self.publish_motion_command(70, 70, 0)

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
