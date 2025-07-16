# src/lidar_pkg/lidar_pkg/vehicle_controller.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import tf_transformations

# --- 튜닝 파라미터 ---
LOOKAHEAD_DISTANCE = 1.0
VEHICLE_LENGTH = 0.3
MAX_STEER = np.radians(25)
TARGET_VELOCITY = 0.2
INITIAL_VELOCITY = 0.1  # <<< 추가: 초기 저속 주행 속도

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')

        # 경로와 현재 위치(Odometry)를 구독
        self.path_subscription = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            1)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # 차량 제어 명령 (Twist)을 발행
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_path = None
        self.current_pose = None
        self.path_index = 0

        # <<< 추가: 초기 주행 상태 플래그 및 타이머 >>>
        self.is_initial_drive = True
        self.initial_drive_timer = self.create_timer(0.1, self.initial_drive_callback)
        # <<< --- >>>

        self.get_logger().info('Vehicle Controller Node has been started.')
        self.get_logger().info('Starting with initial slow drive.')


    # <<< 추가: 초기 저속 직진 콜백 함수 >>>
    def initial_drive_callback(self):
        """초기 주행 상태일 때 저속으로 직진 명령을 보냅니다."""
        if self.is_initial_drive:
            # 저속으로 직진하는 Twist 메시지 생성
            twist = Twist()
            twist.linear.x = INITIAL_VELOCITY
            twist.angular.z = 0.0
            self.cmd_publisher.publish(twist)
    # <<< --- >>>

    def path_callback(self, msg):
        """경로를 수신하면 초기 주행을 멈추고 경로 추종을 시작합니다."""
        if not self.current_path and msg.poses:
            self.get_logger().info('Path received. Switching to path following mode.')
            self.current_path = msg.poses
            self.path_index = 0

            # <<< 추가: 초기 주행 상태 비활성화 및 타이머 취소 >>>
            self.is_initial_drive = False
            if self.initial_drive_timer:
                self.initial_drive_timer.cancel()
            # <<< --- >>>

    def odom_callback(self, msg):
        """Odometry 정보를 받아 제어 명령을 계산하고 발행"""
        # <<< 수정: 초기 주행 상태에서는 odom 콜백이 제어하지 않도록 함 >>>
        if self.is_initial_drive or self.current_path is None:
            return
        # <<< --- >>>

        self.current_pose = msg.pose.pose
        current_position = self.current_pose.position

        lookahead_point, target_index = self.find_lookahead_point(current_position)

        if lookahead_point is None:
            self.publish_control_command(0.0, 0.0)
            self.get_logger().info('Reached the end of the path.')
            self.current_path = None
            return

        steering_angle = self.calculate_steering_angle(lookahead_point)
        current_velocity = TARGET_VELOCITY

        self.publish_control_command(current_velocity, steering_angle)

    # (이하 나머지 코드는 동일)
    def find_lookahead_point(self, current_position):
        """현재 위치에서 가장 가까운 경로점을 찾고, 그 점에서부터 전방 주시 거리만큼 떨어진 목표점을 찾음"""
        min_dist = float('inf')
        closest_index = self.path_index

        # 가장 가까운 경로점 찾기 (현재 인덱스부터 시작)
        for i in range(self.path_index, len(self.current_path)):
            dx = self.current_path[i].pose.position.x - current_position.x
            dy = self.current_path[i].pose.position.y - current_position.y
            dist = np.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        self.path_index = closest_index # 다음 검색 시 여기서부터 시작

        # 가장 가까운 점에서부터 전방 주시 거리(LOOKAHEAD_DISTANCE) 이상 떨어진 첫 번째 점을 찾음
        for i in range(closest_index, len(self.current_path)):
            dx = self.current_path[i].pose.position.x - current_position.x
            dy = self.current_path[i].pose.position.y - current_position.y
            dist = np.sqrt(dx**2 + dy**2)
            if dist >= LOOKAHEAD_DISTANCE:
                return self.current_path[i].pose.position, i

        # 경로 끝까지 그런 점이 없으면 경로의 마지막 점을 반환
        return self.current_path[-1].pose.position, len(self.current_path) - 1

    def calculate_steering_angle(self, target_point):
        """Pure Pursuit 알고리즘을 이용해 조향각 계산"""
        # 1. 차량 좌표계 기준으로 목표점의 위치 변환
        q = self.current_pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        dx = target_point.x - self.current_pose.position.x
        dy = target_point.y - self.current_pose.position.y

        rotated_x = dx * np.cos(-yaw) - dy * np.sin(-yaw)
        rotated_y = dx * np.sin(-yaw) + dy * np.cos(-yaw)

        # 2. 목표점까지의 각도(alpha) 계산
        alpha = np.arctan2(rotated_y, rotated_x)

        # 3. Pure Pursuit 공식으로 조향각 계산
        # delta = atan(2 * L * sin(alpha) / ld)
        # ld는 차량과 목표점 사이의 직선 거리
        ld = np.sqrt(dx**2 + dy**2)
        steering_angle = np.arctan2(2.0 * VEHICLE_LENGTH * np.sin(alpha), ld)

        # 최대 조향각 제한
        return np.clip(steering_angle, -MAX_STEER, MAX_STEER)

    def publish_control_command(self, velocity, steering_angle):
        """계산된 속도와 조향각을 Twist 메시지로 발행"""
        twist = Twist()
        twist.linear.x = velocity
        # 조향각(steering_angle)을 각속도(angular.z)로 변환
        # w = v / r = v * tan(delta) / L
        twist.angular.z = velocity * np.tan(steering_angle) / VEHICLE_LENGTH if steering_angle != 0 else 0.0
        self.cmd_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()