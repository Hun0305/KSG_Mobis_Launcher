import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import tf_transformations

# --- 튜닝 파라미터 ---
LOOKAHEAD_DISTANCE = 1.0  # 전방 주시 거리 (m) - 경로상의 얼마나 앞 포인트를 볼 것인가
VEHICLE_LENGTH = 0.3      # 차량 축거 (m) - 차량의 뒷바퀴와 앞바퀴 사이의 거리
MAX_STEER = np.radians(25)  # 최대 조향각 (radians)
TARGET_VELOCITY = 0.2     # 목표 주행 속도 (m/s)

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')

        # 경로와 현재 위치(Odometry)를 구독
        self.path_subscription = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            1) # 경로는 한 번만 받으면 되므로 QoS=1
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Odometry 토픽 이름 (실제 사용하는 토픽으로 변경 필요)
            self.odom_callback,
            10)

        # 차량 제어 명령 (Twist)을 발행
        # control_pkg의 control.py가 이 토픽을 구독하여 아두이노로 전달해야 함
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_path = None
        self.current_pose = None
        self.path_index = 0

        self.get_logger().info('Vehicle Controller Node has been started.')

    def path_callback(self, msg):
        """경로를 수신하여 저장"""
        if not self.current_path: # 경로를 한 번만 저장
            self.get_logger().info('Path received.')
            self.current_path = msg.poses
            self.path_index = 0

    def odom_callback(self, msg):
        """Odometry 정보를 받아 제어 명령을 계산하고 발행"""
        # 경로가 없거나, 차량의 현재 위치를 모르면 아무것도 하지 않음
        if self.current_path is None:
            return

        self.current_pose = msg.pose.pose
        current_position = self.current_pose.position
        
        # Pure Pursuit 알고리즘 실행
        lookahead_point, target_index = self.find_lookahead_point(current_position)
        
        if lookahead_point is None:
            # 경로의 끝에 도달하면 정지
            self.publish_control_command(0.0, 0.0)
            self.get_logger().info('Reached the end of the path.')
            self.current_path = None # 경로 초기화
            return

        # 조향각과 속도 계산
        steering_angle = self.calculate_steering_angle(lookahead_point)
        current_velocity = TARGET_VELOCITY # 현재는 등속 주행

        # 제어 명령 발행
        self.publish_control_command(current_velocity, steering_angle)

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