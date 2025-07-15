import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf_transformations
# tf2 관련 라이브러리 임포트
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # --- TF 리스너 초기화 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 주차 공간 시각화 데이터를 구독
        self.subscription = self.create_subscription(
            MarkerArray,
            '/parking_visualization',
            self.parking_space_callback,
            10)

        # 생성된 경로를 발행
        self.path_publisher = self.create_publisher(Path, '/global_path', 10)

        self.path_planned = False
        self.get_logger().info('Path Planner Node has been started.')

    def parking_space_callback(self, msg):
        if self.path_planned:
            return

        parking_spaces = [m for m in msg.markers if m.ns == "parking_spaces"]
        if not parking_spaces:
            return

        target_space = parking_spaces[0]
        
        # --- ★★★ 핵심 수정 부분 ★★★ ---
        # 1. 현재 로봇의 위치를 tf에서 조회
        try:
            # 'map' 좌표계를 기준으로 'base_link'(로봇 베이스)의 현재 위치와 방향을 가져옴
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            current_pose = PoseStamped()
            current_pose.header.stamp = self.get_clock().now().to_msg()
            current_pose.header.frame_id = 'map'
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.orientation = trans.transform.rotation
        except Exception as e:
            self.get_logger().error(f'Could not get transform from map to base_link: {e}')
            return
        # --- ★★★ 여기까지 ★★★ ---

        path = self.generate_path(current_pose, target_space.pose)

        if path:
            self.path_publisher.publish(path)
            self.get_logger().info('Global path has been published.')
            self.path_planned = True
            self.destroy_subscription(self.subscription)

    def generate_path(self, start_pose, target_pose):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        # 2. 경로의 기준 좌표계를 'map'으로 변경
        path_msg.header.frame_id = 'map' 

        # 1. 시작점 (이제 (0,0)이 아닌, tf에서 조회한 현재 위치)
        # start_pose가 인자로 들어오므로 별도 생성 필요 없음

        # 2. 중간 경유지
        waypoint = PoseStamped()
        waypoint.header = path_msg.header
        waypoint.pose.position.x = target_pose.position.x - 1.0
        waypoint.pose.position.y = target_pose.position.y
        q = tf_transformations.quaternion_from_euler(0, 0, np.pi / 2)
        waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z, waypoint.pose.orientation.w = q

        # 3. 최종 목표점
        final_pose = PoseStamped()
        final_pose.header = path_msg.header
        final_pose.pose = target_pose
        final_pose.pose.orientation = waypoint.pose.orientation

        # 경로점 추가
        path_msg.poses.append(start_pose) # 하드코딩된 (0,0) 대신 실제 시작 위치 추가
        path_msg.poses.append(waypoint)
        path_msg.poses.append(final_pose)

        return path_msg

# ... (main 함수는 변경 없음)