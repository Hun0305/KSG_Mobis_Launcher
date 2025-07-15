import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

# 주차 공간으로 판단할 최소, 최대 폭 (미터 단위)
MIN_PARKING_SPACE_WIDTH = 0.7  # 예: 차량 폭보다 넓은 최소 공간
MAX_PARKING_SPACE_WIDTH = 1.5   # 예: 너무 넓으면 주차 공간이 아닐 수 있음

# 장애물로 판단할 포인트 군집의 최소 개수
MIN_CLUSTER_SIZE = 5

# 포인트를 같은 군집으로 묶기 위한 최대 거리
CLUSTERING_DISTANCE_THRESHOLD = 0.1 # 10cm

class ParkingPerception(Node):
    def __init__(self):
        super().__init__('parking_perception')

        # LiDAR 스캔 데이터 구독
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # 초음파 센서 데이터 구독 (센서 개수만큼 동적으로 생성)
        self.num_ultrasonic_sensors = 10 # ultrasonic_publisher와 개수 일치
        self.ultrasonic_subscriptions = []
        self.ultrasonic_ranges = [None] * self.num_ultrasonic_sensors
        for i in range(self.num_ultrasonic_sensors):
            self.ultrasonic_subscriptions.append(self.create_subscription(
                Range,
                f'/ultrasonic/sensor_{i}',
                lambda msg, index=i: self.ultrasonic_callback(msg, index),
                10))

        # 인식된 장애물과 주차 공간을 RViz에서 시각화하기 위한 퍼블리셔
        self.marker_publisher = self.create_publisher(MarkerArray, '/parking_visualization', 10)

        self.get_logger().info('Parking Perception Node has been started.')

    def ultrasonic_callback(self, msg, index):
        """초음파 센서 데이터를 수신하여 리스트에 저장"""
        self.ultrasonic_ranges[index] = msg.range
        # self.get_logger().debug(f'Received ultrasonic data from sensor {index}: {msg.range}m')

    def scan_callback(self, msg):
        """LiDAR 데이터를 받아 장애물과 주차 공간을 탐지"""
        points = self.laser_scan_to_points(msg)
        
        # 1. 포인트들을 군집화하여 장애물 후보군 찾기
        clusters = self.cluster_points(points)

        # 2. 군집화된 장애물들을 Marker로 변환
        obstacle_markers = self.create_obstacle_markers(clusters, msg.header)

        # 3. 장애물 사이의 빈 공간(주차 공간 후보) 찾기
        parking_space_candidates = self.find_parking_spaces(clusters, msg.header)

        # 4. 주차 공간 후보들을 Marker로 변환
        parking_space_markers = self.create_parking_space_markers(parking_space_candidates, msg.header)

        # 장애물과 주차 공간 마커를 합쳐서 발행
        marker_array = MarkerArray()
        marker_array.markers.extend(obstacle_markers)
        marker_array.markers.extend(parking_space_markers)
        
        # 이전 마커들을 삭제하기 위한 설정 추가
        # ID가 겹치지 않게 장애물은 0~999, 주차공간은 1000~1999로 설정
        # 새로운 마커가 들어올 때 이전 마커들을 지우기 위해 모든 마커를 삭제하는 마커를 먼저 추가
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[delete_marker]))
        
        if marker_array.markers:
            self.marker_publisher.publish(marker_array)

    def laser_scan_to_points(self, msg):
        """LaserScan 메시지를 (x, y) 좌표 리스트로 변환"""
        points = []
        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                points.append((x, y))
        return points

    def cluster_points(self, points):
        """유클리드 거리 기반으로 포인트들을 군집화"""
        clusters = []
        visited = [False] * len(points)
        for i, point in enumerate(points):
            if not visited[i]:
                visited[i] = True
                new_cluster = [point]
                queue = [i]
                
                head = 0
                while head < len(queue):
                    current_point_index = queue[head]
                    head += 1
                    
                    for j, other_point in enumerate(points):
                        if not visited[j]:
                            dist = np.linalg.norm(np.array(points[current_point_index]) - np.array(other_point))
                            if dist < CLUSTERING_DISTANCE_THRESHOLD:
                                visited[j] = True
                                new_cluster.append(other_point)
                                queue.append(j)
                
                if len(new_cluster) > MIN_CLUSTER_SIZE:
                    clusters.append(new_cluster)
        return clusters

    def find_parking_spaces(self, clusters, header):
        """군집(장애물)들 사이의 빈 공간을 찾아 주차 공간 후보로 반환"""
        # 간단한 구현: x축 기준으로 정렬된 장애물의 중심점 사이의 거리를 확인
        if len(clusters) < 2:
            return []

        # 각 클러스터의 중심점 계산
        cluster_centers = [np.mean(cluster, axis=0) for cluster in clusters]
        # x좌표 기준으로 정렬
        sorted_centers = sorted(cluster_centers, key=lambda p: p[0])

        parking_spaces = []
        for i in range(len(sorted_centers) - 1):
            p1 = sorted_centers[i]
            p2 = sorted_centers[i+1]
            
            # 두 장애물 중심점 사이의 거리 계산
            distance = np.linalg.norm(p1 - p2)
            
            if MIN_PARKING_SPACE_WIDTH < distance < MAX_PARKING_SPACE_WIDTH:
                # 주차 공간의 중심점
                space_center_x = (p1[0] + p2[0]) / 2
                space_center_y = (p1[1] + p2[1]) / 2
                
                # TODO: 초음파 센서 데이터를 활용하여 해당 공간이 정말 비어있는지 추가 확인 로직 구현 가능
                # 예: 주차 공간 방향의 초음파 센서 값이 일정 거리 이상인가?

                parking_spaces.append({
                    'center': (space_center_x, space_center_y),
                    'width': distance,
                    'depth': 1.0 # 깊이는 일단 1m로 가정
                })
        return parking_spaces

    def create_obstacle_markers(self, clusters, header):
        """장애물 군집을 RViz의 CUBE_LIST 타입 마커로 변환"""
        markers = []
        marker = Marker()
        marker.header = header
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for cluster in clusters:
            for x, y in cluster:
                p = Point()
                p.x, p.y, p.z = x, y, 0.05
                marker.points.append(p)
        markers.append(marker)
        return markers

    def create_parking_space_markers(self, spaces, header):
        """주차 공간 후보를 RViz의 CUBE 타입 마커로 변환"""
        markers = []
        for i, space in enumerate(spaces):
            marker = Marker()
            marker.header = header
            marker.ns = "parking_spaces"
            marker.id = 1000 + i # 장애물 ID와 겹치지 않게
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            center_x, center_y = space['center']
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = space['width']
            marker.scale.y = space['depth'] # y방향 크기
            marker.scale.z = 0.4

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5 # 반투명
            markers.append(marker)
        return markers

def main(args=None):
    rclpy.init(args=args)
    node = ParkingPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()