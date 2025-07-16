import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import subprocess
import os

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar')

        # TF 브로드캐스터
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # /scan → /lidar_raw 리맵 받기
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # /lidar_raw 퍼블리셔
        self.publisher = self.create_publisher(
            LaserScan,
            'lidar_raw',
            10
        )

        # TF 주기적 발행 (50Hz)
        self.timer = self.create_timer(0.02, self.send_tf)

        # rplidar_ros 드라이버 실행 (출력 무시 → buffer overflow 방지)
        self.driver = subprocess.Popen([
            'ros2', 'run', 'rplidar_ros', 'rplidar_composition',
            '--ros-args',
            '-p', 'serial_port:=/dev/ttyUSB0',
            '-p', 'serial_baudrate:=115200',
            '-p', 'frame_id:=laser_frame',
            '-p', 'inverted:=false',
            '-p', 'angle_compensate:=true'
        ])


        self.get_logger().info('Started rplidar_composition node')

    def lidar_callback(self, msg):
        msg.header.frame_id = 'laser_frame'

        # 3m 이상 거리값은 무시 (inf 처리)
        msg.ranges = [r if 0.0 < r <= 3.0 else float('inf') for r in msg.ranges]
        self.publisher.publish(msg)

    def send_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.get_logger().info('Shutting down lidar...')
        self.driver.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
