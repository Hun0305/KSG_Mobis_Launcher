import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle')

        self.bridge = CvBridge()
        self.last_status = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.image_sub = Subscriber(self, Image, '/cam1/image_raw', qos_profile=qos)
        self.detection_sub = Subscriber(self, DetectionArray, '/cam1/detections', qos_profile=qos)
        
        
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub],
            queue_size=10,
            slop=0.5
        )
        self.ts.registerCallback(self.sync_callback)

        self.pub = self.create_publisher(String, 'obstacle_result', qos)

    def sync_callback(self, img_msg, det_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        detected, area = self.detect_obstacle(cv_img, det_msg)

        status_msg = f"Detected: {detected}, Area: {area}"
        msg = String()
        msg.data = status_msg
        self.pub.publish(msg)
        self.last_status = status_msg

    def detect_obstacle(self, image, detections: DetectionArray):
        for det in detections.detections:
            if det.class_name == 'car':
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                w = int(det.bbox.size.x)
                h = int(det.bbox.size.y)

                area = w * h
                return True, area  # 감지된 경우

        return False, 0  # 감지 안 된 경우


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
