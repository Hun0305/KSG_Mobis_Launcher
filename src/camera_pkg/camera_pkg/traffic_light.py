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


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')

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

        self.pub = self.create_publisher(String, 'traffic_light_result', qos)

        self.hsv_ranges = {
            'red1': (np.array([0, 100, 95]), np.array([10, 255, 255])),
            'red2': (np.array([160, 100, 95]), np.array([179, 255, 255])),
            'yellow': (np.array([20, 100, 95]), np.array([30, 255, 255])),
            'green': (np.array([40, 100, 95]), np.array([90, 255, 255]))
        }

    def sync_callback(self, img_msg, det_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        detected, area, color = self.detect_traffic_light_color(cv_img, det_msg)

        status_msg = f"Detected: {detected}, Area: {area}, Color: {color}"
        
        msg = String()
        msg.data = status_msg
        self.pub.publish(msg)
        self.last_status = status_msg

    def detect_traffic_light_color(self, image, detections: DetectionArray):
        for det in detections.detections:
            if det.class_name == 'traffic_light':
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                w = int(det.bbox.size.x)
                h = int(det.bbox.size.y)

                x1 = max(cx - w // 2, 0)
                y1 = max(cy - h // 2, 0)
                x2 = min(cx + w // 2, image.shape[1])
                y2 = min(cy + h // 2, image.shape[0])

                roi = image[y1:y2, x1:x2]
                if roi is None or roi.size == 0:
                    self.get_logger().warn("⚠️ Empty ROI, skipping detection")
                    continue

                try:
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                except Exception as e:
                    self.get_logger().warn(f"⚠️ cvtColor failed: {e}")
                    continue

                # 색상 마스크 생성
                red_mask = (
                    cv2.inRange(hsv, self.hsv_ranges['red1'][0], self.hsv_ranges['red1'][1]) +
                    cv2.inRange(hsv, self.hsv_ranges['red2'][0], self.hsv_ranges['red2'][1])
                )
                yellow_mask = cv2.inRange(hsv, self.hsv_ranges['yellow'][0], self.hsv_ranges['yellow'][1])
                green_mask = cv2.inRange(hsv, self.hsv_ranges['green'][0], self.hsv_ranges['green'][1])

                red_area = np.sum(red_mask > 0)
                yellow_area = np.sum(yellow_mask > 0)
                green_area = np.sum(green_mask > 0)

                # 가장 넓은 색상 선택
                max_area = max(red_area, yellow_area, green_area)
                if max_area == 0:
                    color = 'unknown'
                elif max_area == red_area:
                    color = 'red'
                elif max_area == yellow_area:
                    color = 'yellow'
                else:
                    color = 'green'

                area = w * h
                return True, area, color

        return False, 0, 'none'


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
