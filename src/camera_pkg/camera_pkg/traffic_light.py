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


class MultiTrafficLightNode(Node):
    def __init__(self):
        super().__init__('multi_traffic_light_node')

        self.bridge = CvBridge()
        self.last_status = None  # 🟡 마지막 상태 저장 변수 추가

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.image0_sub = Subscriber(self, Image, '/cam0/image_raw', qos_profile=qos)
        self.detection0_sub = Subscriber(self, DetectionArray, '/cam0/detections', qos_profile=qos)
        self.image1_sub = Subscriber(self, Image, '/cam1/image_raw', qos_profile=qos)
        self.detection1_sub = Subscriber(self, DetectionArray, '/cam1/detections', qos_profile=qos)

        self.ts = ApproximateTimeSynchronizer(
            [self.image0_sub, self.detection0_sub, self.image1_sub, self.detection1_sub],
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

    def sync_callback(self, img0, det0, img1, det1):
        cv_img0 = self.bridge.imgmsg_to_cv2(img0, desired_encoding='bgr8')
        cv_img1 = self.bridge.imgmsg_to_cv2(img1, desired_encoding='bgr8')

        status0 = self.detect_traffic_light_color(cv_img0, det0)
        status1 = self.detect_traffic_light_color(cv_img1, det1)

        if status0 == 'Red' or status1 == 'Red':
            result = 'Red'
        elif status0 == 'None' and status1 == 'None':
            result = 'None'
        else:
            result = 'Not_Red'

        # ✅ 상태가 바뀌었을 때만 로그 출력
        if result != self.last_status:
            msg = String()
            msg.data = result
            self.pub.publish(msg)
            self.get_logger().info(f'🟢 Traffic Light Status changed: {result}')
            self.last_status = result

    def detect_traffic_light_color(self, image, detections: DetectionArray):
        for det in detections.detections:
            if det.class_name == 'tarffic_light':
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

                masks = {
                    k: cv2.inRange(hsv, self.hsv_ranges[k][0], self.hsv_ranges[k][1])
                    for k in self.hsv_ranges
                }

                red_mask = masks['red1'] + masks['red2']
                if cv2.countNonZero(red_mask) > 50:
                    return 'Red'
                return 'Not_Red'

        return 'None'


def main(args=None):
    rclpy.init(args=args)
    node = MultiTrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


