import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

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
        # Fraction of image width defining the central region (0.0–1.0)
        self.declare_parameter('center_region_ratio', 0.3) # 인식 영역 판단

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
        self.last_status = None

    def sync_callback(self, img_msg, det_msg):
        # convert ROS Image to OpenCV image
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # compute central region bounds
        ratio = self.get_parameter('center_region_ratio').value
        width = cv_img.shape[1]
        x_min = int(width * (0.5 - ratio / 2))
        x_max = int(width * (0.5 + ratio / 2))

        # detect obstacle and get its center x-coordinate
        detected, area, cx = self.detect_obstacle(det_msg)

        # only count as detected if within central region
        if detected and cx is not None and x_min <= cx <= x_max:
            detected_flag = True
            final_area = area
        else:
            detected_flag = False
            final_area = 0

        # publish result
        status_msg = f"Detected: {detected_flag}, Area: {final_area}"
        msg = String()
        msg.data = status_msg
        self.pub.publish(msg)
        self.last_status = status_msg

    def detect_obstacle(self, detections: DetectionArray):
        # look for any 'car' in detections
        for det in detections.detections:
            if det.class_name == 'car':
                cx = int(det.bbox.center.position.x)
                w = int(det.bbox.size.x)
                h = int(det.bbox.size.y)
                area = w * h
                return True, area, cx
        return False, 0, None


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
