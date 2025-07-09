import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, LaneInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
import warnings

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.declare_parameter('camera_topic', 'image_raw')
        self.declare_parameter('detection_topic', 'detections')
        self.declare_parameter('persp_src', [0.1, 1.0, 0.9, 1.0, 0.6, 0.4, 0.4, 0.4])
        self.declare_parameter('persp_dst', [0.3, 1.0, 0.7, 1.0, 0.7, 0.0, 0.3, 0.0])
        self.declare_parameter('morph_kernel_size', 11)
        self.declare_parameter('canny_thresholds', [50, 150])
        self.declare_parameter('expected_lane_num', 2)

        camera_topic = self.get_parameter('camera_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.bridge = CvBridge()
        image_sub = Subscriber(self, Image, camera_topic, qos_profile=qos)
        det_sub = Subscriber(self, DetectionArray, detection_topic, qos_profile=qos)
        self.ts = ApproximateTimeSynchronizer([image_sub, det_sub], queue_size=10, slop=0.3)
        self.ts.registerCallback(self.sync_callback)

        self.viz_pub = self.create_publisher(Image, 'lane_viz', qos)
        self.debug_pub = self.create_publisher(Image, 'lane_debug', qos)

        self.get_logger().info(f"LaneDetector started on '{camera_topic}' and '{detection_topic}'")

    def get_perspective_matrix(self, shape):
        h, w = shape[:2]
        src = self.get_parameter('persp_src').value
        dst = self.get_parameter('persp_dst').value
        src_pts = np.float32([[src[i]*w, src[i+1]*h] for i in range(0, 8, 2)])
        dst_pts = np.float32([[dst[i]*w, dst[i+1]*h] for i in range(0, 8, 2)])
        return cv2.getPerspectiveTransform(src_pts, dst_pts)

    def preprocess_mask(self, mask_img):
        gray = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
        k = self.get_parameter('morph_kernel_size').value
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        opened = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        blur = cv2.GaussianBlur(closed, (k, k), 0)
        thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        t_low, t_high = self.get_parameter('canny_thresholds').value
        edges = cv2.Canny(thresh, t_low, t_high)
        return gray, edges

    def sync_callback(self, img_msg, det_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]

            M = self.get_perspective_matrix(frame.shape)
            bird = cv2.warpPerspective(frame, M, (w, h), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

            mask = np.zeros_like(bird)
            for det in det_msg.detections:
                if det.class_name != 'lane2':
                    continue
                pts = np.array([[p.x, p.y] for p in det.mask.data], dtype=np.float32)
                if pts.shape[0] < 3:
                    continue
                wp = cv2.perspectiveTransform(pts[None, :, :], M)[0].astype(int)
                cv2.fillPoly(mask, [wp], (0, 255, 255))

            gray, edges = self.preprocess_mask(mask)

            # 양쪽 경계 사이를 얇게 흰색으로 채우기
            line_patch = np.zeros_like(edges)
            for y in range(edges.shape[0]):
                xs = np.where(edges[y] > 0)[0]
                if xs.size >= 2:
                    x_left = xs[0]
                    x_right = xs[-1]
                    center_x = int((x_left + x_right) / 2)
                    line_patch[y, center_x-1:center_x+2] = 255  # 얇은 선으로 채움 (3픽셀 폭)

            # 디버그 이미지: grayscale | edges | 가운데 채운 선
            gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            edge_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            patch_bgr = cv2.cvtColor(line_patch, cv2.COLOR_GRAY2BGR)
            debug_stack = np.hstack([gray_bgr, edge_bgr, patch_bgr])
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_stack, 'bgr8'))

            # 시각화에 적용
            vis = bird.copy()
            vis[line_patch == 255] = [0, 255, 0]  # 초록색 선
            self.viz_pub.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))

        except Exception as e:
            self.get_logger().error(f"Error in sync_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LaneDetector interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

