import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, LaneInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
import math

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.declare_parameter('camera_topic', 'image_raw')
        self.declare_parameter('detection_topic', 'detections')
        self.declare_parameter('persp_src', [0.1, 1.0, 0.9, 1.0, 0.6, 0.4, 0.4, 0.4])
        self.declare_parameter('persp_dst', [0.3, 1.0, 0.7, 1.0, 0.7, 0.0, 0.3, 0.0])
        self.declare_parameter('morph_kernel_size', 11)
        self.declare_parameter('canny_thresholds', [50, 150])

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
        self.lane_pub = self.create_publisher(LaneInfo, 'lane_info', qos)

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
        t_low, t_high = self.get_parameter('canny_thresholds').value
        edges = cv2.Canny(blur, t_low, t_high)
        return gray, edges

    def extract_centerline(self, edges):
        centerline = np.zeros_like(edges)
        points = []

        for y in range(edges.shape[0]):
            xs = np.where(edges[y] > 0)[0]
            if xs.size >= 2:
                x_left = xs[0]
                x_right = xs[-1]
                center_x = int((x_left + x_right) / 2)
                centerline[y, center_x-1:center_x+2] = 255
                points.append((center_x, y))

        return centerline, points

    def compute_angle(self, points):
        if len(points) < 2:
            return 0
        [vx, vy, _, _] = cv2.fitLine(np.array(points), cv2.DIST_L2, 0, 0.01, 0.01)
        angle_rad = math.atan2(vy, vx)
        return int(np.degrees(angle_rad))

    def sync_callback(self, img_msg, det_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]
            M = self.get_perspective_matrix(frame.shape)
            bird = cv2.warpPerspective(frame, M, (w, h), borderMode=cv2.BORDER_CONSTANT)

            masks = {'lane1': np.zeros_like(bird), 'lane2': np.zeros_like(bird)}
            for det in det_msg.detections:
                if det.class_name not in ['lane1', 'lane2']:
                    continue
                pts = np.array([[p.x, p.y] for p in det.mask.data], dtype=np.float32)
                if pts.shape[0] < 3:
                    continue
                wp = cv2.perspectiveTransform(pts[None, :, :], M)[0].astype(int)
                cv2.fillPoly(masks[det.class_name], [wp], (0, 255, 255))

            results = {}
            for name, mask in masks.items():
                gray, edges = self.preprocess_mask(mask)
                centerline, points = self.extract_centerline(edges)
                results[name] = {'centerline': centerline, 'points': points, 'gray': gray, 'edges': edges}

            vehicle_x = w // 2
            vehicle_y = h - 10
            min_dist = float('inf')
            closest = None
            for name, data in results.items():
                if not data['points']:
                    continue
                dists = [abs(x - vehicle_x) + abs(y - vehicle_y) for x, y in data['points']]
                if dists:
                    dist = min(dists)
                    if dist < min_dist:
                        min_dist = dist
                        closest = name

            if closest:
                angle = self.compute_angle(results[closest]['points'])
                lane_num = 1 if closest == 'lane1' else 2

                bottom_xs = [x for x, y in results[closest]['points'] if abs(y - vehicle_y) < 5]
                if bottom_xs:
                    avg_lane_x = int(np.mean(bottom_xs))
                    vehicle_offset = avg_lane_x - vehicle_x
                else:
                    vehicle_offset = 0
                
                #뽑아내야되는정보^^
                msg = LaneInfo()
                msg.steering_angle = angle
                msg.lane_num = lane_num
                msg.vehicle_position_x = vehicle_offset
                self.lane_pub.publish(msg)

            # 시각화: lane1 + lane2 모두 보여주기
            vis = bird.copy()
            for name in ['lane1', 'lane2']:
                if name in results:
                    vis[results[name]['centerline'] == 255] = [0, 255, 0]
            self.viz_pub.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))

            # 디버그 이미지: 각 lane의 gray, edge, centerline 보여주기
            debug_rows = []
            for name in ['lane1', 'lane2']:
                if name in results:
                    g = cv2.cvtColor(results[name]['gray'], cv2.COLOR_GRAY2BGR)
                    e = cv2.cvtColor(results[name]['edges'], cv2.COLOR_GRAY2BGR)
                    c = cv2.cvtColor(results[name]['centerline'], cv2.COLOR_GRAY2BGR)
                    debug_rows.append(np.hstack([g, e, c]))
            if debug_rows:
                debug = np.vstack(debug_rows)
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
