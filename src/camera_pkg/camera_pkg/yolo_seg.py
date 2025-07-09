
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
from interfaces_pkg.msg import Detection, DetectionArray, Mask, Point2D
from interfaces_pkg.msg import BoundingBox2D

import os


class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg')
        #self.get_logger().info("🟠 image_raw 구독 설정 완료")

        # 현재 스크립트 파일의 디렉토리를 기준으로 상대경로 설정
        default_model_path = "/home/sg/contest_ws/src/camera_pkg/camera_pkg/model/best.pt"
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.3)

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value

        self.bridge = CvBridge()
        self.model = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.publisher = self.create_publisher(DetectionArray, "detections", qos)
        self.subscription = self.create_subscription(Image, "image_raw", self.image_cb, qos)

        # 모델 로드
        if not os.path.exists(self.model_path):
            #self.get_logger().error(f"❌ Model not found: {self.model_path}")
            return

        try:
            self.model = YOLO(self.model_path)
            self.model.fuse()
            self.get_logger().info(f"✅ Model loaded from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            return

        #self.get_logger().info("🟢 YoloSegNode is ready and running")

    def image_cb(self, msg: Image):
        #self.get_logger().info("🟢 image_cb 호출됨")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        


        try:
            results = self.model.predict(
                source=cv_image,
                device=self.device,
                conf=self.threshold,
                stream=False,
                verbose=False
            )[0].cpu()
            
            #self.get_logger().info(f"📦 결과 box 수: {len(results.boxes)}")

            detections_msg = DetectionArray()
            detections_msg.header.stamp = self.get_clock().now().to_msg()
            detections_msg.header.frame_id = msg.header.frame_id

            for i in range(len(results.boxes)):
                det = Detection()
                det.class_id = int(results.boxes[i].cls)
                det.class_name = self.model.names[det.class_id]
                det.score = float(results.boxes[i].conf)

                #self.get_logger().info(f"🧾 Detected class: {det.class_name}")

                bbox_xywh = results.boxes.xywh[i]
                det.bbox = BoundingBox2D()
                det.bbox.center.position.x = float(bbox_xywh[0])
                det.bbox.center.position.y = float(bbox_xywh[1])
                det.bbox.size.x = float(bbox_xywh[2])
                det.bbox.size.y = float(bbox_xywh[3])

                if results.masks is not None and i < len(results.masks.xy):
                    mask_msg = Mask()
                    mask_msg.width = results.orig_shape[1]
                    mask_msg.height = results.orig_shape[0]
                    for pt in results.masks.xy[i].tolist():
                        p = Point2D()
                        p.x = float(pt[0])
                        p.y = float(pt[1])
                        mask_msg.data.append(p)
                    det.mask = mask_msg

                

                detections_msg.detections.append(det)

            #self.get_logger().info(f"📤 Publishing {len(detections_msg.detections)} detections")
            self.publisher.publish(detections_msg)

        except Exception as e:
            #self.get_logger().error(f"❌ YOLO Segmentation failed: {e}")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = YoloSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


