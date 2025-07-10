import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import LaneInfo, MotionCommand
import re

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion')

        # 구독
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)
        self.create_subscription(String, 'traffic_light_result', self.traffic_callback, 10)

        # 발행
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 변수
        self.angle_weight = 0.9
        self.position_weight = -0.05
        self.area_threshold = 11000

        self.target_lane = 2
        self.is_changing_lane = False  # 차선 전환 중 여부

    def traffic_callback(self, msg):
        if self.is_changing_lane:
            self.get_logger().info("🚧 차선 변경 중 — 신호등 무시")
            return

        data = msg.data
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)', data)

        if match:
            detected_str, area_str = match.groups()
            detected = detected_str.lower() == 'true'
            area = float(area_str)

            if detected and area > self.area_threshold:
                self.target_lane = 1 if self.target_lane == 2 else 2
                self.get_logger().info(f"🟢 신호등 감지됨 (Area: {area:.2f}) → target_lane 변경: {self.target_lane}")

    def lane_info_callback(self, msg):
        current_lane = msg.lane_num
        steering_angle = msg.steering_angle
        vehicle_position_x = msg.vehicle_position_x

        self.is_changing_lane = (current_lane != self.target_lane)

        if self.is_changing_lane:
            steering_value = -10 if self.target_lane == 1 else 10
            self.get_logger().info(f"차선 변경 중 → 현재 {current_lane}, 목표 {self.target_lane} → steering {steering_value}")
        else:
            mapped_angle = (steering_angle / 60.0) * 10.0 * self.angle_weight
            position_adjustment = -vehicle_position_x * self.position_weight
            steering_value = mapped_angle + position_adjustment
            steering_value = max(-10, min(steering_value, 10))

            self.get_logger().info(
                f"Target Lane: {self.target_lane} | "
                f"Mapped Angle: {mapped_angle:.2f}, "
                f"Position Adj: {position_adjustment:.2f}, "
                f"Steering: {steering_value:.2f}"
            )

        motion_command = MotionCommand()
        motion_command.steering = int(steering_value)
        motion_command.left_speed = 60
        motion_command.right_speed = 60
        self.motion_pub.publish(motion_command)

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MotionNode has been interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
