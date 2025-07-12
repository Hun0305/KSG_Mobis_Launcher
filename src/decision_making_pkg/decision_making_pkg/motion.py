import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import LaneInfo, MotionCommand
import re

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion')

        # Subscriptions
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)
        self.create_subscription(String, 'traffic_light_result', self.traffic_callback, 10)

        # Publisher
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # Control weights and thresholds
        self.angle_weight = 0.7
        self.position_weight = 0.05
        self.area_threshold = 11000

        # Lane state variables
        self.current_lane = None      # 현재 차선 번호
        self.target_lane = 2          # 목표 차선은 2로 시작
        self.is_changing_lane = False # 차선 변경 중 플래그

    def traffic_callback(self, msg):
        # Ignore traffic signals during lane change
        if self.is_changing_lane:
            return

        # Parse traffic light result
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)', msg.data)
        if not match:
            return
        detected = match.group(1).lower() == 'true'
        area = float(match.group(2))

        # Toggle target lane if detected and large enough
        if detected and area > self.area_threshold:
            old = self.target_lane
            self.target_lane = 1 if self.target_lane == 2 else 2
            self.is_changing_lane = True
            self.get_logger().info(f"🟢 Traffic Light triggered: Area={area:.2f} → target_lane {old}→{self.target_lane}")

    def lane_info_callback(self, msg):
        # Update current lane
        self.current_lane = msg.lane_num
        steering_angle = msg.steering_angle
        vehicle_position_x = msg.vehicle_position_x

        # Check if lane change just completed
        if self.is_changing_lane and self.current_lane == self.target_lane:
            self.is_changing_lane = False
            self.get_logger().info(f"✅ Lane change complete: now on lane {self.current_lane}")

        # Determine steering command
        if self.is_changing_lane:
            # Fixed steering during lane change
            steering_value = -5 if self.target_lane == 1 else 5
        else:
            # Compute steering based on angle and position
            mapped = (steering_angle / 50.0) * 10.0 * self.angle_weight
            adjust = -vehicle_position_x * self.position_weight
            steering_value = max(-10, min(mapped + adjust, 10))

        # Publish motion command
        cmd = MotionCommand()
        cmd.steering = int(steering_value)
        cmd.left_speed = 255
        cmd.right_speed = 255
        self.motion_pub.publish(cmd)

        # Debug log
        self.get_logger().info(
            f"현재 차선: {self.current_lane}, 목표 차선: {self.target_lane}, steering: {steering_value:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MotionNode 인터럽트됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
