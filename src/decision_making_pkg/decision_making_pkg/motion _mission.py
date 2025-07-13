import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import LaneInfo, MotionCommand
import re

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_mission')

        # Subscriptions
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)
        self.create_subscription(String, 'traffic_light_result', self.traffic_callback, 10)
        self.create_subscription(String, 'obstacle_result', self.obstacle_callback, 10)

        # Publisher
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 장애물 기준값 및 연속 감지/해제 횟수
        self.obstacle_area_threshold       = 12000
        self.obstacle_required_count       = 5
        self.obstacle_clear_required_count = 5
        self.obstacle_detect_counter       = 0
        self.obstacle_clear_counter        = 0
        self.wait_for_obstacle_clear       = False

        # 신호등 기준값
        self.traffic_area_threshold = 10000  # 필요에 따라 조절

        # 상태 변수
        self.is_changing_lane = False
        self.wait_for_red_clear = False
        self.current_lane      = None
        self.target_lane       = 2

        # lane 1 설정
        self.lane1_angle_weight      = 0.8
        self.lane1_position_weight   = 0.05
        self.lane1_normal_speed      = 255
        self.lane1_lane_change_speed = 200

        # lane 2 설정
        self.lane2_angle_weight      = 0.7
        self.lane2_position_weight   = 0.05
        self.lane2_normal_speed      = 255
        self.lane2_lane_change_speed = 200

    def get_lane_config(self):
        if self.current_lane == 1:
            return (
                self.lane1_angle_weight,
                self.lane1_position_weight,
                self.lane1_normal_speed,
                self.lane1_lane_change_speed
            )
        else:
            return (
                self.lane2_angle_weight,
                self.lane2_position_weight,
                self.lane2_normal_speed,
                self.lane2_lane_change_speed
            )

    def obstacle_callback(self, msg: String):
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)', msg.data)
        if not match:
            return

        detected = match.group(1).lower() == 'true'
        area     = float(match.group(2))
        self.get_logger().info(f"🚧 Obstacle: detected={detected}, area={area}")

        if detected and area > self.obstacle_area_threshold:
            self.obstacle_clear_counter = 0
            self.obstacle_detect_counter += 1

            if not self.wait_for_obstacle_clear and self.obstacle_detect_counter >= self.obstacle_required_count:
                self.target_lane       = 1 if self.target_lane == 2 else 2
                self.is_changing_lane  = True
                self.wait_for_obstacle_clear = True
                self.get_logger().info(f"🎯 Obstacle triggered lane switch to {self.target_lane}")
        else:
            self.obstacle_detect_counter = 0
            if self.wait_for_obstacle_clear:
                self.obstacle_clear_counter += 1
                if self.obstacle_clear_counter >= self.obstacle_clear_required_count:
                    self.wait_for_obstacle_clear = False
                    self.obstacle_clear_counter = 0
                    self.get_logger().info("🟢 Obstacle cleared: allow next lane switch")

    def traffic_callback(self, msg: String):
        # "Detected: True, Area: 1234, Color: red"
        match = re.match(
            r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)(?:,\s*Color:\s*(\w+))?',
            msg.data
        )
        if not match:
            return

        detected = match.group(1).lower() == 'true'
        area     = float(match.group(2))
        color    = (match.group(3) or '').lower()

        self.get_logger().info(f"🚦 Traffic: detected={detected}, area={area}, color={color}")

        # 빨간 불이 감지되고 크기가 임계 이상이면 정지
        if detected and area > self.traffic_area_threshold and color == 'red':
            cmd = MotionCommand()
            cmd.left_speed  = 0
            cmd.right_speed = 0
            cmd.steering    = 0
            self.motion_pub.publish(cmd)
            self.wait_for_red_clear = True
            self.get_logger().warning("⛔⛔⛔⛔⛔⛔⛔⛔⛔⛔⛔⛔⛔⛔⛔⛔⛔")
            return

        # 빨간 불 정지 중, 빨간 불이 사라지면 정지 잠금 해제
        if self.wait_for_red_clear:
            if not detected or color != 'red' or area <= self.traffic_area_threshold:
                self.wait_for_red_clear = False
                self.get_logger().info("✅ Red light cleared: resuming control")

    def lane_info_callback(self, msg: LaneInfo):
        # 빨간 불 정지 중이면 동작 무시
        if self.wait_for_red_clear:
            return

        self.current_lane     = msg.lane_num
        steering_angle        = msg.steering_angle
        vehicle_position_x    = msg.vehicle_position_x

        # 차선 변경 완료 조건
        if (self.is_changing_lane and
            self.current_lane == self.target_lane and
            abs(vehicle_position_x) <= 100):
            self.is_changing_lane = False
            self.get_logger().info(f"✅ Lane change complete: now on lane {self.current_lane}")

        angle_w, pos_w, normal_spd, change_spd = self.get_lane_config()
        cmd = MotionCommand()

        if self.is_changing_lane:
            steering = -8 if self.target_lane == 1 else 7
            cmd.left_speed  = change_spd
            cmd.right_speed = change_spd
            self.get_logger().info("🔁 Changing lane…")
        else:
            mapped   = (steering_angle / 50.0) * 10.0 * angle_w
            adjust   = -vehicle_position_x * pos_w
            steering = max(-10, min(mapped + adjust, 10))
            cmd.left_speed  = normal_spd
            cmd.right_speed = normal_spd

            if self.current_lane == 1 and steering < -8:
                cmd.left_speed = int(250 + steering * 10)

        cmd.steering = int(steering)
        self.motion_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MotionNode interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
