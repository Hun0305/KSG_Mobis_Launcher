import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import LaneInfo, MotionCommand
import re

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_mission')

        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)
        self.create_subscription(String, 'traffic_light_result', self.traffic_callback, 10)
        self.create_subscription(String, 'obstacle_result', self.obstacle_callback, 10)

        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 장애물 감지
        self.obstacle_area_threshold = 5000
        self.obstacle_required_count = 3
        self.obstacle_detect_counter = 0

        # 신호등
        self.traffic_area_threshold = 21000
        self.red_required_count = 3
        self.red_clear_required_count = 3
        self.red_detect_counter = 0
        self.red_clear_counter = 0
        self.wait_for_red_clear = False

        # 상태
        self.is_changing_lane = False
        self.ignore_obstacle_after_lane_change = False
        self.steering_stable_count = 0
        self.current_lane = None
        self.target_lane = 2

        # 주행 설정
        self.angle_weight = 0.7
        self.position_weight = 0.05
        self.normal_speed = 255
        self.lane_change_speed = 100

        # 1차선 장애물 속도 제어 설정
        self.obstacle_min_area = 9000
        self.obstacle_max_area = 10000
        self.forward_max_speed = 255
        self.backward_max_speed = 150


        self.last_obstacle_msg = None

    def obstacle_callback(self, msg: String):
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)', msg.data)
        if not match:
            return

        detected = match.group(1).lower() == 'true'
        area = float(match.group(2))
        self.last_obstacle_msg = msg.data  # 최신 메시지 저장

        self.get_logger().info(f"[Obstacle] detected={detected}, area={area}, current_lane={self.current_lane}")

        if self.current_lane == 2 and detected and area > self.obstacle_area_threshold:
            self.obstacle_detect_counter += 1
            if self.obstacle_detect_counter >= self.obstacle_required_count:
                self.target_lane = 1
                self.is_changing_lane = True
                self.obstacle_detect_counter = 0
                self.get_logger().info("🚧 장애물 감지 3회 → 목표 차선을 1차선으로 변경")
        else:
            self.obstacle_detect_counter = 0

    def traffic_callback(self, msg: String):
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)(?:,\s*Color:\s*(\w+))?', msg.data)
        if not match:
            return

        detected = match.group(1).lower() == 'true'
        area = float(match.group(2))
        color = (match.group(3) or '').lower()

        if detected and area > self.traffic_area_threshold and color == 'red':
            self.red_clear_counter = 0
            self.red_detect_counter += 1

            if not self.wait_for_red_clear and self.red_detect_counter >= self.red_required_count:
                cmd = MotionCommand()
                cmd.left_speed = 0
                cmd.right_speed = 0
                cmd.steering = 0
                self.motion_pub.publish(cmd)
                self.wait_for_red_clear = True
                self.get_logger().info("🛑 Red light detected: stopping vehicle")
        else:
            self.red_detect_counter = 0
            if self.wait_for_red_clear:
                self.red_clear_counter += 1
                if self.red_clear_counter >= self.red_clear_required_count:
                    self.wait_for_red_clear = False
                    self.red_clear_counter = 0
                    self.get_logger().info("🟢 Red light cleared: resuming")

    def lane_info_callback(self, msg: LaneInfo):
        if self.wait_for_red_clear:
            self.get_logger().warning("🚦 Red light active: ignoring lane control")
            return

        self.current_lane = msg.lane_num
        steering_angle = msg.steering_angle
        vehicle_position_x = msg.vehicle_position_x

        angle_w = self.angle_weight
        pos_w = self.position_weight
        normal_spd = self.normal_speed
        change_spd = self.lane_change_speed

        cmd = MotionCommand()

        # 🔁 차선 변경 중
        if self.is_changing_lane:
            cmd.steering = -10 if self.target_lane == 1 else 10
            cmd.left_speed = change_spd
            cmd.right_speed = change_spd
            self.get_logger().info("🔁 Changing lane…")

            if self.current_lane == self.target_lane and abs(vehicle_position_x) <= 30:
                self.is_changing_lane = False
                self.ignore_obstacle_after_lane_change = True
                self.steering_stable_count = 0
                self.get_logger().info(f"✅ Lane change complete: now on lane {self.current_lane}")

        # 🚦 차선 변경 후 정렬 중
        elif self.ignore_obstacle_after_lane_change:
            mapped = (steering_angle / 50.0) * 10.0 * angle_w
            adjust = -vehicle_position_x * pos_w
            steering = max(-10, min(mapped + adjust, 10))
            cmd.steering = int(steering)
            cmd.left_speed = normal_spd
            cmd.right_speed = normal_spd

            if abs(steering) <= 5:
                self.steering_stable_count += 1
                self.get_logger().info(f"🟢 안정화 중: {self.steering_stable_count}/5")
                if self.steering_stable_count >= 5:
                    self.ignore_obstacle_after_lane_change = False
                    cmd.left_speed = 0
                    cmd.right_speed = 0
                    cmd.steering = 0
                    self.motion_pub.publish(cmd)
                    self.get_logger().info("✅ 정렬 완료 → 차량 정지 및 장애물 감지 재개")
                    return
            else:
                self.steering_stable_count = 0

        # 🚗 일반 주행
        else:
            if self.current_lane == 1:
                cmd.steering = 0

                if self.last_obstacle_msg:
                    match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)', self.last_obstacle_msg)
                    if match and match.group(1).lower() == 'true':
                        area = float(match.group(2))
                        if area < self.obstacle_min_area:
                            
                            #직진 조향
                            mapped = (steering_angle / 50.0) * 10.0 * angle_w
                            adjust = -vehicle_position_x * pos_w
                            steering = max(-10, min(mapped + adjust, 10))
                            
                            # 작은 장애물 → 가까이 없음 → 빠르게 전진
                            scale = 1.0 - (area / self.obstacle_min_area)
                            speed = int(scale * self.forward_max_speed)
                            cmd.left_speed = speed
                            cmd.right_speed = speed
                            self.get_logger().info(f"🟢 작은 장애물 (area={area}) → 전진 속도 {speed}")

                        elif area > self.obstacle_max_area:

                            #후진 조향
                            mapped = (steering_angle / 50.0) * 10.0 * angle_w
                            adjust = -vehicle_position_x * pos_w
                            steering = -max(-10, min(mapped + adjust, 10)) 


                            # 큰 장애물 → 가까이 있음 → 빠르게 후진
                            scale = min((area - self.obstacle_max_area) / self.obstacle_max_area, 1.0)
                            speed = int(scale * self.backward_max_speed)
                            cmd.left_speed = -speed
                            cmd.right_speed = -speed
                            self.get_logger().info(f"🔴 큰 장애물 (area={area}) → 후진 속도 {-speed}")

                        else:
                            cmd.left_speed = 0
                            cmd.right_speed = 0
                            self.get_logger().info(f"🟡 중간 장애물 (area={area}) → 정지")
                    else:
                        cmd.left_speed = 0
                        cmd.right_speed = 0
                else:
                    cmd.left_speed = 0
                    cmd.right_speed = 0
                    self.get_logger().info("⚠️ 장애물 정보 없음 → 정지")

            else:
                mapped = (steering_angle / 50.0) * 10.0 * angle_w
                adjust = -vehicle_position_x * pos_w
                steering = max(-10, min(mapped + adjust, 10))
                cmd.steering = int(steering)
                cmd.left_speed = normal_spd
                cmd.right_speed = normal_spd

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
