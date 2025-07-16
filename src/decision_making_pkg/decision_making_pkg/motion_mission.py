#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import LaneInfo, MotionCommand
from sensor_msgs.msg import LaserScan
import re

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_mission')

        # State for steering calculation in obstacle_callback
        self.last_steering_angle = 0.0
        self.last_vehicle_position_x = 0.0

        # Subscriptions
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)
        self.create_subscription(String, 'traffic_light_result', self.traffic_callback, 10)
        self.create_subscription(String, 'obstacle_result', self.obstacle_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publisher
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # LiDAR state
        self.latest_lidar_avg = None
        self.prev_lidar_in_range = False

        # Driving state
        self.current_lane = None
        self.target_lane = 2
        self.is_changing_lane = False
        self.ignore_obstacle_after_lane_change = False
        self.steering_stable_count = 0
        self.wait_for_red_clear = False

        # Thresholds & parameters
        self.obstacle_area_threshold = 5000    # 2→1 lane change trigger
        self.obstacle_max_area = 25000        # reverse trigger (area > this)
        self.obstacle_min_area = 23000        # full stop upper bound
        self.drive_start_area = 20000         # begin forward lower bound

        # Speed parameters (tunable)
        self.forward_min_speed = 80
        self.forward_max_speed = 255
        self.backward_min_speed = 60
        self.backward_max_speed = 150

        # Traffic light parameters
        self.traffic_area_threshold = 21000
        self.red_required_count = 3
        self.red_clear_required_count = 3
        self.red_detect_counter = 0
        self.red_clear_counter = 0

        # Lane change parameters
        self.normal_speed = 255
        self.lane_change_speed = 200

        # Weights for steering adjustment
        self.angle_weight = 0.7
        self.position_weight = 0.05

    def lidar_callback(self, msg: LaserScan):
        # Compute average distance in front sector (250°–290°)
        angle_min_deg = msg.angle_min * 180.0 / 3.141592
        angle_increment_deg = msg.angle_increment * 180.0 / 3.141592
        start_idx = int((250 - angle_min_deg) / angle_increment_deg)
        end_idx = int((290 - angle_min_deg) / angle_increment_deg)
        front_sector = msg.ranges[start_idx:end_idx]
        filtered = [r for r in front_sector if 0.0 < r < float('inf')]

        if filtered:
            self.latest_lidar_avg = sum(filtered) / len(filtered)
        else:
            self.latest_lidar_avg = None

    def obstacle_callback(self, msg: String):
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)', msg.data)
        if not match:
            return

        detected = (match.group(1).lower() == 'true')
        area = float(match.group(2))
        self.get_logger().info(f"[Obstacle] detected={detected}, area={area}, lane={self.current_lane}")

        # 2nd lane → 1st lane change
        if self.current_lane == 2 and detected and area > self.obstacle_area_threshold:
            self.target_lane = 1
            self.is_changing_lane = True
            self.get_logger().info("🚧 2차선 큰 장애물 감지 → 목표 차선을 1번으로 변경")
            return

        # Only process in 1st lane when obstacle detected
        if self.current_lane == 1 and detected:
            cmd = MotionCommand()
            forward_motion = False

            # Steering adjustment
            mapped = (self.last_steering_angle / 50.0) * 10.0 * self.angle_weight
            adjust = -self.last_vehicle_position_x * self.position_weight

            # 1) area > obstacle_max_area → reverse with interpolation
            if area > self.obstacle_max_area:
                ratio = min((area - self.obstacle_max_area) / self.obstacle_max_area, 1.0)
                speed = int(self.backward_min_speed +
                            ratio * (self.backward_max_speed - self.backward_min_speed))
                cmd.steering = int(max(-10, min(mapped + adjust, 10)))
                cmd.left_speed = -speed
                cmd.right_speed = -speed
                self.get_logger().info(f"🔴 너무 가까움 (area={area}) → 후진 속도 {-speed}")

            # 2) obstacle_min_area < area ≤ obstacle_max_area → full stop
            elif area > self.obstacle_min_area:
                cmd.steering = 0
                cmd.left_speed = 0
                cmd.right_speed = 0
                self.get_logger().info(f"🟡 중간 거리 (area={area}) → 정지")

            # 3) drive_start_area < area ≤ obstacle_min_area → calculate only
            elif area > self.drive_start_area:
                calc_spd = int(-0.0375 * area + 375)
                cmd.steering = 0
                cmd.left_speed = 0
                cmd.right_speed = 0
                self.get_logger().info(f"🟠 계산된 속도({calc_spd}), 이 구간에서는 전진하지 않음")

            # 4) area ≤ drive_start_area → forward with interpolation
            else:
                ratio = 1.0 - min(area / self.drive_start_area, 1.0)
                speed = int(self.forward_min_speed +
                            ratio * (self.forward_max_speed - self.forward_min_speed))
                cmd.steering = int(max(-10, min(mapped + adjust, 10)))
                cmd.left_speed = speed
                cmd.right_speed = speed
                forward_motion = True
                self.get_logger().info(f"🟢 작은 장애물 (area={area}) → 직진 속도 {speed}")

            # Publish the motion command
            self.motion_pub.publish(cmd)

            # LiDAR return check only during forward motion
            if forward_motion:
                if self.latest_lidar_avg is not None:
                    if self.latest_lidar_avg <= 1.0:
                        self.prev_lidar_in_range = True
                    elif self.prev_lidar_in_range:
                        self.prev_lidar_in_range = False
                        self.target_lane = 2
                        self.get_logger().info("✅ 장애물 사라짐 → 목표 차선을 2로 변경")
            else:
                # Reset return flag when not moving forward
                self.prev_lidar_in_range = False

    def traffic_callback(self, msg: String):
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)(?:,\s*Color:\s*(\w+))?', msg.data)
        if not match:
            return

        detected = (match.group(1).lower() == 'true')
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
        # Skip control while waiting at red light
        if self.wait_for_red_clear:
            return

        # Update for obstacle steering calculation
        self.current_lane = msg.lane_num
        self.last_steering_angle = msg.steering_angle
        self.last_vehicle_position_x = msg.vehicle_position_x

        # In 1st lane, control only in obstacle_callback
        if self.current_lane == 1:
            return

        cmd = MotionCommand()
        # Lane change logic
        if self.is_changing_lane:
            cmd.steering = -10 if self.target_lane == 1 else 10
            cmd.left_speed = self.lane_change_speed
            cmd.right_speed = self.lane_change_speed
            if self.current_lane == self.target_lane and abs(self.last_vehicle_position_x) <= 30:
                self.is_changing_lane = False
                self.ignore_obstacle_after_lane_change = True
                self.steering_stable_count = 0
                self.get_logger().info(f"✅ Lane change complete: now on lane {self.current_lane}")

        elif self.ignore_obstacle_after_lane_change:
            # Stabilize then stop
            mapped = (self.last_steering_angle / 50.0) * 10.0 * self.angle_weight
            adjust = -self.last_vehicle_position_x * self.position_weight
            steering = max(-10, min(mapped + adjust, 10))
            cmd.steering = int(steering)
            cmd.left_speed = self.normal_speed
            cmd.right_speed = self.normal_speed

            if abs(steering) <= 5:
                self.steering_stable_count += 1
                if self.steering_stable_count >= 5:
                    self.ignore_obstacle_after_lane_change = False
                    cmd.left_speed = 0
                    cmd.right_speed = 0
                    cmd.steering = 0
                    self.motion_pub.publish(cmd)
                    self.get_logger().info("✅ Stabilized → vehicle stopped, resume obstacle detection")
                    return
            else:
                self.steering_stable_count = 0

        else:
            # Normal 2nd-lane driving
            mapped = (self.last_steering_angle / 50.0) * 10.0 * self.angle_weight
            adjust = -self.last_vehicle_position_x * self.position_weight
            steering = max(-10, min(mapped + adjust, 10))
            cmd.stering = int(steering)
            cmd.left_speed = self.normal_speed
            cmd.right_speed = self.normal_speed

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
