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
        self.traffic_area_threshold = 18000
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

        # --- 연속 장애물 감지용 변수 추가 ---
        self.lidar_obstacle_counter = 0
        self.lidar_obstacle_threshold = 3
        self.obstacle_flag = False

        # 연속 장애물 **없음** 감지용 변수
        self.no_obstacle_counter = 0
        self.no_obstacle_threshold = 3

        # 1차선 안정화 완료 플래그
        self.first_lane_stabilized = False

    def lidar_callback(self, msg: LaserScan):
        
        # 실제 라이다 프레임에서는 –90° 에서 –85° 로 매핑
        start_angle = (267.5 - 360.0) * 3.141592 / 180.0   # = –90° in rad
        end_angle   = (272.5 - 360.0) * 3.141592 / 180.0   # = –85° in rad


        # msg.angle_min 부터 시작해서 angle_increment 만큼씩 증가
        # 각도 → 인덱스 계산
        start_idx = int((start_angle - msg.angle_min) / msg.angle_increment)
        end_idx   = int((end_angle   - msg.angle_min) / msg.angle_increment)

        # 배열 경계(clamp) 처리
        start_idx = max(0, min(start_idx, len(msg.ranges)-1))
        end_idx   = max(0, min(end_idx,   len(msg.ranges)))

        # 슬라이스
        front_sector = msg.ranges[start_idx:end_idx]
        # 유효한 값만
        filtered = [r for r in front_sector if 0.0 < r < float('inf')]

        # 평균 계산
        if filtered:
            self.latest_lidar_avg = sum(filtered) / len(filtered)
        else:
            self.latest_lidar_avg = None

        # # ──────── 장애물 유무 로그 출력 ────────
        # if self.latest_lidar_avg is None:
        #     self.get_logger().info("[LiDAR] 유효한 거리 값이 없습니다")
        # elif self.latest_lidar_avg < 1.0:
        #     self.get_logger().info(f"[LiDAR] 🚧 장애물 감지! 평균 거리 = {self.latest_lidar_avg:.2f} m")
        # else:
        #     self.get_logger().info(f"[LiDAR] ✅ 장애물 없음, 평균 거리 = {self.latest_lidar_avg:.2f} m")

    def obstacle_callback(self, msg: String):
        # 차선 변경 중 또는 안정화 전까지 장애물 감지 무시
        if self.is_changing_lane or self.ignore_obstacle_after_lane_change:
            return

        # 카메라 기반 장애물 메시지 파싱
        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)', msg.data)
        if not match:
            return
        detected = (match.group(1).lower() == 'true')
        area = float(match.group(2))
        # self.get_logger().info(f"[Obstacle] detected={detected}, area={area}, lane={self.current_lane}")

        # 2차선 → 1차선 변경 트리거 (카메라 기준)
        if self.current_lane == 2 and detected and area > self.obstacle_area_threshold:
            self.target_lane = 1
            self.is_changing_lane = True
            self.get_logger().info("🚧 2차선 큰 장애물 감지 → 목표 차선을 1번으로 변경")
            return

        # 1차선에서 카메라 기반 이동 제어
        if self.current_lane == 1:
            cmd = MotionCommand()
            forward_motion = False
            backward_motion = False

            # 카메라 장애물 있을 때: 후진·정지·직진 결정
            if detected:
                # 후진 구간
                if area > self.obstacle_max_area:
                    backward_motion = True
                    cmd.steering = -int(max(-10, min((self.last_steering_angle/50*10)*self.angle_weight + 
                                        -self.last_vehicle_position_x*self.position_weight, 10)))
                    speed = int(self.backward_min_speed + min((area - self.obstacle_max_area)/self.obstacle_max_area,1.0) * 
                                (self.backward_max_speed - self.backward_min_speed))
                    cmd.left_speed = -speed
                    cmd.right_speed = -speed
                    # self.get_logger().info(f"🔴 너무 가까움 (area={area}) → 후진 속도 {-speed}")
                # 정지 구간
                elif area > self.obstacle_min_area:
                    cmd.steering = 0
                    cmd.left_speed = 0
                    cmd.right_speed = 0
                    # self.get_logger().info(f"🟡 중간 거리 (area={area}) → 정지")
                # 직진 구간
                else:
                    forward_motion = True
                    ratio = 1.0 - min(area / self.drive_start_area, 1.0)
                    speed = int(self.forward_min_speed + ratio * (self.forward_max_speed - self.forward_min_speed))
                    cmd.steering = int(max(-10, min((self.last_steering_angle/50*10)*self.angle_weight + 
                                        -self.last_vehicle_position_x*self.position_weight, 10)))
                    cmd.left_speed = speed
                    cmd.right_speed = speed
                    # self.get_logger().info(f"🟢 작은 장애물 (area={area}) → 직진 속도 {speed}")
            

            # 퍼블리시
            self.motion_pub.publish(cmd)

            # 후진 중 아니면 라이다 장애물 판단
            if not backward_motion:
                # 장애물 감지 카운트 로직
                if self.latest_lidar_avg is not None and self.latest_lidar_avg < 1.0:
                    self.lidar_obstacle_counter += 1
                    if self.lidar_obstacle_counter >= self.lidar_obstacle_threshold and not self.obstacle_flag:
                        self.obstacle_flag = True
                        self.get_logger().info(
                            f"⚠️ LiDAR 장애물 감지 연속 {self.lidar_obstacle_counter}회 → obstacle_flag=True")
                else:
                    # 장애물 없으면 카운터 리셋
                    self.lidar_obstacle_counter = 0

                # 기록된 장애물 후, forward_motion 중에 장애물 사라졌으면 차선 변경
                if self.obstacle_flag and forward_motion and self.latest_lidar_avg is not None and self.latest_lidar_avg > 1.0:
                    self.no_obstacle_counter += 1
                    if self.no_obstacle_counter >= self.no_obstacle_threshold:
                        self.target_lane = 2
                        self.is_changing_lane = True
                        self.obstacle_flag = False
                        self.first_lane_stabilized = False
                        self.no_obstacle_counter = 0
                        self.get_logger().info(
                            f"🔄 LiDAR 장애물 없음 연속 {self.no_obstacle_threshold}회 → 목표 차선 2로 변경")
                else:
                    if not (self.obstacle_flag and forward_motion):
                        self.no_obstacle_counter = 0


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

         # ——— 2) 1차선에서 완전히 안정화됐으면 여기서 멈춤 ———
        if self.current_lane == 1 and self.first_lane_stabilized:
            return


        
        cmd = MotionCommand()
        # Lane change logic
        if self.is_changing_lane:
            cmd.steering = -10 if self.target_lane == 1 else 7
            cmd.left_speed = self.lane_change_speed
            cmd.right_speed = self.lane_change_speed


            if self.current_lane == self.target_lane and abs(self.last_vehicle_position_x) <= 30:
                self.is_changing_lane = False
                self.ignore_obstacle_after_lane_change = True
                self.steering_stable_count = 0
                self.get_logger().info(f"✅ Lane change complete: now on lane {self.current_lane}")



        elif self.ignore_obstacle_after_lane_change:
            mapped = (self.last_steering_angle / 50.0) * 10.0 * self.angle_weight
            adjust = -self.last_vehicle_position_x * self.position_weight
            steering = max(-10, min(mapped + adjust, 10))
            cmd.steering = int(steering)
            cmd.left_speed = self.normal_speed
            cmd.right_speed = self.normal_speed

            if abs(steering) <= 10:
                self.steering_stable_count += 1
                if self.steering_stable_count >= 5:
                    self.ignore_obstacle_after_lane_change = False
                    self.first_lane_stabilized = True 
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
            cmd.steering = int(steering)
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
