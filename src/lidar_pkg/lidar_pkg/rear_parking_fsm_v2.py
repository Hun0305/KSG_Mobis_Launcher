import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from interfaces_pkg.msg import MotionCommand

class RearParkingFSMv2(Node):
    def __init__(self):
        super().__init__('rear_parking_fsm_v2')

        self.subscription = self.create_subscription(
            LaserScan,
            'lidar_processed',
            self.lidar_callback,
            10
        )

        self.publisher = self.create_publisher(
            MotionCommand,
            'motion_command',
            10
        )

        self.state = 'WAIT_FOR_CAR1'
        self.backward_clear = False
        self.get_logger().info('Rear Parking FSM v2 started.')

    def lidar_callback(self, msg):
        ranges = msg.ranges

        def detected(angle, threshold=1.5):
            idx = angle % len(ranges)
            return 0.05 < ranges[idx] < threshold

        def not_detected(angle, threshold=1.5):
            idx = angle % len(ranges)
            return not (0.05 < ranges[idx] < threshold)

        if self.state == 'WAIT_FOR_CAR1':
            if detected(270):
                self.state = 'REVERSE_RIGHT'
                self.get_logger().info('Car1 detected at 270° → start REVERSE_RIGHT')

        elif self.state == 'REVERSE_RIGHT':
            if not_detected(270) and not_detected(90):
                self.backward_clear = True
                self.state = 'FORWARD_CHECK'
                self.get_logger().info('Both 90° and 270° are clear → move to FORWARD_CHECK')
            else:
                motion = MotionCommand()
                motion.steering = 30
                motion.left_speed = -130
                motion.right_speed = -130
                self.publisher.publish(motion)

        elif self.state == 'FORWARD_CHECK':
            if not_detected(90) and not_detected(270):
                self.state = 'FORWARD_RIGHT'
                self.get_logger().info('Still clear → move to FORWARD_RIGHT')
            else:
                self.get_logger().info('Detected again, skipping to FORWARD_OUT')
                self.state = 'FORWARD_OUT'

        elif self.state == 'FORWARD_RIGHT':
            if detected(270):
                self.state = 'FORWARD_OUT'
                self.get_logger().info('Car1 detected again at 270° → switch to FORWARD_OUT')
            else:
                motion = MotionCommand()
                motion.steering = 30
                motion.left_speed = 150
                motion.right_speed = 150
                self.publisher.publish(motion)

        elif self.state == 'FORWARD_OUT':
            motion = MotionCommand()
            motion.steering = 0
            motion.left_speed = 150
            motion.right_speed = 150
            self.publisher.publish(motion)
            self.get_logger().info('Straight OUT → Parking complete.')

    def destroy(self):
        motion = MotionCommand()
        motion.steering = 0
        motion.left_speed = 0
        motion.right_speed = 0
        self.publisher.publish(motion)

def main(args=None):
    rclpy.init(args=args)
    node = RearParkingFSMv2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
