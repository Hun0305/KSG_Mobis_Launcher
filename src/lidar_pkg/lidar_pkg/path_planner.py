import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf_transformations
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            MarkerArray,
            '/parking_visualization',
            self.parking_space_callback,
            10)
        self.path_publisher = self.create_publisher(Path, '/global_path', 10)
        self.path_planned = False
        self.get_logger().info('Path Planner Node has been started.')

    def parking_space_callback(self, msg):
        if self.path_planned:
            return

        parking_spaces = [m for m in msg.markers if m.ns == "parking_spaces"]
        if not parking_spaces:
            return

        target_space = parking_spaces[0]
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            current_pose = PoseStamped()
            current_pose.header.stamp = self.get_clock().now().to_msg()
            current_pose.header.frame_id = 'map'
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.orientation = trans.transform.rotation
        except Exception as e:
            self.get_logger().error(f'Could not get transform from map to base_link: {e}')
            return

        path = self.generate_path(current_pose, target_space.pose)

        if path:
            self.path_publisher.publish(path)
            self.get_logger().info('Global path has been published.')
            self.path_planned = True
            self.destroy_subscription(self.subscription)

    def generate_path(self, start_pose, target_pose):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map' 

        waypoint = PoseStamped()
        waypoint.header = path_msg.header
        waypoint.pose.position.x = target_pose.position.x - 1.0
        waypoint.pose.position.y = target_pose.position.y
        q = tf_transformations.quaternion_from_euler(0, 0, 1.5707)
        waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z, waypoint.pose.orientation.w = q

        final_pose = PoseStamped()
        final_pose.header = path_msg.header
        final_pose.pose = target_pose
        final_pose.pose.orientation = waypoint.pose.orientation

        path_msg.poses.append(start_pose)
        path_msg.poses.append(waypoint)
        path_msg.poses.append(final_pose)
        return path_msg

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()