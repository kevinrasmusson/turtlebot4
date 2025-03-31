from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')

        self.subscription = self.create_subscription(
            Path,
            '/turtle/plan',
            self.path_callback,
            10
        )
        self.get_logger().info("Subscribed to /turtle/plan")

    def path_callback(self, msg: Path):
        distance = 0.0
        poses = msg.poses
        for i in range(1, len(poses)):
            x1 = poses[i-1].pose.position.x
            y1 = poses[i-1].pose.position.y
            x2 = poses[i].pose.position.x
            y2 = poses[i].pose.position.y
            dx = x2 - x1
            dy = y2 - y1
            distance += math.hypot(dx, dy)

        self.get_logger().info(f"Path length: {distance:.2f} meters")
def main(args=None):
    rclpy.init(args=args)

    path_listener = PathListener()

    rclpy.spin(path_listener)

    path_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()