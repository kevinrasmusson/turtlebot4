import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time

class PlannerComparisonNode(Node):
    def __init__(self):
        super().__init__('planner_comparison_node')
        self.goal_time = None

        # Subscribe to the goal topic
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/turtle/goal_pose',
            self.goal_callback,
            10
        )

        # Subscribe to the plan topic
        self.plan_subscriber = self.create_subscription(
            Path,
            '/turtle/plan',
            self.plan_callback,
            10
        )

    def goal_callback(self, msg):
        # Record the time when a goal is received
        self.goal_time = time.time()
        self.get_logger().info('Goal received')

    def plan_callback(self, msg):
        # Check if a goal time was recorded
        if self.goal_time is not None:
            # Calculate the time difference
            plan_time = time.time()
            time_difference = plan_time - self.goal_time
            self.get_logger().info(f'Planning time: {time_difference:.4f} seconds')
            # Reset the goal time
            self.goal_time = None

def main(args=None):
    rclpy.init(args=args)
    node = PlannerComparisonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
