import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import logging
import time

class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')

        logging.basicConfig(
            filename='path_distance.log',
            level=logging.INFO,
            format='%(asctime)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        self.get_logger().info("Initializing PathListener...")

        # Publishers & Subscribers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/turtle/initialpose', 10)
        self.subscription = self.create_subscription(Path, '/turtle/plan', self.path_callback, 10)
        self.action_client = ActionClient(self, NavigateToPose, '/turtle/navigate_to_pose')

        self.logged_once = False

        # Start the sequence
        self.publish_initial_pose()
        time.sleep(2)  # Give AMCL time to converge
        self.send_goal()

    def publish_initial_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # Your RViz2 pose values
        pose.pose.pose.position.x = -3.71
        pose.pose.pose.position.y = 0.953
        pose.pose.pose.position.z = 0.0

        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = -0.00534  # Note: this may need full quaternion

        # Covariance values
        pose.pose.covariance[0] = 0.25     # x
        pose.pose.covariance[7] = 0.25     # y
        pose.pose.covariance[35] = 0.0685  # yaw

        self.initial_pose_pub.publish(pose)
        self.get_logger().info("Published initial pose to /turtle/initialpose")

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Set your goal position
        goal_pose.pose.position.x = 25.1
        goal_pose.pose.position.y = -13.2
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = -0.00134  # Again, full quaternion might be better

        goal_msg.pose = goal_pose

        self.get_logger().info("Waiting for action server...")
        self.action_client.wait_for_server()
        self.get_logger().info("Sending goal to /turtle/navigate_to_pose")

        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback}")

    def path_callback(self, msg: Path):
        if not self.logged_once:
            distance = 0.0
            poses = msg.poses
            for i in range(1, len(poses)):
                x1 = poses[i-1].pose.position.x
                y1 = poses[i-1].pose.position.y
                x2 = poses[i].pose.position.x
                y2 = poses[i].pose.position.y
                distance += math.hypot(x2 - x1, y2 - y1)

            log_msg = f"Path length: {distance:.2f} meters"
            self.get_logger().info(log_msg)
            logging.info(log_msg)
            self.logged_once = True

def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
