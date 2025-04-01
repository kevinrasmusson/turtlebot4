from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import rclpy
import logging
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class PathListener(Node):
	def __init__(self):
		super().__init__('path_listener')
		self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
		logging.basicConfig(
			filename='path_distance.log',
			level=logging.INFO,
			format='%(asctime)s - %(message)s',
			datefmt='%Y-%m-%d %H:%M:%S'
		)
		self.subscription = self.create_subscription(
			Path,
			'/turtle/plan',
			self.path_callback,
			10
		)
		self.get_logger().info("Subscribed to /turtle/plan")
		self.logged_once = False
		self.send_goal()
	def send_goal(self):
		goal_msg = NavigateToPose.Goal()

		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map'


		goal_pose.position.x = 25.1
		goal_pose.position.y = -13.2
		goal_pose.orientation.w = -0.00134
		self.action_client.wait_for_server()
		self.get_logger().info("Sending goal to /navigate_to_pose")
		self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
	def path_callback(self, msg: Path):
		if not self.logged_once:
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
			logging.info(f"Path length: {distance:.2f} meters")
			self.logged_once = True
			
def main(args=None):
	rclpy.init(args=args)

	path_listener = PathListener()

	rclpy.spin(path_listener)

	path_listener.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()