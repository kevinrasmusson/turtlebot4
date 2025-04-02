import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from nav_msgs.msg import Path
import logging

class TurtlebotNavigator(Node):
	def __init__(self):
		super().__init__('turtlebot_navigator')
		logging.basicConfig(
			filename='path_distance.log',
			level=logging.INFO,
			format='%(asctime)s - %(message)s',
			datefmt='%Y-%m-%d %H:%M:%S'
		)
		# Publisher for initial pose
		self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/turtle/initialpose', 10)
		
		# Subscriber for AMCL pose to check if initial pose is set
		qos_profile = QoSProfile(depth=10)
		self.amcl_pose_subscriber = self.create_subscription(
			PoseWithCovarianceStamped,
			'/turtle/amcl_pose',
			self.amcl_pose_callback,
			qos_profile
		)
		# Subscriber for the planner's computed path
		self.path_subscriber = self.create_subscription(
			Path,
			'/turtle/plan',
			self.path_callback,
			10
		)
		self.get_logger().info("Subscribed to /turtle/plan")
		
		# Action client for navigation goal
		self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/turtle/navigate_to_pose')
		
		# Flags to check if initial pose is set and goal is sent
		self.initial_pose_set = False
		self.goal_sent = False
		
		# Timer to continuously publish initial pose
		self.initial_pose_timer = self.create_timer(1.0, self.set_initial_pose)
		
		# Timer to send goal pose after initial pose is set
		self.goal_pose_timer = None
	def set_initial_pose(self):
		if not self.initial_pose_set:
			initial_pose = PoseWithCovarianceStamped()
			initial_pose.header.frame_id = 'map'
			initial_pose.header.stamp = self.get_clock().now().to_msg()  # Ensure timestamp is set
			initial_pose.pose.pose.position.x = -0.157
			initial_pose.pose.pose.position.y = 0.068
			initial_pose.pose.pose.position.z = 0.0  # Ensure z is set
			
			# Set orientation with the values from RViz2
			initial_pose.pose.pose.orientation.x = 0.0
			initial_pose.pose.pose.orientation.y = 0.0
			initial_pose.pose.pose.orientation.z = 0.0
			initial_pose.pose.pose.orientation.w = 1.739
			
			# Set the covariance matrix to the values from RViz2
			initial_pose.pose.covariance = [0.0] * 36
			initial_pose.pose.covariance[0] = 0.25
			initial_pose.pose.covariance[7] = 0.25
			initial_pose.pose.covariance[35] = 0.06853891909122467
			
			self.initial_pose_publisher.publish(initial_pose)
			self.get_logger().info(f'Published initial pose: {initial_pose}')
				
	def amcl_pose_callback(self, msg):
		self.get_logger().info(f'Received AMCL pose: {msg}')
		self.initial_pose_set = True
		self.initial_pose_timer.cancel()
		if not self.goal_sent:
			self.goal_pose_timer = self.create_timer(2.0, self.send_goal_pose)

	def send_goal_pose(self):
		if not self.goal_sent:
			goal_pose = PoseStamped()
			goal_pose.header.frame_id = 'map'
			goal_pose.pose.position.x = -0.40
			goal_pose.pose.position.y = 1.42
			goal_pose.pose.orientation.z = 0.0
			goal_pose.pose.orientation.w = -0.00134
			
			goal_msg = NavigateToPose.Goal()
			goal_msg.pose = goal_pose
			
			self.get_logger().info('Waiting for action server...')
			self.nav_to_pose_client.wait_for_server()
			self.get_logger().info('Action server available. Sending goal...')
			
			send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
			send_goal_future.add_done_callback(self.goal_response_callback)
			self.goal_sent = True

	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback
		self.get_logger().info(f'Received feedback: {feedback}')

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected')
			self.goal_sent = False
			return

		self.get_logger().info('Goal accepted')
		get_result_future = goal_handle.get_result_async()
		get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info(f'Result: {result}')
		self.goal_sent = False  # Reset the flag if you want to send another goal later
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
	navigator = TurtlebotNavigator()
	rclpy.spin(navigator)
	navigator.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()