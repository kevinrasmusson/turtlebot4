#!/usr/bin/env python
# This program will track the time it takes for a turtlebot to calculate a path
import sys
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
class TurtleBot4TimeMeasurer(Node):
    def __init__(self):
        super().__init__('turtlebot4_time_measurer')
        self.subscription = self.create_subscription(
            Path,
            '/turtle/plan',
            self.plan_callback,
            10)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/turtle/goal_pose',
            self.pose_callback,
            10)
        self.start_time = None
    def goal_callback(self, msg):
        self.get_logger().info('Goal received')
        self.start_time = time.time() # Start the timer
    def plan_callback(self, msg):
        if self.start_time is not None:
            end_time = time.time()
            planning_time = end_time - self.start_time
            path_length = self.compute_path_length(msg)
            self.get_logger().info(f'Planning time: {planning_time:.4f} s | Path length: {path_length:.2f} m')
            self.start_time = None # Reset the timer
        else:
            self.get_logger().info('No goal received yet')
        #self.start_time = time.time() # Start the timer
        
    #Use euclidiean distance to calculate the length of the path
    def compute_path_length(self, path_msg):
        path_length = 0
        for i in range(1, len(path_msg.poses)):
            x1 = path_msg.poses[i-1].pose.position.x
            y1 = path_msg.poses[i-1].pose.position.y
            x2 = path_msg.poses[i].pose.position.x
            y2 = path_msg.poses[i].pose.position.y
            path_length += self.euclidean_distance(x1, y1, x2, y2)
        return path_length
    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def main (args=None):
    rclpy.init(args=args)
    node = TurtleBot4TimeMeasurer()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()