import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from navigation_robot_custom_interfaces.srv import FindClosestWall
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import threading
import time


class WallFinder(Node):
    def __init__(self):
        super().__init__('wall_finder_service_server')

        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            FindClosestWall,
            'find_closest_wall',
            self.find_wall_callback,
            callback_group=self.callback_group
        )
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        self.closest_wall_x = float('inf')
        self.closest_wall_y = float('inf')
        self.closest_wall_z = 0.0
        self.path = []
        self.scan_data = None
        self.scanning = False
        
    def scan_callback(self, msg):
        # This function handles incoming laser scan messages.
        self.scan_data = msg.ranges
        if not self.scanning:
            return

        min_distance = float('inf')
        angle_of_min_distance = 0.0

        for i, distance in enumerate(self.scan_data):
            if np.isfinite(distance) and distance < min_distance:
                min_distance = distance
                angle_of_min_distance = i * msg.angle_increment

        if min_distance != float('inf'):
            self.closest_wall_x = min_distance * np.cos(angle_of_min_distance)
            self.closest_wall_y = min_distance * np.sin(angle_of_min_distance)
            self.closest_wall_z = 0.0
        else:
            self.get_logger().info('No valid wall detected.')
        
    def odom_callback(self, msg):
        # This function handles incoming odometry messages.
        position = msg.pose.pose.position
        self.path.append((position.x, position.y))
        
    def find_wall_callback(self, request, response):
        # This function handles incoming service requests to find the closest wall.
        self.get_logger().info('Incoming FindClosestWall request.')

        self.scanning = True
        time.sleep(5)  # Simulate scanning time
        self.scanning = False

        rotate_right = self.determine_rotation_direction()
        msg = Twist()
        msg.angular.z = -0.2 if rotate_right else 0.2
        self.publisher.publish(msg)

        done = False
        while not done:
            self.scanning = True
            time.sleep(5)  # Simulate scanning time
            self.scanning = False
            done = self.has_found_correct_direction()

        msg.angular.z = 0.0
        self.publisher.publish(msg)

        response.x = self.closest_wall_x
        response.y = self.closest_wall_y
        response.z = self.closest_wall_z

        if self.closest_wall_x != float('inf') or self.closest_wall_y != float('inf'):
            self.get_logger().info(f"Wall found at position: ({response.x}, {response.y}, {response.z})")
        else:
            self.get_logger().info("No wall detected yet.")

        return response
    
    def determine_rotation_direction(self):
        # Determines if the robot should rotate right or left to find the wall.
        if self.scan_data is None:
            return False

        segment_size = len(self.scan_data) // 4
        segment_distances = [np.mean(self.scan_data[i * segment_size:(i + 1) * segment_size]) for i in range(4)]
        
        min_index = np.argmin(segment_distances)
        return min_index >= 2
    
    def has_found_correct_direction(self):
        # Checks if the robot has found the correct direction towards the wall.
        if self.scan_data is None:
            return False

        quarter_size = len(self.scan_data) // 4
        forward_segment = self.scan_data[:quarter_size]
        forward_distance = np.mean(forward_segment)

        surrounding_segments = [
            self.scan_data[quarter_size:2 * quarter_size],
            self.scan_data[2 * quarter_size:3 * quarter_size],
            self.scan_data[3 * quarter_size:]
        ]
        surrounding_distances = [np.mean(segment) for segment in surrounding_segments]
        return all(forward_distance >= distance for distance in surrounding_distances)
    
    def plot_path(self):
        # Plots the robot's path.
        while rclpy.ok():
            if self.path:
                x, y = zip(*self.path)
                plt.figure()
                plt.plot(x, y, label='Path')
                plt.scatter(x[-1], y[-1], color='blue', label='Current Position')
                plt.title('Robot Path and Distance to Wall plot')
                plt.xlabel('X position')
                plt.ylabel('Y position')
                plt.legend()
                plt.grid(True)
                plt.show()
            else:
                self.get_logger().info("Path is empty, cannot plot.")
            time.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    wall_finder = WallFinder()
    executor = MultiThreadedExecutor()
    executor.add_node(wall_finder)

    try:
        executor.spin()
    finally:
        wall_finder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

