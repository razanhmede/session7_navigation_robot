import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from navigation_robot_custom_interfaces.srv import FindClosestWall
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import threading
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
import time 

class WallFinder(Node):
    def __init__(self):
        super().__init__('wall_finder_service_server')
        #initializes a reentrant callbackgroup to handle asynchronous tasks
        self.callback_group = ReentrantCallbackGroup() 
        # Create service
        self.srv = self.create_service(FindClosestWall, 'find_closest_wall', self.find_wall_callback)
        
        # Create publisher to control linear and angular velocity of the the robot 
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to LaserScan and Odometry topics
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Initialize variables
        self.closest_wall_x = float('inf')
        self.closest_wall_y = float('inf')
        self.closest_wall_z = 0.0
        self.path = [] 
        self.scan_data = None
        self.scanning = False
        
        # Start plotting thread
        self.plot_thread = threading.Thread(target=self.plot_path, daemon=True)
        self.plot_thread.start()
        
    def scan_callback(self, msg):
    #updates the scan data with the latest laser scan ranges 
        self.scan_data = msg.ranges
    #if scanning is not active skip processing 
        if not self.scanning:
            return
    #initialize min distance and angle 
        min_distance = float('inf')
        angle_of_min_distance = None
    # iterate over the scanned data to find the minimum distance and calculate the angle     
        for i, distance in enumerate(self.scan_data):
            if np.isfinite(distance):
                if distance < min_distance:
                    min_distance = distance
                    angle_of_min_distance = i * msg.angle_increment
        
        if min_distance != float('inf'):
            angle = angle_of_min_distance
            self.closest_wall_x = min_distance * np.cos(angle)
            self.closest_wall_y = min_distance * np.sin(angle)
            self.closest_wall_z = 0.0
        else:
            self.get_logger().info('No valid wall detected.')

    def find_wall_callback(self, request, response):
        self.get_logger().info('Incoming FindClosestWall request.')
        
        # Start scanning
        self.scanning = True
        while self.scanning:
            pass
        
        # Determine rotation direction
        rotate_right = self.determine_rotation_direction()
        msg = Twist()
        #set rotation speed based on the direction of rotation
        msg.angular.z = -0.2 if rotate_right else 0.2
        self.publisher.publish(msg)
        
        # Wait until the correct direction is found
        done = False
        while not done:
            self.scanning = True
            while self.scanning:
                pass
            done = self.has_found_correct_direction()
        
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        #wall position
        response.x = self.closest_wall_x
        response.y = self.closest_wall_y
        response.z = self.closest_wall_z
        
        if self.closest_wall_x != float('inf') or self.closest_wall_y != float('inf'):
            self.get_logger().info(f"Wall found at position: ({response.x}, {response.y}, {response.z})")
        else:
            self.get_logger().info("No wall detected yet.")
        
        return response
    
    def odom_callback(self, msg):
        position = msg.pose.pose.position
    #add robot's current position to the path list
        self.path.append((position.x, position.y))
    #this method determines the rotation direction based on the scanned data     
    def determine_rotation_direction(self):
        if self.scan_data is None:
            return False
        
        # Divide scan data into segments and calculate the average distance for each segment
        segment_size = len(self.scan_data) // 4
        segment_distances = [np.mean(self.scan_data[i * segment_size:(i + 1) * segment_size]) for i in range(4)]
        
        # Determine which segment has the smallest average distance
        min_index = np.argmin(segment_distances)
        # Turn right if the smallest distance is in the last two segments
        return min_index >= 2 
    #checks if the robot is facing the correct direction
    def has_found_correct_direction(self):
        if self.scan_data is None:
            return False
        #data from the forward segment of the scan
        forward_segment = self.scan_data[:len(self.scan_data) // 4]
        #average distance in the forward segment
        forward_distance = np.mean(forward_segment)
        
        surrounding_segments = [
            self.scan_data[len(self.scan_data) // 4:len(self.scan_data) // 2],
            self.scan_data[len(self.scan_data) // 2:3 * len(self.scan_data) // 4],
            self.scan_data[3 * len(self.scan_data) // 4:]
        ]
        #average distances for the surrounding segments 
        surrounding_distances = [np.mean(segment) for segment in surrounding_segments]
        return all(forward_distance >= distance for distance in surrounding_distances)
    
    def plot_path(self):
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
