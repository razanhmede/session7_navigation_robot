import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from navigation_robot_custom_interfaces.srv import FindClosestWall

class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        
        # Publisher to control the robot's movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create client for FindClosestWall service
        self.client = self.create_client(FindClosestWall, 'find_closest_wall')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Initialize subscription to LaserScan topic
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer to periodically call FindClosestWall service
        self.timer = self.create_timer(1.0, self.call_find_closest_wall)

    def scan_callback(self, msg):
        twist = Twist()

        # Adjusted values for obstacle detection
        front_obstacle_distance = 0.6
        side_obstacle_distance = 0.65
        # msg[0] is the distance at the first angle of the laser scanner which is directly in front of the robot
        if msg.ranges[0] < front_obstacle_distance:
            twist.linear.x = 0.0
            twist.angular.z = -0.4
            self.get_logger().info(f"Obstacle detected very close at {msg.ranges[0]:.2f} meters. Stopping and turning sharply.")
        # msg[25] distance measurement 35 measurements away from the start of the scan data 
        elif msg.ranges[35] < side_obstacle_distance:
            twist.linear.x = 0.0
            twist.angular.z = -0.2
            self.get_logger().info(f"Obstacle detected nearby at {msg.ranges[25]:.2f} meters. Turning slightly.")
        else:
            twist.linear.x = 0.3
            twist.angular.z = 0.0
            self.get_logger().info("No immediate obstacle detected. Moving forward with slight speed.")
        
        self.publisher.publish(twist)

    def call_find_closest_wall(self):
        request = FindClosestWall.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Closest wall at x: {response.x}, y: {response.y}, distance: {response.z}")
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)
    robot_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
