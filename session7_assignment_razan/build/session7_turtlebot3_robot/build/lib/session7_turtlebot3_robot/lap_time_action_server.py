import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from navigation_robot_custom_interfaces.action import MeasureLapTime
from nav_msgs.msg import Odometry
import time

class LapTimeActionServer(Node):
    def __init__(self):
        super().__init__('lap_time_action_server')
        self._action_server = ActionServer(
            self,
            MeasureLapTime,
            'measure_lap_time',
            self.execute_callback
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.start_position = None
        self.current_position = None
        self.start_time = None
        self.tolerance = 0.5
        self.start_x = 2.0
        self.start_y = 2.0
        self.time_limit = 25.0  # Time limit for lap completion

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose

    def execute_callback(self, goal_handle):
        self.get_logger().info('Lap time server started ...')

        # Initialize start time
        self.start_time = self.get_clock().now()

        feedback_msg = MeasureLapTime.Feedback()

        while not self.is_lap_completed():
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            feedback_msg.elapsed_time = elapsed_time
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        lap_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        result = MeasureLapTime.Result()
        result.lap_time = lap_time
        goal_handle.succeed()

        self.get_logger().info(f'Lap completed by robot in {lap_time:.2f} seconds')
        return result

    def is_lap_completed(self):
        if self.current_position is None:
            return False

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time > self.time_limit:
            distance_to_start = self._compute_distance_to_start()
            if distance_to_start < self.tolerance:
                return True
        return False

    def _compute_distance_to_start(self):
        if self.current_position is None:
            return float('inf')  # Large distance if no position data

        x = self.current_position.position.x
        y = self.current_position.position.y
        distance_squared = (self.start_x - x) ** 2 + (self.start_y - y) ** 2
        return distance_squared ** 0.5

def main(args=None):
    rclpy.init(args=args)
    action_server = LapTimeActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

