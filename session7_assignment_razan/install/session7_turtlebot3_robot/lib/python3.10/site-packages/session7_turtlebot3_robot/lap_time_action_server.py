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
        self.start_position = 0.0
        self.current_position =0.0
        self.start_time = None
        self.tolerance = 0.5

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose

    def execute_callback(self, goal_handle):
        self.get_logger().info('Lap time server started ...')

        self.start_time = self.get_clock().now()
        self.start_position = self.current_position

        feedback_msg = MeasureLapTime.Feedback()

        while not self.is_lap_completed():
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            feedback_msg.elapsed_time = elapsed_time
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        total_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        result = MeasureLapTime.Result()
        result.total_time = total_time
        goal_handle.succeed()

        self.get_logger().info(f'Lap completed by robot in {total_time:.2f} seconds')
        return result

    def is_lap_completed(self):
        if self.start_position is None or self.current_position is None:
            return False

        current_elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if current_elapsed_time > 25:
            distance_traveled = self._compute_distance(self.start_position, self.current_position)
            if distance_traveled < self.tolerance:
                return True
        return False

    def _compute_distance(self, start, current):
        x = start.position.x - current.position.x
        y = start.position.y - current.position.y
        z = start.position.z - current.position.z
        distance= (x ** 2 + y ** 2 + z ** 2)**0.5
        return distance

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


