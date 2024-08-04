import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from navigation_robot_custom_interfaces.action import MeasureLapTime

class LapTimeClient(Node):
    def __init__(self):
        super().__init__('lap_time_client')
        self.action_client = ActionClient(self, MeasureLapTime, 'measure_lap_time')

        # Create a timer that will call send_goal() after a delay
        self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        # Wait for the action server to be available
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available.')
            self.destroy_node()
            rclpy.shutdown()
            return

        goal_msg = MeasureLapTime.Goal()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('LapTime goal rejected.')
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Total lap time: {result.total_time:.2f} seconds')
        self.destroy_node()
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Elapsed time: {feedback.elapsed_time:.2f} seconds')

def main(args=None):
    rclpy.init(args=args)
    lap_time_client = LapTimeClient()
    rclpy.spin(lap_time_client)  # Spin will keep the node active and processing callbacks

if __name__ == '__main__':
    main()










