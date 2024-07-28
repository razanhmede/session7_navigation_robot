import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from navigation_robot_custom_interfaces.action import MeasureLapTime

class LapTimeActionClient(Node):
    def __init__(self):
        super().__init__('lap_time_action_client')
#create action client to measure lap time
        self.action_client = ActionClient(self, MeasureLapTime, 'measure_lap_time')
#Send a goal request to the MeasureLapTime action server
    def send_goal(self):
        goal = MeasureLapTime.Goal()
        self.action_client.wait_for_server()
# Send the goal asynchronously and attach a callback to handle the response
        self.send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
# Get the goal handle from the future
        goal_handle = future.result()
# Check if the goal was accepted or rejected
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
# Log that the goal was accepted and set up to get the result
        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
# Get the result from the future
        result = future.result().result
# Log the total lap time received from the action server
        self.get_logger().info(f'Total lap time: {result.total_time:.2f} seconds')
        self.destroy_node()
        rclpy.shutdown()
#Callback to handle feedback messages during action execution.
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Elapsed time: {feedback.elapsed_time:.2f} seconds')

def main(args=None):
    rclpy.init(args=args)
    lap_time_client = LapTimeActionClient()
    rclpy.spin_once(lap_time_client, timeout_sec=0.1)
    lap_time_client.send_goal()
    rclpy.spin(lap_time_client)

if __name__ == '__main__':
    main()
