#Lap time is the time that takes the robot to go from the starting position back to the starting position
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from navigation_robot_custom_interfaces.action import MeasureLapTime
import math

class LapTimeServiceServer(Node):
    def __init__(self):
        super().__init__('lap_time_service_server')
        # Define a callback group to allow reentrant callbacks
        self.callback_group = ReentrantCallbackGroup()
        #create server to measure lap time
        self.action_server = ActionServer(self,MeasureLapTime,'measure_lap_time',self.execute_callback)
        self.get_logger().info('LapTime server started!')
        self.reset_state()
#initialization
    def reset_state(self):
        self.current_position = (0.0, 0.0)
        self.start_time = None
        self.lap_active = False
        self.current_time=0.0
        self.feedback_timer = None
#getting updated position from odometry       
    def odometry_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def execute_callback(self, goal_handle):
        self.get_logger().info('LapTime request received. Waiting for robot to get to the starting point')
#wait for robot to get to the starting point       
        if not self.wait_for_starting_point():
            goal_handle.abort()
            return MeasureLapTime.Result()
#start lap timing 
        self.start_time = self.current_time
        self.lap_active = True
        feedback_msg = MeasureLapTime.Feedback()
# Timer callback to send feedback at regular intervals
        def timer_callback():
            if not self.lap_active:
                return
            feedback_msg.elapsed_time = self.current_time - self.start_time
            goal_handle.publish_feedback(feedback_msg)

        self.create_timer(0.2, timer_callback)
# Wait for the robot to complete the lap
        if not self.wait_for_lap_completion():
            goal_handle.abort()
            return MeasureLapTime.Result()

        result = MeasureLapTime.Result()
        result.total_time = self.current_time - self.start_time
        self.lap_active = False
        goal_handle.succeed()
        return result

# Spin until the robot is at the starting point
    def wait_for_starting_point(self):
        tolerance = 0.1
        start_x, start_y = self.current_position

        while not self.is_at_starting_point(start_x, start_y, tolerance):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Robot is at the starting point. Starting lap timing.')
        return True
#Check if the robot is at the starting point within a given tolerance.
    def is_at_starting_point(self, start_x, start_y, tolerance=0.1):
        current_x, current_y = self.current_position
        distance = math.sqrt((current_x - start_x) ** 2 + (current_y - start_y) ** 2)
        return distance < tolerance

    def wait_for_lap_completion(self):
        start_x, start_y = self.current_position
        tolerance = 0.1
        laps_needed = 1 

        self.get_logger().info('Waiting for the robot to complete the lap...')
        while laps_needed > 0:
            if self.is_at_starting_point(start_x, start_y, tolerance):
                self.get_logger().info(f'Robot has reached the starting point again. Laps left: {laps_needed - 1}')
                laps_needed -= 1
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Lap completed.')
        return True

def main(args=None):
    rclpy.init(args=args)
    lap_time_service_server = LapTimeServiceServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lap_time_service_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    lap_time_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
