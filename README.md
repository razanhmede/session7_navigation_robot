# session7_navigation_robot
## Overview
This project involves a ROS 2-based navigation system for a robot equipped with a wall detection service and lap timing functionality. The project includes the following key components:

- **robot_driver**: Handles the robot’s movement and interacts with the wall detection service.
- **wall_finder_service_server**: A service that detects walls and logs their distance and angle.
- **lap_time_action_server**: Measures the time taken to complete a lap.
- **lap_time_action_client**: Requests lap timing from the action server and processes the results.
  
## Usage
1- **Clone the repository**
2- **Build the package**:

 ```bash
   colcon build
 ```
3- **Source the workspace**:

 ```bash
   source install/setup.bash
 ```
4- **Run the turtlebot3 simulation**

5- **Run the launch file**
 ```bash
 ros2 launch session7_turtlebot3_robot navigation_robot_launch.py
 ```

