import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/razanhmede/session7_navigation_robot/session7_assignment_razan/install/session7_turtlebot3_robot'
