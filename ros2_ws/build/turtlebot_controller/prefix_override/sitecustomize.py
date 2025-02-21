import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ganathipan/SWARM_ROS/ros2_ws/install/turtlebot_controller'
