import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ganathipan/SWARM_ROS/install/turtlebot_controller'
