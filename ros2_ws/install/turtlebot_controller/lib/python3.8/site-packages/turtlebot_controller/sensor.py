"""
TurtleBot Sensor Integration

This script defines classes for integrating sensors with the TurtleBot.
It subscribes to sensor topics and processes sensor data, and also simulates distance measurements.

Dependencies:
- rclpy: ROS 2 Python library
- sensor_msgs.msg: Contains message types for sensor data

Author: S. Ganathipan
Date: 21/02/2025
"""

import rclpy  # Import ROS 2 client library for Python
from rclpy.node import Node  # Import Node class to create a ROS 2 node
from sensor_msgs.msg import LaserScan  # Import LaserScan message type for lidar data

class TurtleBotSensors(Node):
    """
    ROS 2 Node for integrating sensors with the TurtleBot.
    Subscribes to sensor topics and processes sensor data.
    """
    def __init__(self):
        super().__init__('turtlebot_sensors')  # Initialize node with a unique name
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )  # Subscribe to the /scan topic for lidar data
        self.subscription  # Prevent unused variable warning

    def laser_scan_callback(self, msg):
        """
        Callback function to process incoming LaserScan messages.
        Displays the distance to the closest obstacle ahead.
        """
        if msg.ranges:
            min_distance = min(msg.ranges)  # Find the minimum distance in the scan data
            self.get_logger().info(f"Distance to the closest obstacle: {min_distance:.2f} meters")

class CustomDistanceSensor(Node):
    """
    ROS 2 Node for a custom distance measuring sensor.
    Simulates distance measurements and provides the distance to the closest obstacle.
    """
    def __init__(self):
        super().__init__('custom_distance_sensor')  # Initialize node with a unique name
        self.timer = self.create_timer(1.0, self.timer_callback)  # Create a timer to simulate sensor readings

    def timer_callback(self):
        """
        Timer callback function to simulate distance measurements.
        """
        import random
        distance = random.uniform(0.5, 5.0)  # Simulate a random distance measurement between 0.5 and 5 meters
        self.get_logger().info(f"Simulated distance to the closest obstacle: {distance:.2f} meters")
