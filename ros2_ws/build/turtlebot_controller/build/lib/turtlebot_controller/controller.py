"""
TurtleBot Keyboard Controller

This script allows users to control a TurtleBot using keyboard inputs.
The arrow keys are used to move the robot forward, backward, left, and right.
Pressing SPACEBAR stops the robot immediately, and pressing 'q' exits the program.

Dependencies:
- rclpy: ROS 2 Python library
- geometry_msgs.msg: Contains the Twist message type for velocity control
- sys, termios, tty, select: Used for reading keyboard input in real-time

Author: S. Ganathipan
Date: 21/02/2025
"""

import rclpy  # Import ROS 2 client library for Python
from rclpy.node import Node  # Import Node class to create a ROS 2 node
from geometry_msgs.msg import Twist  # Import Twist message type for velocity commands
import sys  # Import system-specific parameters and functions
import termios  # Import POSIX terminal control for raw keyboard input handling
import tty  # Import terminal handling module for character-based input
import select  # Import I/O multiplexing to check for input availability
from turtlebot_controller.sensor import TurtleBotSensors, CustomDistanceSensor  # Import the sensor classes
import threading  # Import threading module for running nodes in separate threads

class TurtleBotKeyboardController(Node):
    """
    ROS 2 Node for controlling a TurtleBot using keyboard inputs.
    Publishes velocity commands to the /cmd_vel topic based on keypresses.
    """
    def __init__(self):
        super().__init__('turtlebot_keyboard_controller')  # Initialize node with a unique name
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # Create a publisher for velocity commands
        self.sensors = TurtleBotSensors()  # Initialize the sensor integration
        self.custom_sensor = CustomDistanceSensor()  # Initialize the custom distance sensor
        
        # Display control instructions in the console
        self.get_logger().info(
            "\nUse Arrow Keys to Move the TurtleBot:\n"
            "   ↑ : Accelerate Forward\n"
            "   ↓ : Move Backward\n"
            "   ← : Turn Left\n"
            "   → : Turn Right\n"
            "   SPACEBAR : Stop Immediately\n"
            "   q : Quit"
        )
        
        self.speed = 0.0  # Linear speed of the robot (meters per second)
        self.turn = 0.0  # Angular speed of the robot (radians per second)
        self.speed_increment = 0.05  # Increment value for acceleration/deceleration

    def get_key(self):
        """
        Reads a single character from keyboard input without requiring Enter key press.
        Uses raw terminal settings to capture key events immediately.
        """
        old_settings = termios.tcgetattr(sys.stdin)  # Save current terminal settings
        try:
            tty.setraw(sys.stdin.fileno())  # Set terminal to raw mode for direct character reading
            select.select([sys.stdin], [], [], 0)  # Check if there is an available key press
            key = sys.stdin.read(1)  # Read a single character from standard input
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)  # Restore terminal settings
        return key  # Return the captured key

    def rotate_x(self):
        """
        Rotate the TurtleBot around the x-axis.
        """
        msg = Twist()
        msg.angular.x = 0.5  # Set angular velocity for x-axis rotation
        self.publisher_.publish(msg)
        self.get_logger().info("🔄 Rotating around x-axis")

    def rotate_y(self):
        """
        Rotate the TurtleBot around the y-axis.
        """
        msg = Twist()
        msg.angular.y = 0.5  # Set angular velocity for y-axis rotation
        self.publisher_.publish(msg)
        self.get_logger().info("🔄 Rotating around y-axis")

    def rotate_z(self):
        """
        Rotate the TurtleBot around the z-axis.
        """
        msg = Twist()
        msg.angular.z = 0.5  # Set angular velocity for z-axis rotation
        self.publisher_.publish(msg)
        self.get_logger().info("🔄 Rotating around z-axis")

    def run(self):
        """
        Main loop that listens for keyboard inputs and sends velocity commands.
        """
        msg = Twist()  # Create a Twist message instance for velocity control
        try:
            while rclpy.ok():  # Keep running while ROS is active
                key = self.get_key()  # Capture keyboard input

                if key == '\x1b':  # Check for escape sequence (arrow keys start with ESC)
                    key = self.get_key()
                    if key == '[':
                        key = self.get_key()
                        if key == 'A':  # UP arrow key pressed - Move forward
                            self.speed += self.speed_increment  # Increase speed
                        elif key == 'B':  # DOWN arrow key pressed - Move backward
                            self.speed -= self.speed_increment  # Decrease speed
                        elif key == 'C':  # RIGHT arrow key pressed - Turn right
                            self.turn = 0.5  # Set angular velocity for right turn
                        elif key == 'D':  # LEFT arrow key pressed - Turn left
                            self.turn = -0.5  # Set angular velocity for left turn

                elif key == ' ':  # SPACEBAR pressed - Stop movement immediately
                    self.speed = 0.0  # Reset linear speed to zero
                    self.turn = 0.0  # Reset angular speed to zero

                elif key == 'q':  # Quit the program when 'q' is pressed
                    self.get_logger().info("🔴 Stopping TurtleBot...")  # Log shutdown message
                    msg.linear.x = 0.0  # Set linear speed to zero
                    msg.angular.z = 0.0  # Set angular speed to zero
                    self.publisher_.publish(msg)  # Publish stop command
                    return  # Exit the function
                elif key == 'x':  # Rotate around x-axis
                    self.rotate_x()
                elif key == 'y':  # Rotate around y-axis
                    self.rotate_y()
                elif key == 'z':  # Rotate around z-axis
                    self.rotate_z()
                else:
                    self.turn = 0.0  # Reset turn value if no left/right key is pressed

                # Assign updated speed values to the Twist message
                msg.linear.x = self.speed  # Set linear speed
                msg.angular.z = self.turn  # Set angular speed
                self.publisher_.publish(msg)  # Publish velocity command to the topic

                # Log the current speed and turn values for debugging
                self.get_logger().info(f"🔹 Speed: {msg.linear.x:.2f}, Turn: {msg.angular.z:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")  # Log any exception that occurs
        finally:
            # Ensure the robot stops if an error occurs
            msg.linear.x = 0.0  # Reset linear speed to zero
            msg.angular.z = 0.0  # Reset angular speed to zero
            self.publisher_.publish(msg)  # Publish stop command


def main(args=None):
    """
    Initializes the ROS 2 node and starts the keyboard controller.
    """
    rclpy.init(args=args)  # Initialize ROS 2 communication
    node = TurtleBotKeyboardController()  # Create an instance of the keyboard controller
    sensor_node = TurtleBotSensors()  # Create an instance of the sensor node
    custom_sensor_node = CustomDistanceSensor()  # Create an instance of the custom sensor node

    # Spin the sensor nodes in separate threads
    sensor_thread = threading.Thread(target=rclpy.spin, args=(sensor_node,), daemon=True)
    custom_sensor_thread = threading.Thread(target=rclpy.spin, args=(custom_sensor_node,), daemon=True)
    sensor_thread.start()
    custom_sensor_thread.start()

    node.run()  # Start listening for keyboard inputs and controlling the robot

    node.destroy_node()  # Properly destroy the node when done
    sensor_node.destroy_node()  # Properly destroy the sensor node when done
    custom_sensor_node.destroy_node()  # Properly destroy the custom sensor node when done
    rclpy.shutdown()  # Shutdown ROS 2 to clean up resources


if __name__ == '__main__':
    main()  # Execute main function if script is run directly
