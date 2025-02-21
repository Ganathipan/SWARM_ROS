import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class TurtleBotKeyboardController(Node):
    def __init__(self):
        super().__init__('turtlebot_keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info(
            "\nUse Arrow Keys to Move the TurtleBot:\n"
            "   ↑ : Accelerate Forward\n"
            "   ↓ : Move Backward\n"
            "   ← : Turn Left\n"
            "   → : Turn Right\n"
            "   SPACEBAR : Stop Immediately\n"
            "   q : Quit"
        )
        self.speed = 0.0  # Linear speed
        self.turn = 0.0  # Angular speed
        self.speed_increment = 0.05  # Acceleration rate

    def get_key(self):
        """Function to read keyboard input"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        """Main loop to read keyboard input and send velocity commands"""
        msg = Twist()
        try:
             while rclpy.ok():
                key = self.get_key()

                if key == '\x1b':  # Detect ESC sequence (for arrow keys)
                    key = self.get_key()
                    if key == '[':
                        key = self.get_key()
                        if key == 'A':  # UP arrow (Accelerate forward)
                            self.speed += self.speed_increment
                        elif key == 'B':  # DOWN arrow (Move backward)
                            self.speed -= self.speed_increment
                        elif key == 'C':  # RIGHT arrow (Turn right)
                            self.turn = 0.5
                        elif key == 'D':  # LEFT arrow (Turn left)
                            self.turn = -0.5

                elif key == ' ':  # SPACEBAR (Stop immediately)
                    self.speed = 0.0
                    self.turn = 0.0

                elif key == 'q':  # Quit on 'q'
                    self.get_logger().info("🔴 Stopping TurtleBot...")
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher_.publish(msg)
                    return

                else:  # Reset turning angle if no left/right input
                    self.turn = 0.0

                # Update velocity message
                msg.linear.x = self.speed
                msg.angular.z = self.turn
                self.publisher_.publish(msg)

                self.get_logger().info(f"🔹 Speed: {msg.linear.x:.2f}, Turn: {msg.angular.z:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
        finally:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotKeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

