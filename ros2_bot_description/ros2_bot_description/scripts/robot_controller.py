#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(String, 'voice_commands', self.command_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Robot Controller Node Started")

    def command_callback(self, msg):
        twist = Twist()
        command = msg.data.lower()

        if command == "move forward":
            twist.linear.x = 0.5
            self.get_logger().info("Moving Forward")
        elif command == "turn left":
            twist.angular.z = 0.5
            self.get_logger().info("Turning Left")
        elif command == "turn right":
            twist.angular.z = -0.5
            self.get_logger().info("Turning Right")
        elif command == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Stopping")

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
