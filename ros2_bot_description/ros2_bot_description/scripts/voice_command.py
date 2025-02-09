#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import speech_recognition as sr
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.get_logger().info("Voice Command Node Started. Speak a command...")

        self.timer = self.create_timer(5.0, self.listen_command)  # Listen every 5 sec

    def listen_command(self):
        with self.microphone as source:
            self.get_logger().info("Listening for a command...")
            self.recognizer.adjust_for_ambient_noise(source)
            try:
                audio = self.recognizer.listen(source, timeout=5)
                command = self.recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Recognized Command: {command}")

                valid_commands = ["move forward", "turn left", "turn right", "stop"]
                for cmd in valid_commands:
                    if cmd in command:
                        msg = String()
                        msg.data = cmd
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published: {cmd}")
                        return
                self.get_logger().info("Invalid command. Try again.")

            except sr.UnknownValueError:
                self.get_logger().info("Could not understand the command.")
            except sr.RequestError:
                self.get_logger().info("Error with the speech recognition service.")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
