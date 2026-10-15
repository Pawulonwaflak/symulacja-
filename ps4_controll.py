#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading

class TeleopWSL(Node):
    def __init__(self):
        super().__init__('ps4_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 9999))
        self.get_logger().info("Czekam na dane z pada (Windows) na porcie UDP 9999...")
        
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
                v, w = data.decode().split(',')
                msg = Twist()
                msg.linear.x = float(v)
                msg.angular.z = float(w)
                self.pub.publish(msg)
            except:
                pass

def main():
    rclpy.init()
    node = TeleopWSL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()