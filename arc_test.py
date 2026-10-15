#!/usr/bin/env python3
"""
Test skoordynowanego ruchu - sterownik w dziedzinie prędkości.
Nasłuchuje poleceń vx i omega na temacie /cmd_vel.

Ctrl+C zatrzymuje i zeruje.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

# --- geometria (z joint_map.py) ---------------------------------------
A = 0.198          # os srodkowa -> os skrajna [m]
B = 0.124          # polowa rozstawu [m]
R_WHEEL = 0.063    # <-- PODMIEN na swoj promien kola [m]

# Osie kol sa lustrzane: Rev 17/41/42 maja +X, Rev 22/39/40 maja -X.
# Dopoki nie ujednolicisz tego w URDF, kompensujemy tutaj.
# Kolejnosc jak w controllers.yaml: [17, 22, 41, 39, 42, 40]
FLIP = [1, -1, 1, -1, 1, -1]

WHEEL_COORDS = [
    (A, B),
    (A, -B),
    (0, B),
    (0, -B),
    (-A, B),
    (-A, -B)
]

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_kinematics_node')
        
        self.vx = 0.0
        self.omega = 0.0
        
        self.angles = [0.0] * 6
        self.speeds = [0.0] * 6

        self.ps = self.create_publisher(
            Float64MultiArray, '/steer_position_controller/commands', 10)
        self.pw = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
            
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.create_timer(0.05, self.tick)
        self.get_logger().info('Węzeł uruchomiony. Czekam na /cmd_vel...')

    def cmd_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.omega = msg.angular.z

    def solve(self):
        angles = []
        speeds = []
        
        for i, (x, y) in enumerate(WHEEL_COORDS):
            vx_k = self.vx - self.omega * y
            vy_k = self.omega * x
            
            speed = math.hypot(vx_k, vy_k)
            angle = math.atan2(vy_k, vx_k)
            
            if angle > math.pi / 2:
                angle -= math.pi
                speed = -speed
            elif angle < -math.pi / 2:
                angle += math.pi
                speed = -speed
                
            omega_wheel = (speed / R_WHEEL) * FLIP[i]
            
            angles.append(angle)
            speeds.append(omega_wheel)
            
        return angles, speeds

    def tick(self):
        self.angles, self.speeds = self.solve()
        self.ps.publish(Float64MultiArray(data=self.angles))
        self.pw.publish(Float64MultiArray(data=self.speeds))

    def stop(self):
        self.pw.publish(Float64MultiArray(data=[0.0] * 6))
        self.ps.publish(Float64MultiArray(data=[0.0] * 6))


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymywanie robota...')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()