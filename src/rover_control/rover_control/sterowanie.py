import math
import rclpy
import csv
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import os

FLIP = [-1, 1, -1, 1, -1, 1]
STEER_SIGN = -1

class Sterowanie(Node):
    def __init__(self):
        super().__init__('sterowanie')

        ######################PARAMETRY#########################
        self.a = self.declare_parameter('a', 0.198).value
        self.b = self.declare_parameter('b', 0.124).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.063).value
        self.rate = self.declare_parameter('rate', 50.0).value
        self.deg_tol = math.radians(self.declare_parameter('realign_tol_deg', 3.0).value)

        #####################WSPOLRZEDNE POLOZENIA KOL W UKLADZIE LOKALNYM ROBOTA#########################
        self.wheel_positions = [(-self.a, self.b), (-self.a, -self.b), (0.0, self.b), (0.0, -self.b), (self.a, self.b), (self.a, -self.b)]
        self.steer_joints = ['rl_steer_joint', 'rr_steer_joint', 'ml_steer_joint',
                             'mr_steer_joint', 'fl_steer_joint', 'fr_steer_joint']
        #####################WEJSCIA STERUJACE#########################
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        ######################WYJSCIA##################################
        self.angles = [0.0] * 6
        self.speeds = [0.0] * 6
        self.steer_pos = [0.0] * 6
        ######################PUBLISHERS###############################
        self.ps = self.create_publisher(
            Float64MultiArray, '/steer_position_controller/commands', 10)
        self.pw = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)

        #######################SUBSCRIBER##############################         
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.js_callback, 10)

        self.create_timer(1/self.rate, self.tick)

    def cmd_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z

    def js_callback(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        self.steer_pos = [pos.get(j, 0.0) for j in self.steer_joints]

    def solve(self):
        angles = []  
        speeds = [] 

        for i, (x, y) in enumerate(self.wheel_positions):
            vx_i = self.vx - self.omega * y
            vy_i = self.vy + self.omega * x

            angle = math.atan2(vy_i, vx_i)
            speed = math.hypot(vx_i, vy_i)

            if angle > math.pi / 2:
                angle -= math.pi
                speed = -speed

            elif angle < -math.pi / 2:
                angle += math.pi
                speed = -speed

            omega_i = speed / self.wheel_radius * FLIP[i]

            angles.append(angle * STEER_SIGN)
            speeds.append(omega_i)    


        return angles, speeds

    def tick(self):
        self.angles, self.speeds = self.solve()
        self.ps.publish(Float64MultiArray(data=self.angles))

        # t = self.get_clock().now().nanoseconds * 1e-9

        for i in range(len(self.angles)):
            err = self.angles[i] - self.steer_pos[i]
            scale = max(0.0, math.cos(err))
            self.speeds[i] *= scale
            # if abs(err) > self.deg_tol:
            #     self.speeds[i] = 0.0
         
        self.pw.publish(Float64MultiArray(data=self.speeds))

    def stop(self):
        self.pw.publish(Float64MultiArray(data=[0.0] * 6))
        self.ps.publish(Float64MultiArray(data=[0.0] * 6))

                  

def main(args=None):
    rclpy.init(args=args)
    node = Sterowanie()

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
