#!/usr/bin/env python3
"""
Test skoordynowanego ruchu - zadajesz promien skretu, skrypt liczy
i publikuje KOMPLET katow i predkosci.

    python3 arc_test.py 0.5 0.25      # R = 0.5 m w lewo, 0.25 m/s
    python3 arc_test.py -0.5 0.25     # R = 0.5 m w prawo
    python3 arc_test.py 0 0.25        # jazda prosto
    python3 arc_test.py 0.5 0         # sam skret, bez jazdy

Ctrl+C zatrzymuje i zeruje.
"""

import sys
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# --- geometria (z joint_map.py) ---------------------------------------
A = 0.198          # os srodkowa -> os skrajna [m]
B = 0.124          # polowa rozstawu [m]
R_WHEEL = 0.063    # <-- PODMIEN na swoj promien kola [m]

# Osie kol sa lustrzane: Rev 17/41/42 maja +X, Rev 22/39/40 maja -X.
# Dopoki nie ujednolicisz tego w URDF, kompensujemy tutaj.
# Kolejnosc jak w controllers.yaml: [17, 22, 41, 39, 42, 40]
FLIP = [1, -1, 1, -1, 1, -1]


class ArcTest(Node):
    def __init__(self, R, v):
        super().__init__('arc_test')
        self.R, self.v = R, v

        self.ps = self.create_publisher(
            Float64MultiArray, '/steer_position_controller/commands', 10)
        self.pw = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)

        self.angles, self.speeds = self.solve()

        print(f'\nR = {"prosto" if R == 0 else f"{R:+.3f} m"}   v = {v:.3f} m/s')
        print(f'katy   [fl fr rl rr] = '
              f'{[round(math.degrees(x), 1) for x in self.angles]} st')
        print(f'omega  [fl fr ml mr rl rr] = '
              f'{[round(x, 2) for x in self.speeds]} rad/s\n')

        self.create_timer(0.05, self.tick)

    def solve(self):
        if abs(self.R) < 1e-6:
            angles = [0.0] * 6
            v = [self.v] * 6
        else:
            R, s = abs(self.R), math.copysign(1.0, self.R)
            d_in = math.atan2(A, R - B)
            d_out = math.atan2(A, R + B)

            d_fl, d_fr = (d_in, d_out) if s > 0 else (-d_out, -d_in)
            angles = [d_fl, d_fr, 0, 0, -d_fl, -d_fr]

            r_in = math.hypot(A, R - B)
            r_out = math.hypot(A, R + B)
            m_in, m_out = R - B, R + B

            radii = ([r_in, r_out, m_in, m_out, r_in, r_out] if s > 0
                     else [r_out, r_in, m_out, m_in, r_out, r_in])
            v = [self.v * r / R for r in radii]

        omega = [x / R_WHEEL * f for x, f in zip(v, FLIP)]
        return angles, omega

    def tick(self):
        self.ps.publish(Float64MultiArray(data=self.angles))
        self.pw.publish(Float64MultiArray(data=self.speeds))

    def stop(self):
        self.pw.publish(Float64MultiArray(data=[0.0] * 6))
        self.ps.publish(Float64MultiArray(data=[0.0] * 4))


def main():
    R = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    v = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25

    rclpy.init()
    node = ArcTest(R, v)

    node.ps.publish(Float64MultiArray(data=node.angles))
    print('ustawiam kola...')
    time.sleep(1.5)
    print('jazda (Ctrl+C konczy)\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
        time.sleep(0.3)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()