import numpy as np
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrajektoriaBenchmark(Node):
    def __init__(self):
        super().__init__('trajektoria_benchmark')

        ###################### PARAMETRY TRAJEKTORII ######################
        self.v_ref = self.declare_parameter('v_ref', 0.5).value            # Prędkość referencyjna [m/s]
        self.dt_mpc = self.declare_parameter('dt_mpc', 0.1).value          # Krok czasowy [s]
        self.frame_id = self.declare_parameter('frame_id', 'base_footprint').value  # Globalny układ odniesienia

        ###################### PUBLISHER #############################
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # Wygenerowanie ścieżki raz przy starcie węzła
        self.path_msg = self.generate_path()

        # Publikacja ścieżki co 1 sekundę
        self.create_timer(1.0, self.publish_path)
        self.get_logger().info(
            f'Wygenerowano zaktualizowany tor benchmarkowy, '
            f'liczba punktów: {len(self.path_msg.poses)}'
        )

    def generate_path(self) -> Path:
        ds = self.v_ref * self.dt_mpc  # Krok przestrzenny [m]

        all_x = []
        all_y = []
        all_yaw = []

        # Początkowa pozycja i kierunek
        curr_x = 0.0
        curr_y = 0.0
        curr_yaw = 0.0

        def add_straight(length):
            nonlocal curr_x, curr_y, curr_yaw
            if length <= 0:
                return
            
            num_points = max(1, int(round(length / ds)))
            distances = np.linspace(0.0, length, num_points, endpoint=False)
            
            for d in distances:
                x = curr_x + d * np.cos(curr_yaw)
                y = curr_y + d * np.sin(curr_yaw)
                all_x.append(x)
                all_y.append(y)
                all_yaw.append(curr_yaw)
                
            curr_x += length * np.cos(curr_yaw)
            curr_y += length * np.sin(curr_yaw)

        def add_arc(radius, sweep_angle_deg, direction='left'):
            nonlocal curr_x, curr_y, curr_yaw
            
            sweep_angle_rad = math.radians(sweep_angle_deg)
            arc_length = radius * sweep_angle_rad
            num_points = max(2, int(round(arc_length / ds)))
            
            # direction_sign: 1 dla skrętu w lewo (CCW), -1 dla skrętu w prawo (CW)
            dir_sign = 1.0 if direction == 'left' else -1.0
            
            thetas = np.linspace(0.0, sweep_angle_rad, num_points, endpoint=False)
            
            # Środek obrotu dla danego łuku
            cx = curr_x - dir_sign * radius * np.sin(curr_yaw)
            cy = curr_y + dir_sign * radius * np.cos(curr_yaw)
            
            start_angle_from_center = curr_yaw - dir_sign * (np.pi / 2.0)

            for theta in thetas:
                angle_from_center = start_angle_from_center + dir_sign * theta
                x = cx + radius * np.cos(angle_from_center)
                y = cy + radius * np.sin(angle_from_center)
                yaw = curr_yaw + dir_sign * theta
                
                # Zapewnienie ciągłości kąta [-pi, pi]
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                
                all_x.append(x)
                all_y.append(y)
                all_yaw.append(yaw)

            curr_yaw += dir_sign * sweep_angle_rad
            curr_yaw = math.atan2(math.sin(curr_yaw), math.cos(curr_yaw))
            curr_x = cx + radius * np.cos(start_angle_from_center + dir_sign * sweep_angle_rad)
            curr_y = cy + radius * np.sin(start_angle_from_center + dir_sign * sweep_angle_rad)

        # ==========================================================
        # BUDOWA TRASY BENCHMARKOWEJ
        # ==========================================================
        
        # 1. Krótka prosta startowa (1.0m zamiast 2.5m)
        add_straight(1.0)
        
        # 2. Łagodny zakręt w lewo o 90 stopni
        add_arc(radius=1.5, sweep_angle_deg=90.0, direction='left')
        
        # 3. Krótka prosta
        add_straight(1.0)
        
        # 4. Szeroki, łagodny nawrót w prawo o 180 stopni (promień 3.5m zamiast 2.0m)
        add_arc(radius=3.5, sweep_angle_deg=180.0, direction='right')
        
        # 5. Prosta po nawrocie
        add_straight(1.5)
        
        # 6. Eska (Slalom) - odrobinę szersza do płynnej demonstracji
        add_arc(radius=1.2, sweep_angle_deg=90.0, direction='left')
        add_arc(radius=1.2, sweep_angle_deg=90.0, direction='right')
        
        # 7. Prosta do mety
        add_straight(2.0)
        
        # ==========================================================

        # --- BUDOWA WIADOMOŚCI nav_msgs/Path ---
        path = Path()
        path.header.frame_id = self.frame_id

        for x, y, yaw in zip(all_x, all_y, all_yaw):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # Orientacja yaw w postaci kwaternionu
            pose.pose.orientation.z = float(np.sin(yaw / 2.0))
            pose.pose.orientation.w = float(np.cos(yaw / 2.0))
            
            path.poses.append(pose)

        return path

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajektoriaBenchmark()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()