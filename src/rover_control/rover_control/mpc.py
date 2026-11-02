#!/usr/bin/env python3
import math
import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class MPCSterownik(Node):
    def __init__(self):
        super().__init__('mpc_sterownik')

        ###################### PARAMETRY MPC #########################
        self.rate = self.declare_parameter('rate', 10.0).value          # Częstotliwość [Hz]
        self.dt = 1.0 / self.rate                                       # Krok czasowy dt = 0.1 s
        self.N = self.declare_parameter('N', 35).value                  # Horyzont predykcji (35 kroków)

        # Ograniczenia prędkości wysyłanych do sterowanie.py
        self.v_max = self.declare_parameter('v_max', 0.6).value         # Maks. prędkość do przodu [m/s]
        self.v_min = self.declare_parameter('v_min', 0.0).value         # Zablokowany wsteczny (eliminuje oscylacje)
        self.omega_max = self.declare_parameter('omega_max', 1.2).value # Maks. prędkość kątowa [rad/s]

        ###################### WAGI FUNKCJI KOSZTU ###################
        # Wagi błędów stanu (pozycja i orientacja)
        self.w_x = self.declare_parameter('w_x', 15.0).value            # Waga błędu pozycji X
        self.w_y = self.declare_parameter('w_y', 15.0).value            # Waga błędu pozycji Y
        self.w_theta = self.declare_parameter('w_theta', 25.0).value    # Duża waga orientacji (wyrównanie z linią)

        # Wagi wartości sterowania (oszczędzanie energii)
        self.w_v = self.declare_parameter('w_v', 0.1).value             # Waga prędkości liniowej v
        self.w_omega = self.declare_parameter('w_omega', 0.2).value     # Waga prędkości kątowej omega

        # Wagi przyrostów sterowania (mniejsze opóźnienie fazowe przy wychodzeniu z zakrętu)
        self.w_dv = self.declare_parameter('w_dv', 1.5).value           # Kara za szarpnięcia dv
        self.w_domega = self.declare_parameter('w_domega', 0.8).value   # Kara za szarpnięcia domega

        ###################### STAN ROBOTA I ŚCIEŻKA #################
        self.current_state = np.zeros(3)  # [x, y, theta (ciągły)]
        self.prev_u = np.zeros(2)         # Poprzednie sterowanie [v, omega]
        self.odom_received = False
        self.path_points = []             # Lista punktów [(x, y, theta_ref), ...]
        self.last_closest_idx = 0         # Zapobiega cofaniu się indeksu na narożnikach
        self.warm_start_ready = False

        ###################### INICJALIZACJA SOLVERA #################
        self.setup_mpc()

        ###################### PUBLISHERS & SUBSCRIBERS ##############
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom_ground_truth', self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10)

        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'Węzeł MPC uruchomiony (rate={self.rate}Hz, N={self.N})')

    def setup_mpc(self):
        """Buduje strukturę problemu optymalizacji w CasADi."""
        self.opti = ca.Opti()

        # Zmienne decyzyjne w horyzoncie N
        self.X = self.opti.variable(3, self.N + 1)  # Stany: [x, y, theta] dla k = 0..N
        self.U = self.opti.variable(2, self.N)      # Sterowania: [v, omega] dla k = 0..N-1

        # Parametry wejściowe (aktualizowane w każdym kroku czasowym)
        self.P_x0 = self.opti.parameter(3)          # Aktualny stan początkowy [x0, y0, theta0]
        self.P_u_prev = self.opti.parameter(2)      # Poprzednie sterowanie [v_prev, omega_prev]
        self.P_ref = self.opti.parameter(3, self.N) # Okno N punktów referencyjnych [x_ref, y_ref, theta_ref]

        # Warunek początkowy
        self.opti.subject_to(self.X[:, 0] == self.P_x0)

        cost = 0
        for k in range(self.N):
            # Bieżące zmienne stanu, sterowania i referencji w kroku k
            x_k, y_k, theta_k = self.X[0, k], self.X[1, k], self.X[2, k]
            v_k, omega_k = self.U[0, k], self.U[1, k]
            x_ref, y_ref, theta_ref = self.P_ref[0, k], self.P_ref[1, k], self.P_ref[2, k]

            # 1. Model kinematyczny (dyskretyzacja Eulera)
            x_next = x_k + v_k * ca.cos(theta_k) * self.dt
            y_next = y_k + v_k * ca.sin(theta_k) * self.dt
            theta_next = theta_k + omega_k * self.dt

            self.opti.subject_to(self.X[0, k + 1] == x_next)
            self.opti.subject_to(self.X[1, k + 1] == y_next)
            self.opti.subject_to(self.X[2, k + 1] == theta_next)

            # 2. Koszt błędu trajektorii i orientacji
            cost += self.w_x * (x_k - x_ref)**2
            cost += self.w_y * (y_k - y_ref)**2
            cost += self.w_theta * (1.0 - ca.cos(theta_k - theta_ref))

            # 3. Koszt energii sterowania
            cost += self.w_v * (v_k**2) + self.w_omega * (omega_k**2)

            # 4. Koszt przyrostów sterowania (delta U)
            if k == 0:
                dv = v_k - self.P_u_prev[0]
                domega = omega_k - self.P_u_prev[1]
            else:
                dv = v_k - self.U[0, k - 1]
                domega = omega_k - self.U[1, k - 1]

            cost += self.w_dv * (dv**2) + self.w_domega * (domega**2)

        # Koszt końcowy (terminal cost) dla ostatniego punktu k = N (wzmocniony x2 dla stabilności)
        cost += 2.0 * self.w_x * (self.X[0, self.N] - self.P_ref[0, self.N - 1])**2
        cost += 2.0 * self.w_y * (self.X[1, self.N] - self.P_ref[1, self.N - 1])**2
        cost += 2.0 * self.w_theta * (1.0 - ca.cos(self.X[2, self.N] - self.P_ref[2, self.N - 1]))

        self.opti.minimize(cost)

        # Ograniczenia na sterowania
        self.opti.subject_to(self.opti.bounded(self.v_min, self.U[0, :], self.v_max))
        self.opti.subject_to(self.opti.bounded(-self.omega_max, self.U[1, :], self.omega_max))

        # Konfiguracja solvera IPOPT
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 100,
            'ipopt.acceptable_tol': 1e-4,
            'ipopt.warm_start_init_point': 'yes'
        }
        self.opti.solver('ipopt', opts)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # Konwersja kwaternionu na kąt yaw (theta)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_theta = math.atan2(siny_cosp, cosy_cosp)

        # Rozwijanie kąta (angle unwrapping), aby uniknąć skoków o 2*pi przy +-pi
        if self.odom_received:
            prev_theta = self.current_state[2]
            diff = math.atan2(math.sin(raw_theta - prev_theta),
                              math.cos(raw_theta - prev_theta))
            theta = prev_theta + diff
        else:
            theta = raw_theta

        self.current_state = np.array([x, y, theta])
        self.odom_received = True

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return

        raw_points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            q = pose_stamped.pose.orientation

            # Odczyt kąta orientacji bezpośrednio z wiadomości Path
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta_ref = math.atan2(siny_cosp, cosy_cosp)

            raw_points.append((x, y, theta_ref))

        # Usunięcie zduplikowanych punktów w narożnikach (gdy dwa punkty mają to samo (x, y),
        # zachowujemy ten z nowym kierunkiem jazdy)
        filtered_points = []
        for i, pt in enumerate(raw_points):
            if i < len(raw_points) - 1:
                next_pt = raw_points[i + 1]
                if math.hypot(next_pt[0] - pt[0], next_pt[1] - pt[1]) < 1e-4:
                    continue
            filtered_points.append(pt)

        # Jeśli to ta sama ścieżka publikowana cyklicznie co 1 s, nie resetuj postępu
        if len(filtered_points) != len(self.path_points):
            self.last_closest_idx = 0

        self.path_points = filtered_points

    def get_reference_window(self):
        """Znajduje najbliższy punkt na ścieżce (tylko w przód) i buduje okno N punktów."""
        rx, ry, rtheta = self.current_state

        # Szukanie najbliższego punktu w oknie od ostatniego indeksu w przód (zapobiega cofaniu za zakręt)
        search_end = min(self.last_closest_idx + 25, len(self.path_points))
        dists = [
            (self.path_points[i][0] - rx)**2 + (self.path_points[i][1] - ry)**2
            for i in range(self.last_closest_idx, search_end)
        ]
        closest_idx = self.last_closest_idx + int(np.argmin(dists))
        self.last_closest_idx = closest_idx

        # Sprawdzenie błędu kursowego względem najbliższego punktu ścieżki
        _, _, theta_at_closest = self.path_points[closest_idx]
        heading_err = abs(math.atan2(math.sin(rtheta - theta_at_closest),
                                     math.cos(rtheta - theta_at_closest)))

        # Jeśli błąd orientacji przekracza ~35 stopni (np. na ostrym zakręcie 90 st.),
        # zatrzymujemy przesuwanie okna w przód, aby robot najpierw obrócił się w miejscu
        if heading_err > 0.60:
            step_scale = 0.0
        elif heading_err > 0.30:
            step_scale = 0.5
        else:
            step_scale = 1.0

        ref_window = np.zeros((3, self.N))
        for k in range(self.N):
            offset = int(round((k + 1) * step_scale))
            idx = min(closest_idx + offset, len(self.path_points) - 1)
            ref_window[0, k] = self.path_points[idx][0]
            ref_window[1, k] = self.path_points[idx][1]
            ref_window[2, k] = self.path_points[idx][2]

        return ref_window, closest_idx

    def tick(self):
        if not self.odom_received or not self.path_points:
            return

        # Sprawdzenie, czy łazik dojechał do końca ścieżki (< 10 cm od mety i blisko końca indeksów)
        goal_x, goal_y, _ = self.path_points[-1]
        dist_to_goal = math.hypot(goal_x - self.current_state[0], goal_y - self.current_state[1])
        if dist_to_goal < 0.10 and self.last_closest_idx >= len(self.path_points) - 5:
            self.publish_cmd(0.0, 0.0)
            self.prev_u = np.zeros(2)
            return

        # 1. Przygotowanie okna N punktów referencyjnych
        ref_window, _ = self.get_reference_window()

        # 2. Przekazanie aktualnych parametrów do problemu CasADi
        self.opti.set_value(self.P_x0, self.current_state)
        self.opti.set_value(self.P_u_prev, self.prev_u)
        self.opti.set_value(self.P_ref, ref_window)

        # Inicjalizacja zmiennych decyzyjnych przy pierwszym uruchomieniu
        if not self.warm_start_ready:
            init_X = np.tile(self.current_state.reshape(3, 1), (1, self.N + 1))
            self.opti.set_initial(self.X, init_X)
            self.opti.set_initial(self.U, np.zeros((2, self.N)))

        # 3. Rozwiązanie problemu optymalizacji
        try:
            sol = self.opti.solve()
            u_opt = sol.value(self.U[:, 0])
            x_sol = sol.value(self.X)
            u_sol = sol.value(self.U)

            # Przesunięty warm-start (shift o 1 krok w przód)
            x_next_init = np.hstack((x_sol[:, 1:], x_sol[:, -1:]))
            u_next_init = np.hstack((u_sol[:, 1:], u_sol[:, -1:]))
            self.opti.set_initial(self.X, x_next_init)
            self.opti.set_initial(self.U, u_next_init)
            self.warm_start_ready = True

            v_cmd = float(u_opt[0])
            omega_cmd = float(u_opt[1])

        except RuntimeError:
            self.get_logger().warn('Solver MPC nie znalazł optimum! Zatrzymuję awaryjnie.')
            v_cmd, omega_cmd = 0.0, 0.0
            self.warm_start_ready = False

        # 4. Publikacja na /cmd_vel i zapisanie poprzedniego sterowania
        self.publish_cmd(v_cmd, omega_cmd)
        self.prev_u = np.array([v_cmd, omega_cmd])

    def publish_cmd(self, vx: float, omega: float):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0.0
        msg.angular.z = omega
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCSterownik()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymywanie MPC...')
        node.publish_cmd(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()