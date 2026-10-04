import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray

# Kolejnosc musi byc tak jak w controllers.yaml
STEER_ORDER = ['rl', 'rr', 'ml', 'mr', 'fl', 'fr'] #######cos sie chyba nie zgadza z kolejnoscia bo sie zlizga
WHEEL_ORDER = ['rl', 'rr', 'ml', 'mr', 'fl', 'fr']

# TRYBY STEROWANIA I PRZEŁĄCZANIE MIĘDZY NIMI
DRIVE, REALIGN, SPOT, CRAB = 'DRIVE', 'REALIGN', 'SPOT', 'CRAB'

class Kinematyka(Node):
    def __init__(self):
        super().__init__('kinematyka')

        self.a = self.declare_parameter('a', 0.198).value
        self.b = self.declare_parameter('b', 0.123).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.063).value
        self.angle_max = math.radians(self.declare_parameter('angle_max_deg', 60.0).value)
        self.rate = self.declare_parameter('rate', 50.0).value
        self.kappa_rate = self.declare_parameter('kappa_rate', 1.5).value #####okresla o jaka wartosc moze zmienic sie kappa w ciagu jednej iteracji, bez tego parametru punkt ICR przesuwa sie natychmiastowo, co mogloby spowodowac nieprawidlowe dzialanie sterowania
        self.deg_tol = math.radians(self.declare_parameter('realign_tol_deg', 2.0).value)
        self.v_max = self.declare_parameter('v_max', 1.0).value
        self.omega_max_spot = self.declare_parameter('omega_max_spot', 0.6).value
        #PRZYPISANIE MIEJSCA DANYCH W ODEBRANEJ TABLICY
        self.axis_steer = self.declare_parameter('axis_steer', 0).value
        self.axis_velocity = self.declare_parameter('axis_velocity', 1).value
        self.axis_type = self.declare_parameter('axis_type', 0).value
        self.button_mode = self.declare_parameter('button_mode', 0).value

        self.R_min = self.b + self.a / math.tan(self.angle_max)
        self.kappa_max = 1.0 / self.R_min
        self.dt = 1.0 / self.rate

        self.delta_skid = math.atan2(self.a, self.b)

        self.skret_pub = self.create_publisher(Float64MultiArray, '/steer_position_controller/commands', 10)
        self.predkosc_pub = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Float32MultiArray, '/rover_cmd', self.drive_mode_callback, 10)
        self.create_timer(self.dt, self.update)

        ###################AKTUALNY STAN#########################
        self.delta_now = {}
        self.delta_cmd = [0.0] * 4
        self.mode_number = 0
        self.mode = DRIVE
        self.next_mode = DRIVE
        self.kappa = 0.0
        self.steer_input = 0.0
        self.velocity_input = 0.0
        self.btn_prev = 0

    def drive_mode_callback(self, msg):
        self.type = msg.data[self.axis_type]
        if self.type == 1:
            self.next_mode = SPOT
        elif self.type == 2:
            self.next_mode = CRAB
        else:
            self.next_mode = DRIVE
        if self.mode != REALIGN and self.next_mode != self.mode:
            self.mode = REALIGN
            self.kappa = 0.0    

    #ros2 interface show sensor_msgs/msg/Joy - ukazuej interface wiadomosci Joy - float32[] axes - pomiary z osi joysticka, int32[] buttons - stan przycisku, 0 - nie wcisniety, 1 - wcisniety
    def joy_callback(self, msg):
        self.steer_input = msg.axes[self.axis_steer]
        self.velocity_input = msg.axes[self.axis_velocity]
        # if len(msg.buttons) > self.button_mode:
        #     b = msg.buttons[self.button_mode]
        #     if b == 1 and self.btn_prev == 0 and self.mode != REALIGN: #czyli jezeli przycisk wcisniety i wystapilo zbocze narastajace i nie dokonuje sie juz jakis inny REALIGN
        #         #self.next_mode = self.type ###CZY TO ZADZIALA???? DO SPRAWDZENIA - zalezy od kolejnsoci wywolania callbackow
        #         self.mode = REALIGN

        #     self.btn_prev = b

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            for k in STEER_ORDER:
                if name == f'{k}_steer_joint':
                    self.delta_now[k] = pos

    #OBLICZA ODPOWIEDNIE KATY NA PODSTAIW DANYCH Z JOYSTICKA
    def katy(self, kappa):
        if abs(kappa) < 1e-4:
            return [0.0] * 6, [float('inf')] * 6 #dla kappa bliskiego 0, zwracamy 0 dla katow i inf dla promieni, bo nie ma ICR (ICR->infinity), jazda prosto oznacza brak skretow kol i jazda po obwodzie hipotytecznego okregu o nieskonczonym promieniu
        
        R = 1 / kappa 
        s = 1 if R >= 0 else -1
        Ra = abs(R)

        delta_in = math.atan2(self.a, Ra - self.b)
        delta_out = math.atan2(self.a, Ra + self.b)

        delta_fl, delta_fr = (delta_in, delta_out) if s > 0 else (-delta_out, -delta_in)
        angles = [delta_fl, delta_fr, 0, 0, -delta_fl, -delta_fr]

        r_in = math.sqrt((Ra - self.b)**2 + self.a**2)
        r_out = math.sqrt((Ra + self.b)**2 + self.a**2)
        r_min = Ra - self.b
        r_mout = Ra + self.b

        if s > 0:
            D = [r_in, r_out, r_min, r_mout, r_in, r_out]
        else:
            D = [r_out, r_in, r_mout, r_min, r_out, r_in]

        return angles, D


    def align(self):
        if len(self.delta_now) < 4:
            return False
        error = max(abs(self.delta_now[k] - self.delta_cmd[i]) for i, k in enumerate(STEER_ORDER))
        return error < self.deg_tol

    #OBLICZA ODPOWIEDNIE PREDKOSCI NA PODSTAWIE DANYCH Z JOYSTICKA
    def predkosci(self, kappa, v_srodka_masy, D):
        if abs(kappa) < 1e-4:
            return [v_srodka_masy / self.wheel_radius] * 6

        omega = v_srodka_masy * kappa
        v = [omega * r * (1 if kappa > 0 else -1) for r in D]
        ####kontrola przed przekroczeniem predkosci maksymalnej
        vmax = max(abs(vi) for vi in v)
        ####skalowanie predkosci, jesli przekroczona zostala predkosc maksymalna
        if vmax > self.v_max:
            v = [vi * self.v_max / vmax for vi in v]
        return [vi / self.wheel_radius for vi in v]

    def spot_katy_vel(self, omega):
        delta = math.atan2(self.a, self.b)
        r_corner = math.hypot(self.a, self.b)
        v_mid = omega * self.b
        v_corner = omega * r_corner

        v = [-v_corner, v_corner, -v_mid, v_mid, -v_corner, v_corner]
        angles = [-delta, delta, 0, 0, delta, -delta]

        vmax = max(abs(vi) for vi in v)
        if vmax > self.v_max:
            k = self.v_max / vmax
            v = [x * k for x in v]

        return angles, v

    #AKTUALIZUJE AKTUALNY STAN - KATY, PREDKOSCI, TRYB STEROWANIA
    def update(self):
        target = self.next_mode if self.mode == REALIGN else self.mode

        if target == DRIVE:
            v_body = self.velocity_input * self.v_max
            target = self.steer_input * self.kappa_max #docelowy punkt ICR czyli docelowa kappa
            step = self.kappa_rate * self.dt ##o tyle kappa moze sie zmienic w trkacie jednej iteracji, aby zmienic potencjalnie mozna zmienic kappa rate albo paramter rate(wtedy trzeba zmienic rater tez w kinematics.yaml)
            roznica = target - self.kappa #roznica miedzy aktualna wartoscia kappa a wartoscia docelowa
            ograniczenie = min(step, roznica) #roznica w cyklu nie moze byc wieksza niz step
            ograniczenie = max(-step, ograniczenie) #istotnie jedynie gdy roznica < 0, wtdy z pierwszej linijki zostaje rozncia a z tej -step
            self.kappa = self.kappa + ograniczenie

            self.delta_cmd, D = self.katy(self.kappa)
            wheels = self.predkosci(self.kappa, v_body, D)
        elif target == SPOT:
            omega = self.steer_input * self.omega_max_spot
            self.delta_cmd, wheels = self.spot_katy_vel(omega)
        else:
                angle = math.atan2(self.steer_input, self.velocity_input)     # kierunek
                mag   = min(1.0, math.hypot(self.steer_input, self.velocity_input))
                angle = max(-self.angle_max, min(self.angle_max, angle))
                if abs(angle) > math.pi / 2:          # do tylu: odwroc kolo, jedz wstecz
                    angle -= math.copysign(math.pi, angle)
                    mag = -mag
                angle = max(-self.angle_max, min(self.angle_max, angle))
                self.delta_cmd = [angle] * 6
                v = mag * self.v_max
                wheels = [v / self.wheel_radius] * 6
        if self.mode == REALIGN:
            self.delta_cmd = [0.0] * 6
            wheels = [0.0] * 6
            if self.align():
                self.mode = self.next_mode
        
        self.skret_pub.publish(Float64MultiArray(data=self.delta_cmd))
        self.predkosc_pub.publish(Float64MultiArray(data=wheels))

def main():
    rclpy.init()
    node = Kinematyka()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        