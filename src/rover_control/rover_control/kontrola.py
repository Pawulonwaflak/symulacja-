import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy, JointState

# Kolejność kół: ['rl', 'rr', 'ml', 'mr', 'fl', 'fr']
FLIP = [-1, 1, -1, 1, -1, 1]
STEER_SIGN = -1

DRIVE, REALIGN, SPOT, CRAB = 'DRIVE', 'REALIGN', 'SPOT', 'CRAB'


class Kinematyka(Node):
    """Wszystkie tryby liczone z jednego wzoru bryły sztywnej:  v_i = v + omega x r_i

        DRIVE : vy = 0,        omega = vx * steer / R_min      (Ackermann)
        SPOT  : vx = vy = 0,   omega z prawej gałki
        CRAB  : omega = 0,     (vx, vy) = kierunek lewej gałki

    Gaz ze znakiem: R2 = do przodu, L2 = do tyłu (throttle = R2 - L2).
    W CRAB gałka wybiera OŚ ruchu, a triggery zwrot po tej osi.

    Kąty liczone są z KIERUNKU zadania (twist jednostkowy), a prędkość skalowana
    osobno gazem - dzięki temu koła ustawiają się też przy stojącym robocie.
    """

    def __init__(self):
        super().__init__('kinematyka')

        ###################### PARAMETRY #########################
        self.a = self.declare_parameter('a', 0.198).value
        self.b = self.declare_parameter('b', 0.124).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.063).value
        self.rate = self.declare_parameter('rate', 50.0).value
        self.deg_tol = math.radians(self.declare_parameter('realign_tol_deg', 3.0).value)

        self.v_max = self.declare_parameter('v_max', 1.0).value                  # prędkość robota [m/s]
        self.wheel_v_max = self.declare_parameter('wheel_v_max', 1.0).value      # limit prędkości LINIOWEJ koła [m/s]
        self.omega_max_spot = self.declare_parameter('omega_max_spot', 0.6).value
        self.ackermann_max_deg = self.declare_parameter('ackermann_max_deg', 40.0).value
        self.deadzone = self.declare_parameter('deadzone', 0.10).value

        # Skręt kół: twardy limit stawu i zapas na histerezę flipu (kąt <-> kąt+-180 st. z odwróconą prędkością)
        self.steer_limit = math.radians(self.declare_parameter('steer_limit_deg', 90.0).value)
        self.flip_hyst = math.radians(self.declare_parameter('flip_hyst_deg', 10.0).value)

        # Limitery (ochrona prądowa)
        self.steer_rate = math.radians(self.declare_parameter('steer_rate_deg', 90.0).value)  # [rad/s]
        self.accel_max = self.declare_parameter('accel_max', 1).value   # przyspieszenie koła [m/s^2]
        self.decel_max = self.declare_parameter('decel_max', 4.0).value   # hamowanie koła [m/s^2]
        # Gaz maleje liniowo do 0, gdy koła nie są jeszcze ustawione (błąd skrętu między start a stop)
        self.gate_start = math.radians(self.declare_parameter('gate_start_deg', 5.0).value)
        self.gate_stop = math.radians(self.declare_parameter('gate_stop_deg', 20.0).value)

        self.crab_stick_min = self.declare_parameter('crab_stick_min', 0.3).value
        self.r2_rest = self.declare_parameter('r2_rest', 1.0).value      # wartość osi R2 w spoczynku
        self.l2_rest = self.declare_parameter('l2_rest', 1.0).value      # wartość osi L2 w spoczynku
        self.trigger_deadzone = self.declare_parameter('trigger_deadzone', 0.03).value
        self.joy_timeout = self.declare_parameter('joy_timeout', 0.5).value

        # Osie PS4 (sprawdź: ros2 topic echo /joy)
        self.axis_left_x = 0
        self.axis_left_y = 1
        self.axis_right_x = 3
        self.axis_l2 = 2
        self.axis_r2 = 5

        self.btn_drive = 0      # Krzyżyk (X)  -> DRIVE
        self.btn_spot = 1       # Kółko (O)    -> SPOT
        self.btn_crab = 3       # Kwadrat      -> CRAB
        self.btn_deadman = 4    # L1           -> DEADMAN_SWITCH

        self.R_min = self.b + self.a / math.tan(math.radians(self.ackermann_max_deg))

        self.wheel_pos = [(-self.a, self.b), (-self.a, -self.b), (0.0, self.b),
                          (0.0, -self.b), (self.a, self.b), (self.a, -self.b)]

        self.steer_joints = [
            'rl_steer_joint', 'rr_steer_joint', 'ml_steer_joint',
            'mr_steer_joint', 'fl_steer_joint', 'fr_steer_joint'
        ]

        ##################### STAN #########################
        self.mode = DRIVE
        self.next_mode = DRIVE
        self.prev_buttons = []

        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.throttle = 0.0      
        self.armed = {'r2': False, 'l2': False}
        self.deadman = False
        self.crab_phi = 0.0       # ostatni kierunek CRAB [rad], pełny okrąg

        self.last_joy = None
        self.last_tick = None
        self.have_js = False

        self.tgt_raw = [0.0] * 6      # ostatni cel kątów (konwencja fizyczna, bez STEER_SIGN) - pamięć histerezy
        self.angles = [0.0] * 6       # cel kątów stawów (po STEER_SIGN)
        self.speeds = [0.0] * 6       # cel prędkości kół [rad/s] (po FLIP)
        self.cmd_angles = [0.0] * 6   # to, co idzie na topic (po limiterze)
        self.cmd_speeds = [0.0] * 6
        self.steer_pos = [0.0] * 6

        self.ps = self.create_publisher(Float64MultiArray, '/steer_position_controller/commands', 10)
        self.pw = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_callback, 10)

        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            'Uruchomiono. DRIVE: L1 czuwak, R2 gaz, L2 wstecz, lewa gałka X skręt | '
            'SPOT: prawa gałka X obrót | CRAB: lewa gałka oś ruchu, R2 przód / L2 tył')

    ##################### JOY #########################
    def apply_deadzone(self, val):
        if abs(val) < self.deadzone:
            return 0.0
        return (val - math.copysign(self.deadzone, val)) / (1.0 - self.deadzone)

    def axis(self, msg, idx):
        return msg.axes[idx] if idx < len(msg.axes) else 0.0

    def is_rising_edge(self, buttons, idx):
        if len(buttons) <= idx or len(self.prev_buttons) <= idx:
            return False
        return buttons[idx] == 1 and self.prev_buttons[idx] == 0

    def read_trigger(self, msg, idx, rest, key):
        """Trigger -> 0..1 (0 = puszczony, 1 = wciśnięty do końca)."""
        if idx >= len(msg.axes):
            return 0.0
        raw = msg.axes[idx]
        if not self.armed[key]:
            # część sterowników daje 0.0 do pierwszego ruchu triggera
            if abs(raw - rest) < 0.1:
                self.armed[key] = True
            else:
                return 0.0
        t = max(0.0, min(1.0, abs(raw - rest) / 2.0))
        return 0.0 if t < self.trigger_deadzone else t

    def joy_callback(self, msg: Joy):
        self.last_joy = self.get_clock().now()

        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)

        # 1. Zmiana trybu (X, O, Kwadrat)
        if self.mode != REALIGN:
            req = self.mode
            if self.is_rising_edge(msg.buttons, self.btn_drive):
                req = DRIVE
            elif self.is_rising_edge(msg.buttons, self.btn_spot):
                req = SPOT
            elif self.is_rising_edge(msg.buttons, self.btn_crab):
                req = CRAB

            if req != self.mode:
                self.next_mode = req
                self.mode = REALIGN
                self.get_logger().info(f'Przełączanie: REALIGN -> {self.next_mode}')

        self.prev_buttons = list(msg.buttons)

        # 2. R2 - L2 (czytamy zawsze, żeby się "uzbroiły")
        throttle = (self.read_trigger(msg, self.axis_r2, self.r2_rest, 'r2')
                    - self.read_trigger(msg, self.axis_l2, self.l2_rest, 'l2'))

        # 3. Czuwak (L1)
        self.deadman = len(msg.buttons) > self.btn_deadman and msg.buttons[self.btn_deadman] == 1
        if not self.deadman:
            self.lx = self.ly = self.rx = self.throttle = 0.0
            return

        if not all(self.armed.values()):
            waiting = ' i '.join(k.upper() for k, v in self.armed.items() if not v)
            self.get_logger().warn(f'{waiting} jeszcze nieuzbrojone - wciśnij i puść raz',
                                   throttle_duration_sec=3.0)

        # 4. Gałki
        self.lx = self.apply_deadzone(self.axis(msg, self.axis_left_x))
        self.ly = self.apply_deadzone(self.axis(msg, self.axis_left_y))
        self.rx = self.apply_deadzone(self.axis(msg, self.axis_right_x))
        self.throttle = throttle

    def js_callback(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        self.steer_pos = [pos.get(j, 0.0) for j in self.steer_joints]
        if not self.have_js:
            # start od faktycznych kątów kół, żeby nie było skoku zadania
            self.cmd_angles = list(self.steer_pos)
            self.tgt_raw = [p * STEER_SIGN for p in self.steer_pos]
        self.have_js = True

    ##################### KINEMATYKA #########################
    def swerve(self, vx, vy, w, scale):
        band = self.steer_limit + self.flip_hyst
        raw, lin = [], []
        for i, (x, y) in enumerate(self.wheel_pos):
            ux = vx - w * y
            uy = vy + w * x
            v = math.hypot(ux, uy)
            ang = math.atan2(uy, ux) if v > 1e-6 else self.tgt_raw[i]   # v~0 -> trzymaj kąt

            cands = [(ang, v), (ang - math.copysign(math.pi, ang), -v)]
            ok = [c for c in cands if abs(c[0]) <= band]
            if not ok:
                ok = [min(cands, key=lambda c: abs(c[0]))]
            ang, v = min(ok, key=lambda c: abs(c[0] - self.tgt_raw[i]))

            self.tgt_raw[i] = ang
            raw.append(max(-self.steer_limit, min(self.steer_limit, ang)))
            lin.append(v * scale)
        return raw, lin

    def plan(self, target, realign):
        """Tryb + gałki -> (vx, vy, w, scale). scale ze znakiem (R2 - L2). W REALIGN: poza docelowa, scale = 0."""
        if target == DRIVE:
            if realign:
                return 1.0, 0.0, 0.0, 0.0
            # scale ze znakiem: L2 (scale < 0) jedzie po tym samym łuku do tyłu
            return 1.0, 0.0, self.lx / self.R_min, self.throttle * self.v_max

        if target == SPOT:
            s = 1.0 if self.rx >= 0.0 else -1.0     # ta sama linia kół w obie strony, zmienia się tylko znak prędkości
            return 0.0, 0.0, s, 0.0 if realign else abs(self.rx) * self.omega_max_spot

        # CRAB
        if realign:
            self.crab_phi = 0.0
            return 1.0, 0.0, 0.0, 0.0
        if math.hypot(self.ly, self.lx) > self.crab_stick_min:
            self.crab_phi = math.atan2(self.lx, self.ly)    # vx = ly (przód), vy = lx (lewo)
        return math.cos(self.crab_phi), math.sin(self.crab_phi), 0.0, self.throttle * self.v_max

    ##################### LIMITERY #########################
    def max_steer_error(self):
        return max(abs(self.angles[i] - self.steer_pos[i]) for i in range(6))

    def is_aligned(self):
        return self.max_steer_error() < self.deg_tol

    def steer_gate(self):
        """1.0 = koła ustawione, 0.0 = daleko od celu (nie jedziemy, żeby nie szorować)."""
        if not self.have_js:
            return 1.0
        err = self.max_steer_error()
        if err <= self.gate_start:
            return 1.0
        if err >= self.gate_stop:
            return 0.0
        return (self.gate_stop - err) / (self.gate_stop - self.gate_start)

    def ramp_angles(self, dt):
        step = self.steer_rate * dt
        self.cmd_angles = [c + max(-step, min(step, t - c)) for c, t in zip(self.cmd_angles, self.angles)]

    def ramp_speeds(self, dt):
        out = []
        for c, t in zip(self.cmd_speeds, self.speeds):
            lim = (self.decel_max if abs(t) < abs(c) else self.accel_max) / self.wheel_radius * dt
            out.append(c + max(-lim, min(lim, t - c)))
        self.cmd_speeds = out

    def check_joy_timeout(self):
        if self.last_joy is None:
            return
        dt = (self.get_clock().now() - self.last_joy).nanoseconds * 1e-9
        if dt > self.joy_timeout:
            self.lx = self.ly = self.rx = self.throttle = 0.0
            self.deadman = False

    ##################### PĘTLA #########################
    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9 if self.last_tick is not None else 1.0 / self.rate
        self.last_tick = now
        dt = max(0.0, min(dt, 0.1))

        self.check_joy_timeout()

        realign = self.mode == REALIGN
        target = self.next_mode if realign else self.mode

        # 1. Zadanie -> kąty i prędkości kół z jednego wzoru
        vx, vy, w, scale = self.plan(target, realign)
        raw, lin = self.swerve(vx, vy, w, scale)
        self.angles = [a * STEER_SIGN for a in raw]

        # 2. Limit prędkości liniowej koła (skaluje wszystkie koła razem, kierunek jazdy się nie zmienia)
        m = max(abs(v) for v in lin)
        if m > self.wheel_v_max:
            lin = [v * self.wheel_v_max / m for v in lin]

        # 3. Blokada gazu, dopóki koła nie dojadą do kątów
        gate = self.steer_gate()
        if gate < 1.0 and m > 1e-6:
            self.get_logger().info(
                f'Gaz ograniczony ({gate:.0%}): błąd skrętu {math.degrees(self.max_steer_error()):.1f} st.',
                throttle_duration_sec=2.0)
        self.speeds = [(lin[i] * gate / self.wheel_radius) * FLIP[i] for i in range(6)]

        # 4. Limitery: kąty zawsze z ograniczoną szybkością, prędkości z rampą
        #    (brak czuwaka lub REALIGN = natychmiastowy stop kół)
        self.ramp_angles(dt)
        if realign or not self.deadman:
            self.cmd_speeds = [0.0] * 6
        else:
            self.ramp_speeds(dt)

        # 5. REALIGN: czekamy aż skręty dojadą do kątów docelowych
        if realign:
            if not self.have_js:
                self.get_logger().warn('REALIGN: brak /joint_states - nie wiem, czy koła są ustawione',
                                       throttle_duration_sec=2.0)
            elif self.is_aligned():
                self.mode = self.next_mode
                self.get_logger().info(f'Zakończono REALIGN. Aktywny tryb: {self.mode}')
            else:
                self.get_logger().info(
                    f'REALIGN: max błąd skrętu {math.degrees(self.max_steer_error()):.1f} st.',
                    throttle_duration_sec=1.0)

        self.ps.publish(Float64MultiArray(data=[float(x) for x in self.cmd_angles]))
        self.pw.publish(Float64MultiArray(data=[float(x) for x in self.cmd_speeds]))

    def stop(self):
        self.pw.publish(Float64MultiArray(data=[0.0] * 6))
        self.ps.publish(Float64MultiArray(data=[0.0] * 6))


def main(args=None):
    rclpy.init(args=args)
    node = Kinematyka()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()