# uart_controller.py
# Receives MSG_SPEED packets from ESP32 via UART and controls the motors
# using closed-loop PID on wheel encoder speed (counts/s).
#
# The two signed bytes in MSG_SPEED now represent target rotational wheel
# speeds scaled to [-127..+127] → [-MAX_WHEEL_CPS..+MAX_WHEEL_CPS].
#
# Optimisations retained:
#   - UART RX buffer enlarged to 256 bytes
#   - Skip-to-latest: only the most recent speed packet is applied per loop
#   - Display update fully decoupled from packet receive

# ─── Feature toggles ───────────────────────────────────────────
ENGINE_SOUND_ENABLED = True   # False: skip importing/loading engine_sound entirely

from machine import UART, Pin
from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.battery import Battery
from robot_uart import UARTProtocol, MSG_SPEED
from screen_manager import ScreenManager, Screen, LineGraph
import framebuf
import struct
import time

# ─── Odometry / PID tuning ────────────────────────────────────
# Closed-loop (encoder+PID) vs. open-loop (target mapped straight to power).
# Flip to True to re-enable closed-loop control — the PID objects and
# run_pid()'s measurement code stay intact either way.
ODOMETRY_ENABLED = False

if ENGINE_SOUND_ENABLED:
    from pololu_3pi_2040_robot.buzzer import Buzzer
    from engine_sound import EngineSound

# ─── Hardware ─────────────────────────────────────────────────
uart     = UART(0, baudrate=921600, bits=8, parity=None, stop=1,
                tx=Pin(28), rx=Pin(29), rxbuf=256)
motors   = robot.Motors()
encoders = robot.Encoders()
display  = robot.Display()
buttonB  = robot.ButtonB()
battery  = Battery()
buzzer   = Buzzer() if ENGINE_SOUND_ENABLED else None

# Encoder counts per second at full commanded speed.
# Tune MAX_WHEEL_CPS to match the physical top speed of the robot.
MAX_WHEEL_CPS = 5000      # counts/s  ↔  signed-byte value 127

# PI gains  –  tune Kp first (Ki=0), then add Ki to remove steady-state offset.
# Kd is intentionally omitted: differentiating a speed error yields jerk,
# which is dominated by encoder noise and causes oscillation on this plant.
PID_KP = 2.5
PID_KI = 0.1

# Anti-windup: integral clamped to ±this value (in counts/s units)
PID_I_MAX = 250.0

# Motor power output range (pololu_3pi_2040_robot uses ±6000)
MOTOR_MAX = 6000

# PID update interval
PID_INTERVAL_MS = 20      # 50 Hz control loop

# Battery sampling interval
BAT_INTERVAL_MS = 2000    # read voltage every 2 s


# ─── PID Controller ───────────────────────────────────────────

class PI:
    """
    Discrete PI controller with anti-windup integral clamp.

    Target and measurement must use the same unit (here: encoder counts/s).
    Output is dimensionless motor power in [-MOTOR_MAX..+MOTOR_MAX].
    """
    def __init__(self, kp: float, ki: float, i_max: float, out_max: int):
        self.kp      = kp
        self.ki      = ki
        self.i_max   = i_max
        self.out_max = out_max
        self._integral = 0.0

    def reset(self):
        self._integral = 0.0

    def update(self, target: float, measured: float, dt: float) -> int:
        """Returns clamped motor power for this timestep."""
        error = target - measured

        self._integral += error * dt
        if self._integral >  self.i_max: self._integral =  self.i_max
        if self._integral < -self.i_max: self._integral = -self.i_max

        output = self.kp * error + self.ki * self._integral
        if output >  self.out_max: output =  self.out_max
        if output < -self.out_max: output = -self.out_max
        return int(output)


pid_left  = PI(PID_KP, PID_KI, PID_I_MAX, MOTOR_MAX)
pid_right = PI(PID_KP, PID_KI, PID_I_MAX, MOTOR_MAX)

engine = EngineSound(buzzer, MAX_WHEEL_CPS) if ENGINE_SOUND_ENABLED else None

# ─── Shared state ─────────────────────────────────────────────
_target_left_cps  = [0.0]   # target wheel speed, counts/s
_target_right_cps = [0.0]

_actual_left_cps  = [0.0]   # last measured wheel speed, counts/s
_actual_right_cps = [0.0]

_motor_left_pwr   = [0]     # last motor power output
_motor_right_pwr  = [0]

_robot_id = [None]
_bat_mv   = [0]

# Encoder state for speed measurement
_enc_left_prev  = [0]
_enc_right_prev = [0]
_enc_time_prev  = [time.ticks_ms()]


# ─── Display helpers ──────────────────────────────────────────

def _draw_scaled_text(display, text, x, y, scale=2):
    """Render text at (x, y) with pixel-doubled scaling using a temp framebuf."""
    n = len(text)
    w = n * 8
    row_bytes = (w + 7) // 8
    buf = bytearray(row_bytes * 8)
    fb = framebuf.FrameBuffer(buf, w, 8, framebuf.MONO_HLSB)
    fb.fill(0)
    fb.text(text, 0, 0, 1)
    for row in range(8):
        for col in range(w):
            if fb.pixel(col, row):
                display.fill_rect(x + col * scale, y + row * scale, scale, scale, 1)


# ─── Screen setup ─────────────────────────────────────────────
mgr           = ScreenManager(display=display, button=buttonB)
status_screen = Screen("MOTORS")
mgr.register(status_screen)

# Show target and actual CPS side-by-side
status_screen.set_metric("tL",  lambda: f"{int(_target_left_cps[0]):5d}",  x=0, y=14)
status_screen.set_metric("aL",  lambda: f"{int(_actual_left_cps[0]):5d}",  x=0, y=24)
status_screen.set_metric("tR",  lambda: f"{int(_target_right_cps[0]):5d}", x=0, y=34)
status_screen.set_metric("aR",  lambda: f"{int(_actual_right_cps[0]):5d}", x=0, y=44)
status_screen._add_primitive(
    lambda d: d.text(f"{_bat_mv[0]/1000:.2f}V", 52, 0)
)

status_screen._add_primitive(
    lambda d: _draw_scaled_text(
        d, '?' if _robot_id[0] is None else f"{_robot_id[0]:2d}", 80, 38, scale=3)
)

graph_screen = Screen("GRAPH")
mgr.register(graph_screen)

left_graph  = LineGraph(label="L", auto_scale=True)
right_graph = LineGraph(label="R", auto_scale=True)
graph_screen.set_diagram(left_graph,  height=22)
graph_screen.add_diagram(right_graph, height=22)

_GRAPH_INTERVAL_MS = 200
_last_graph_update = 0


# ─── PID control loop ─────────────────────────────────────────
_last_pid_time = time.ticks_ms()
_last_bat_time = time.ticks_ms()

def run_pid():
    """Measure wheel speeds and run PID; called at PID_INTERVAL_MS cadence."""
    global _last_pid_time, _last_graph_update, _last_bat_time

    now = time.ticks_ms()
    dt_ms = time.ticks_diff(now, _last_pid_time)
    if dt_ms < PID_INTERVAL_MS:
        return
    _last_pid_time = now
    dt = dt_ms / 1000.0         # seconds

    # ── Measure actual wheel speed from encoders ──────────────
    enc_l = encoders.get_counts()[0]
    enc_r = encoders.get_counts()[1]

    delta_l = enc_l - _enc_left_prev[0]
    delta_r = enc_r - _enc_right_prev[0]
    _enc_left_prev[0]  = enc_l
    _enc_right_prev[0] = enc_r

    actual_l = delta_l / dt
    actual_r = delta_r / dt
    _actual_left_cps[0]  = actual_l
    _actual_right_cps[0] = actual_r

    if engine is not None:
        engine.update(actual_l, actual_r, dt)

    # ── PID update ────────────────────────────────────────────
    target_l = _target_left_cps[0]
    target_r = _target_right_cps[0]

    if ODOMETRY_ENABLED:
        # Reset integrators when target is zero to prevent wind-up while stopped
        if target_l == 0.0:
            pid_left.reset()
        if target_r == 0.0:
            pid_right.reset()

        pwr_l = pid_left.update(target_l,  actual_l,  dt)
        pwr_r = pid_right.update(target_r, actual_r, dt)
    else:
        # Open-loop: map target counts/s straight to motor power, no encoder feedback.
        pid_left.reset()
        pid_right.reset()
        pwr_l = int(max(-MOTOR_MAX, min(MOTOR_MAX, target_l / MAX_WHEEL_CPS * MOTOR_MAX)))
        pwr_r = int(max(-MOTOR_MAX, min(MOTOR_MAX, target_r / MAX_WHEEL_CPS * MOTOR_MAX)))

    motors.set_speeds(pwr_l, pwr_r)
    _motor_left_pwr[0]  = pwr_l
    _motor_right_pwr[0] = pwr_r

    if time.ticks_diff(now, _last_bat_time) >= BAT_INTERVAL_MS:
        _bat_mv[0] = battery.get_level_millivolts()
        _last_bat_time = now

    mgr.mark_dirty()

    if mgr.active_screen is graph_screen:
        if time.ticks_diff(now, _last_graph_update) >= _GRAPH_INTERVAL_MS:
            left_graph.push(actual_l)
            right_graph.push(actual_r)
            _last_graph_update = now


# ─── Packet handling ──────────────────────────────────────────
_pending_speed = None

def on_packet(packet):
    """Store the latest speed packet; applied on next loop iteration."""
    global _pending_speed
    if packet.msg_type != MSG_SPEED:
        return
    if len(packet.payload) < 2:
        return
    left_byte, right_byte = struct.unpack('bb', packet.payload[:2])
    _pending_speed = (left_byte, right_byte)

def process_pending():
    """Convert the pending signed bytes to target wheel speeds (counts/s)."""
    global _pending_speed
    if _pending_speed is None:
        return
    left_byte, right_byte = _pending_speed
    _pending_speed = None

    _target_left_cps[0]  = left_byte  / 127.0 * MAX_WHEEL_CPS
    _target_right_cps[0] = right_byte / 127.0 * MAX_WHEEL_CPS

def on_robot_id(robot_id):
    _robot_id[0] = robot_id
    mgr.mark_dirty()


# ─── Protocol ─────────────────────────────────────────────────
proto = UARTProtocol(uart, screen_manager=mgr, on_packet=on_packet, on_robot_id=on_robot_id)

# ─── Main loop ────────────────────────────────────────────────
motors.set_speeds(0, 0)

# Seed encoder baseline
_enc_left_prev[0], _enc_right_prev[0] = encoders.get_counts()

while True:
    # 1. Read and parse incoming UART bytes
    proto.loop()

    # 2. Update target speeds from the latest received packet
    process_pending()

    # 3. Run PID control loop (rate-limited internally)
    run_pid()

    # 4. Update display (rate-limited by ScreenManager)
    mgr.loop()
