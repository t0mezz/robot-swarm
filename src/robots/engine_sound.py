# engine_sound.py
# Continuous engine-like tone driven by wheel speed, via the buzzer's direct
# PWM path (see README "Sound" section) — not the RTTTL music macro language,
# since that's note-based and can't track a continuously varying speed.
#
# Self-contained: callers only ever call update() once per loop. No UART/
# protocol involvement — it's a pure function of the speeds the caller
# already has on hand.

class EngineSound:
    """
    Idle hum that rises in pitch/volume with wheel speed, with a turn-wobble
    layered on top when the wheels are spinning at noticeably different rates.

    Usage:
        engine = EngineSound(buzzer, max_cps=MAX_WHEEL_CPS)
        # In the loop, with the wheel speeds you already measured/commanded:
        engine.update(left_cps, right_cps, dt)
    """

    def __init__(self, buzzer, max_cps: float,
                 idle_freq: int = 55, max_freq: int = 480,
                 idle_duty: int = 300, max_duty: int = 16000,
                 turn_threshold: float = 80.0, wobble_hz: float = 14.0,
                 wobble_depth: int = 6, smoothing: float = 0.25):
        self._pwm            = buzzer.pwm
        self._max_cps        = max_cps
        self._idle_freq      = idle_freq
        self._freq_span      = max_freq - idle_freq
        self._idle_duty      = idle_duty
        self._duty_span      = max_duty - idle_duty
        self._turn_threshold = turn_threshold
        self._wobble_hz      = wobble_hz
        self._wobble_depth   = wobble_depth
        self._alpha          = smoothing   # EMA factor: higher = faster response, more raw

        self._avg_speed = 0.0
        self._wobble_t  = 0.0

    def update(self, left_cps: float, right_cps: float, dt: float):
        """Recompute and apply the tone. dt in seconds, same dt as the caller's own loop."""
        speed = (abs(left_cps) + abs(right_cps)) * 0.5
        turn  = abs(left_cps - right_cps)

        # Smoothed so encoder/PID noise doesn't make the tone crackle.
        self._avg_speed += (speed - self._avg_speed) * self._alpha

        ratio = self._avg_speed / self._max_cps
        if ratio > 1.0:
            ratio = 1.0

        freq = self._idle_freq + ratio * self._freq_span

        if turn > self._turn_threshold:
            self._wobble_t += dt
            period = 1.0 / self._wobble_hz
            phase  = self._wobble_t % period
            freq  += self._wobble_depth if phase < period / 2 else -self._wobble_depth
        else:
            self._wobble_t = 0.0

        duty = int(self._idle_duty + ratio * self._duty_span)

        self._pwm.freq(int(freq))
        self._pwm.duty_u16(duty)

    def stop(self):
        """Silence the buzzer (idle hum off)."""
        self._pwm.duty_u16(0)
