# Pololu 3pi+ 2040 Swarm Control

MicroPython firmware for the Pololu 3pi+ 2040 robot, designed to operate as part of a wireless swarm control pipeline.

## Data Pipeline

```
[Pololu 3pi+ 2040]
      UART (921600 baud)
          |
    [ESP32 Receiver]  ← onboard, bridges UART ↔ ESP-NOW
          |
        ESP-NOW
          |
    [ESP32 Dongle]    ← connected to PC via USB/serial
          |
       Serial
          |
     [swarm_hub]      ← PC-side hub, multiplexes the serial socket
      /    |    \
controller plotter terminal
```

The robot communicates with an onboard ESP32 receiver via UART. The receiver relays data wirelessly over ESP-NOW to an ESP32 dongle plugged into a PC. A `swarm_hub` process on the PC holds the serial socket and fans out the data stream to multiple tools: a controller (sends commands), a plotter (visualizes telemetry), and a terminal (raw serial monitor).

## Files

| File | Description |
|------|-------------|
| `robot_uart.py` | UART protocol library — framing, CRC-8, packet parsing, debug screen registration |
| `screen_manager.py` | Display manager — multiple screens, line graphs, bar graphs, gauges, metrics, primitives |
| `uart_controller.py` | Main application — receives `MSG_SPEED` packets and drives the motors |

## Packet Format

```
[0xAA][0x55][type][len][payload...][crc8]
```

CRC-8 covers `type + len + payload` (polynomial 0x07).

### Message Types

| ID | Name | Direction | Payload |
|----|------|-----------|---------|
| `0x01` | `MSG_SPEED` | ESP32 → Robot | `[left: int8][right: int8]` (-127..+127) |
| `0x02` | `MSG_DEBUG` | — | — |
| `0x03` | `MSG_METRICS` | — | — |
| `0x04` | `MSG_ROBOT_ID` | ESP32 → Robot | `[id: uint8]` |
| `0x20` | `MSG_PING` | bidirectional | empty |
| `0x22` | `MSG_DEBUG_REG` | ESP32 → Robot | `[field_id][display_type][x][y][label...]` |
| `0x23` | `MSG_DEBUG_DATA` | ESP32 → Robot | `[field_id][value_type][data...]` |

### Debug Display Types

| Value | Type |
|-------|------|
| `0x01` | Metric (fixed position label + value) |
| `0x02` | Line graph (scrolling) |
| `0x03` | Log entry |

## Display

The `ScreenManager` manages multiple `Screen` pages, cycled with button B on the robot.

**Default screens in `uart_controller.py`:**
- **MOTORS** — shows current left/right motor speeds and robot ID (2x scaled)
- **GRAPH** — split line graphs for left and right motor speeds

## Sound

Sound comes entirely from `pololu_3pi_2040_robot.buzzer.Buzzer` — a piezo buzzer wired to GP7, driven directly by one of the RP2040's hardware PWM channels (`machine.PWM`). The drive motors also run on PWM (20833 Hz, see `motors.py`), but that's at/above the edge of human hearing and isn't used as a sound source; the buzzer is the only intentional speaker on the board.

**Two ways to make a tone:**
- **Direct PWM** — `buzzer.pwm.freq(hz)` / `buzzer.pwm.duty_u16(level)` set the square-wave frequency and loudness immediately. `Buzzer.beep()`, `.on()`/`.off()`, and effects like a sweeping siren (recomputing frequency every loop iteration) use this path.
- **Music macro language** — `Buzzer.play(str)` / `.play_in_background(str)` parse an RTTTL-like text mini-language (notes `a`-`g`, rest `r`, octave `O`/`>`/`<`, tempo `T`, volume `V`, duration `L`, staccato `MS`, accidentals `+`/`#`/`-`) into parallel `frequencies`/`durations`/`volumes`/`notes`/`beats` arrays.

Each note is encoded as `octave*12 + offset` (C=0 ... B=11) and converted to Hz with the standard equal-temperament formula referenced to A4 = 440 Hz:

```
freq = round(440 * 2 ** ((note - 57) / 12))
```

`play_in_background()` doesn't block: it precomputes the whole note sequence, then arms a `machine.Timer` one-shot. The timer callback sets the PWM frequency/duty for the current note and re-arms itself for that note's duration, stepping through the sequence on hardware-timer interrupts — so a tune keeps playing while the main loop does other work (e.g. `music.py` drives an RGB hue/brightness animation per note via `Buzzer.set_callback()`). `play()` is the same thing plus a busy-wait until playback finishes, so it behaves like a blocking call. Volume is just duty cycle (`V0`-`V15` → `volume_levels[v] * 256`, max ≈ half-scale) — there's no separate amplitude control.

**In this firmware:** `uart_controller.py`, `robot_uart.py`, and `screen_manager.py` never touch the buzzer — the swarm protocol is silent by design. The only sound played in this project comes from the stock Pololu `main.py`: a startup chime + button-press beeps from the splash screen, and a single low tone (`buzzer.play("O2c4")`) if the deployed program raises an uncaught exception. To add audible feedback for swarm events (low battery, lost packets, robot ID confirmation), instantiate `robot.Buzzer()` once and trigger short `play_in_background("...")` calls from the packet/event handlers — it won't block `proto.loop()`.

## Usage

Deploy `robot_uart.py`, `screen_manager.py`, and `uart_controller.py` to the Pololu 3pi+ 2040 via MicroPython. `uart_controller.py` is the entry point (`main.py`).

```python
# uart_controller.py wires everything together:
proto = UARTProtocol(uart, screen_manager=mgr, on_packet=on_packet, on_robot_id=on_robot_id)

while True:
    proto.loop()       # read + parse UART
    process_pending()  # apply latest speed packet
    mgr.loop()         # update display
```

## Dependencies

- MicroPython on Pololu 3pi+ 2040
- `pololu_3pi_2040_robot` MicroPython library (bundled with the robot)
