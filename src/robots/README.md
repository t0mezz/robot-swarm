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
| `0x02` | `MSG_DEBUG` | Robot → PC | `[field_id][value_type][data...]` (robot→ESP32; ESP32 prepends `robot_id`, so the PC sees `[robot_id][field_id][value_type][data...]`) |
| `0x03` | `MSG_METRICS` | Robot → ESP32 | `[battery: uint8]` (0-255 → 0-5V; ESP32 forwards it as-is into `MSG_TELEMETRY`'s battery byte) |
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

## Usage

Deploy `robot_uart.py`, `screen_manager.py`, and `uart_controller.py` to the Pololu 3pi+ 2040 via MicroPython. `uart_controller.py` is the entry point (`main.py`).

### Batch flashing

`flash.py` deploys every `*.py` file in this directory (except `main.py`, Pololu's stock splash loader) over USB via `mpremote`, then soft-resets the board so the new code runs immediately:

```bash
pip install mpremote
python3 src/robots/flash.py
```

It watches for a robot's serial port to appear, flashes it, waits for it to be unplugged, and repeats — so you can plug robots in one after another without re-running anything. Not yet verified against real hardware.

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
- `mpremote` (PC-side, only for `flash.py`) — `pip install mpremote`
