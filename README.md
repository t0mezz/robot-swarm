# Robot Swarm

ESP-NOW basiertes Schwarm-Steuerungssystem für bis zu min. 20 Pololu 3pi+ 2040 Roboter. (MAX_ROBOTS)



---

## Architektur

```
Controller-PC  →  USB-Serial  →  Dongle-ESP32  →  ESP-NOW Broadcast  →  20× Robot-ESP32  →  UART  →  RP2040
                  921600 Baud                        ~1ms Latenz                              921600 Baud
```

Gesamtlatenz: ~4ms vom Tastendruck bis Motorreaktion.

## Projektstruktur

```
robot-swarm/
├── platformio.ini              # Build-Konfiguration
├── lib/
│   └── SwarmProtocol/          # Gemeinsame Header
│       ├── protocol.h          # Nachrichtentypen, CRC, Frame-Builder
│       ├── hardware.h          # Pin-Definitionen, Timing
│       └── debug_protocol.h    # RP2040 Display-Kommunikation
├── src/
│   ├── receiver/
│   │   └── main.cpp            # Robot-ESP32 Firmware
│   └── dongle/
│       └── main.cpp            # Dongle-ESP32 Firmware
├── tools/
│   ├── swarm_terminal.cpp      # Terminal-Monitor (PC)
│   └── Makefile
└── docs/
    └── architecture.md         # Pipeline-Dokumentation
```

## Voraussetzungen

- [PlatformIO](https://platformio.org/) (CLI oder VS Code Extension)

## Quick Start

### 1. Dongle flashen

```bash
pio run -e dongle -t upload
```

### 2. Receiver flashen

Robot-ID beim Flashen per Umgebungsvariable setzen (Standard: 0):

```bash
# Robot 0 (Standard)
pio run -e receiver -t upload

# Robot 3
ROBOT_ID=3 pio run -e receiver -t upload

# Robot 7
ROBOT_ID=7 pio run -e receiver -t upload
```

Die ID wird als `ROBOT_ID`-Makro in die Firmware eingebaut und ist nach dem Flashen fest verdrahtet.

### 3. Terminal-Monitor starten

```bash
cd tools
make
./swarm_terminal /dev/tty.usbmodem* 50
```

Zeigt alle registrierten Roboter mit RSSI, Latenz und Status.

## Protokoll

| Typ | Code | Richtung | Payload |
|-----|------|----------|---------|
| SWARM | 0x10 | PC → Broadcast | [id\|L\|R] × 20 = 60 Bytes |
| ANNOUNCE | 0x20 | Robot → Broadcast | [id, MAC × 6] |
| ANNOUNCE_ACK | 0x21 | Dongle → Broadcast | [id] |
| PING | 0x22 | PC → Robot | [target_id, timestamp × 4] |
| PONG | 0x23 | Robot → Dongle | [id, echo_timestamp × 4] |
| TELEMETRY | 0x30 | Robot → Dongle | [id, rssi, bat, flags, mL, mR, uptime × 2] |
| SPEED | 0x01 | ESP32 → RP2040 | [left, right] |

Alle Frames: `[0xAA][0x55][type][len][payload...][CRC-8]`

## Features

- **Automatische Registrierung**: Roboter meldet sich per ANNOUNCE, Dongle bestätigt mit ACK
- **TDMA-Telemetrie**: Jeder Robot hat einen 50ms Zeitslot im 1s-Zyklus
- **Ping/Latenz**: Round-Robin Ping mit µs-Auflösung
- **Motor-Watchdog**: Stoppt nach 300ms ohne Paket
- **Debug-Display**: RSSI, Latenz, Status live auf dem RP2040 OLED
- **Transport-Abstraktion**: Receiver vorbereitet für Wechsel auf WiFi STA

---

## Summary

ESP-NOW based swarm control system for up to 20 Pololu 3pi+ 2040 robots. A controller PC sends commands over USB serial to a dongle ESP32, which broadcasts them via ESP-NOW to all robot ESP32s. Each robot ESP32 forwards motor commands to the RP2040 over UART. End-to-end latency is ~4ms from keypress to motor response.

**Key features:**
- Automatic robot registration via ANNOUNCE/ACK handshake
- TDMA telemetry: each robot gets a 50ms slot in a 1s cycle
- Round-robin ping with µs-resolution latency measurement
- Motor watchdog: stops motors after 300ms without a packet
- Live debug display (RSSI, latency, status) on the RP2040 OLED

**Quick start:** Flash the dongle (`pio run -e dongle -t upload`), flash each robot with its ID (`ROBOT_ID=0 pio run -e receiver -t upload`), then run the terminal monitor (`./tools/swarm_terminal /dev/tty.usbmodem* 50`).