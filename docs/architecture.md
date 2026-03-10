# Robot Swarm Pipeline — Architekturplan

## Übersicht

```
┌──────────────┐    USB-Serial     ┌──────────────┐   ESP-NOW    ┌──────────────┐
│  Controller  │ ──────────────▶   │    Dongle    │  Broadcast   │   Robot 0    │
│     (PC)     │ ◀──────────────   │   (ESP32)    │ ──────────▶  │   (ESP32)    │
│              │    921600 Baud     │              │  ◀─────────  │      │       │
└──────────────┘                   └──────────────┘   Unicast    │   UART      │
                                                      (TDMA)     │      ▼       │
                                                                 │   RP2040    │
                                                                 └──────────────┘
                                                                       × 20
```

---

## 1. Roboter-Registrierung

### Problem
Der Dongle muss wissen, welche Roboter aktiv sind, und jeder Roboter braucht eine eindeutige ID.

### Lösung: Automatische Registrierung per Announce

Beim Einschalten sendet jeder Roboter periodisch ein **ANNOUNCE**-Paket per ESP-NOW Broadcast, bis der Dongle ihn bestätigt.

```
Roboter → Broadcast:
  [0xAA][0x55][0x20][6][ ROBOT_ID | MAC[0..4] ][CRC]
  MSG_ANNOUNCE = 0x20

Dongle → Broadcast (Bestätigung):
  [0xAA][0x55][0x21][1][ ROBOT_ID ][CRC]
  MSG_ANNOUNCE_ACK = 0x21
```

**Ablauf:**

1. Robot bootet, liest `ROBOT_ID` aus Flash (konfiguriert beim Flashen, 0–19)
2. Robot sendet alle 500ms `MSG_ANNOUNCE` mit seiner ID und MAC-Adresse
3. Dongle empfängt Announce, speichert ID→MAC Mapping, antwortet mit `MSG_ANNOUNCE_ACK`
4. Robot empfängt ACK, stoppt Announce, geht in normalen Empfangsmodus
5. Dongle leitet die Registrierung per Serial an den PC weiter
6. PC zeigt im Controller-UI an, welche Roboter aktiv sind

**Timeout:** Wenn ein Roboter 10 Sekunden kein Swarm-Paket empfängt, beginnt er wieder zu announcen.

### Roboter-ID Vergabe

Statisch per `#define ROBOT_ID` im Code. Für die Produktion kann die ID auch per NVS (Non-Volatile Storage) gesetzt werden, z.B. über einen einmaligen Konfigurations-Befehl per USB-Serial.

---

## 2. Steuerkanal (PC → Roboter)

### Paketformat: MSG_SWARM (0x10)

Ein einzelnes Broadcast-Paket steuert alle 20 Roboter gleichzeitig.

```
[0xAA][0x55][0x10][60][ 0|L|R | 1|L|R | 2|L|R | ... | 19|L|R ][CRC]

Pro Roboter: 3 Bytes
  - Byte 0: Robot-ID (uint8)
  - Byte 1: Left Motor (int8, -127..+127)
  - Byte 2: Right Motor (int8, -127..+127)

Gesamt: 65 Bytes (Header 4 + Payload 60 + CRC 1)
```

### Timing

```
PC sendet alle 50ms ein MSG_SWARM → 20 Pakete/Sekunde
```

### Datenfluss

```
PC (controller.cpp)
  │  Baut 65-Byte Frame
  │  Schreibt auf USB-Serial @ 921600 Baud
  ▼
Dongle (ESP32)
  │  Sammelt Serial-Bytes bis 2ms Pause
  │  Sendet gesamten Frame als ESP-NOW Broadcast
  ▼
Alle 20 Roboter empfangen gleichzeitig (~1ms)
  │  Jeder Robot parsed den Frame
  │  Findet seinen ID-Slot, extrahiert L/R
  │  Baut MSG_SPEED [0xAA][0x55][0x01][2][L][R][CRC]
  │  Sendet 7 Bytes an UART @ 921600 Baud
  ▼
RP2040 empfängt und setzt Motoren
```

### Latenz-Budget

| Segment | Dauer |
|---------|-------|
| PC → Serial Write | < 0.1ms |
| Serial → Dongle Buffer | ~0.6ms (65 Bytes @ 921600) |
| Dongle Serial-Timeout | 2ms |
| ESP-NOW Broadcast | ~1ms |
| Robot Parse + UART Write | ~0.2ms |
| UART → RP2040 Parse | ~0.1ms |
| **Gesamt** | **~4ms** |

---

## 3. Telemetrie-Rückkanal (Roboter → PC)

### Problem
20 Roboter können nicht gleichzeitig senden — ESP-NOW Kollisionen.

### Lösung: TDMA (Time Division Multiple Access)

Jeder Roboter hat ein festes Zeitfenster basierend auf seiner ID.

```
Zyklus: 1000ms (1 Sekunde)
Slot-Breite: 1000ms / 20 = 50ms pro Roboter
Robot 0 sendet bei:    0ms,  1000ms, 2000ms, ...
Robot 1 sendet bei:   50ms,  1050ms, 2050ms, ...
Robot 2 sendet bei:  100ms,  1100ms, 2100ms, ...
...
Robot 19 sendet bei: 950ms,  1950ms, 2950ms, ...
```

### Synchronisation

Der Dongle sendet am Anfang jedes Swarm-Pakets einen impliziten Takt. Die Roboter synchronisieren sich darauf:

```
Robot berechnet seinen Slot:
  slot_offset = ROBOT_ID * SLOT_WIDTH
  time_since_last_swarm = millis() - last_swarm_received
  if (time_since_last_swarm >= slot_offset &&
      time_since_last_swarm < slot_offset + SLOT_WIDTH)
      → JETZT senden!
```

Das ist eine einfache, robuste Synchronisation ohne expliziten Sync-Befehl.

### Telemetrie-Paketformat: MSG_TELEMETRY (0x30)

```
[0xAA][0x55][0x30][len][ ROBOT_ID | RSSI | BATTERY | STATUS | ... ][CRC]
MSG_TELEMETRY = 0x30

Payload (8 Bytes):
  - Byte 0: Robot-ID (uint8)
  - Byte 1: WiFi RSSI (int8, dBm)
  - Byte 2: Batterie-Spannung (uint8, 0-255 → 0-5V)
  - Byte 3: Status-Flags (uint8)
      Bit 0: Motor aktiv
      Bit 1: Sensor-Fehler
      Bit 2: Low Battery
      Bit 3: Announce-Modus (noch nicht registriert)
  - Byte 4-5: Aktueller Motor L/R (int8, int8) — Ist-Werte
  - Byte 6-7: Uptime (uint16, Sekunden, little-endian)

Gesamt: 13 Bytes
```

### Datenfluss Rückkanal

```
RP2040 → UART → Robot-ESP32
  │  Telemetrie-Daten sammeln
  │  Im eigenen TDMA-Slot:
  │  ESP-NOW Unicast an Dongle-MAC (zuverlaessiger als Broadcast)
  ▼
Dongle empfängt
  │  Leitet per USB-Serial an PC weiter
  ▼
PC parsed Telemetrie
  │  Aktualisiert Dashboard pro Roboter
```

---

## 4. Protokoll-Übersicht

| Typ | Code | Richtung | Beschreibung |
|-----|------|----------|-------------|
| MSG_SPEED | 0x01 | ESP32 → RP2040 (UART) | Einzelner Motor-Befehl [L, R] |
| MSG_SWARM | 0x10 | PC → Dongle → Broadcast | Alle 20 Roboter in einem Paket |
| MSG_ANNOUNCE | 0x20 | Robot → Broadcast | Roboter meldet sich an |
| MSG_ANNOUNCE_ACK | 0x21 | Dongle → Broadcast | Bestätigung der Anmeldung |
| MSG_PING | 0x22 | Bidirektional | Heartbeat / Verbindungstest |
| MSG_TELEMETRY | 0x30 | Robot → Dongle (Unicast) | Status, RSSI, Batterie |
| MSG_CONFIG | 0x40 | PC → Dongle → Unicast | Konfiguration an einzelnen Robot |

---

## 5. Fehlerbehandlung

### Motor-Watchdog (auf RP2040)

```python
WATCHDOG_TIMEOUT_MS = 300

if time_since_last_speed_packet > WATCHDOG_TIMEOUT_MS:
    motors.set_speeds(0, 0)  # Notfall-Stop
```

Wenn 300ms kein neues Speed-Paket kommt, stoppen die Motoren automatisch. Das verhindert unkontrolliertes Weiterfahren bei Verbindungsverlust.

### Dongle-Watchdog

Wenn der Dongle 5 Sekunden keine Serial-Daten vom PC empfängt, sendet er ein Stop-Broadcast (alle Motoren auf 0).

### Robot-Reconnect

Wenn ein Robot 10 Sekunden kein Swarm-Paket empfängt:
1. Motoren stoppen (Watchdog greift schon nach 300ms)
2. Wechselt zurück in Announce-Modus
3. Beginnt wieder alle 500ms zu announcen

### CRC-Fehler

Pakete mit ungültigem CRC werden auf allen Stufen still verworfen. Kein Retry, das nächste Paket kommt in 50ms.

---

## 6. Controller-PC Software

### Architektur

```
┌─────────────────────────────────────────────┐
│              Controller (C++/SFML)           │
│                                              │
│  ┌──────────┐  ┌───────────┐  ┌──────────┐  │
│  │  Input   │  │  Vehicle  │  │  Swarm   │  │
│  │  (WASD)  │→ │  Physics  │→ │  Manager │  │
│  └──────────┘  └───────────┘  └────┬─────┘  │
│                                     │        │
│  ┌──────────┐  ┌───────────┐       │        │
│  │  Teleme- │← │  Serial   │←──────┘        │
│  │  try UI  │  │  Port     │→ (TX: Swarm)   │
│  └──────────┘  └───────────┘← (RX: Telemetry)│
│                                              │
│  ┌──────────────────────────────────────┐    │
│  │           Cockpit HUD (SFML)         │    │
│  │  Tacho │ Trail │ G-Force │ Output    │    │
│  │  Boost │ Keys  │ Robots  │ Telemetry │    │
│  └──────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
```

### Swarm Manager

- Hält Array von 20 Robot-Slots mit Status (aktiv/inaktiv/error)
- Empfängt Telemetrie und updated Status pro Robot
- Zeigt Verbindungsqualität (RSSI) und Batterie an
- Markiert Roboter als "lost" wenn Telemetrie > 3s ausbleibt

### Multi-Robot Steuerung (Zukunft)

Aktuell: Ein Roboter per WASD, Rest bekommt Nullwerte.

Nächster Schritt: Schwarm-Algorithmen auf dem PC:
- Formation (alle fahren gleich, versetzt)
- Follow-the-Leader (Robot 0 wird gesteuert, Rest folgt)
- Choreographie (vorprogrammierte Muster)

---

## 7. Hardware-Checkliste pro Roboter

| Komponente | Funktion |
|-----------|----------|
| ESP32-S3 SuperMini | WiFi/ESP-NOW Bridge |
| Pololu 3pi+ 2040 | Motorsteuerung, Sensorik |
| UART-Verbindung | ESP32 TX→RP2040 RX, ESP32 RX→RP2040 TX |
| Baudrate | 921600, 8N1 |
| ESP-NOW Kanal | 1 (fest, alle gleich) |
| ROBOT_ID | 0–19, eindeutig pro Roboter |

### Zentrale Infrastruktur

| Komponente | Funktion |
|-----------|----------|
| ESP32 Dongle | USB am PC, ESP-NOW Broadcast/Empfang |
| PC mit SFML | Controller-Software, Cockpit HUD |
| USB-Serial | 921600 Baud, PC ↔ Dongle |

---

## 8. Implementierungsreihenfolge

### Phase 1: Basis (bereits fertig)
- [x] Einzelroboter-Steuerung per UDP
- [x] ESP-NOW Punkt-zu-Punkt
- [x] Broadcast 20-Roboter Paket
- [x] Controller mit Cockpit HUD

### Phase 2: Registrierung & Telemetrie
- [ ] MSG_ANNOUNCE / MSG_ANNOUNCE_ACK im Dongle und Robot
- [ ] TDMA-Telemetrie Rückkanal im Robot
- [ ] Telemetrie-Parser im Dongle (Serial → PC)
- [ ] Motor-Watchdog auf RP2040
- [ ] Robot-Status Dashboard im Controller

### Phase 3: Robustheit
- [ ] Dongle-Watchdog (Stop-Broadcast bei PC-Disconnect)
- [ ] Robot-Reconnect nach Timeout
- [ ] RSSI-Monitoring und Warnung bei schwachem Signal
- [ ] Batterie-Monitoring

### Phase 4: Schwarm-Algorithmen
- [ ] Multi-Robot Steuerung im Controller
- [ ] Formation-Modus
- [ ] Follow-the-Leader
- [ ] Choreographie-Sequenzer
