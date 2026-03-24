// hardware.h
// Hardware-Konfiguration und Timing-Konstanten
// Gilt für Robot-ESP32 (Receiver)

#pragma once

// ─── Robot-ID ────────────────────────────────────────────────
// Set at flash time: ROBOT_ID=3 pio run -e receiver -t upload
// extra_script.py generates this header in $BUILD_DIR so SCons
// recompiles whenever the value changes.
#include "robot_id_cfg.h"

// ─── Pins (ESP32-C3 SuperMini v2) ────────────────────────────
#define LED_PIN           8   // WS2812 RGB LED (GRB order)
#define UART_TX_PIN       4
#define UART_RX_PIN       3
#define UART_BAUD         921600
#define UART              Serial1

// ─── ESP-NOW ─────────────────────────────────────────────────
#define ESPNOW_CHANNEL    1

// ─── Timing ──────────────────────────────────────────────────
#define ANNOUNCE_INTERVAL_MS   500     // Announce-Rate im Registrierungsmodus
#define ANNOUNCE_TIMEOUT_MS    10000   // 10s ohne Swarm -> re-announce
#define REANNOUNCE_INTERVAL_MS 30000   // Periodic re-announce while ACTIVE (swarm_terminal reconnect)
#define TDMA_SLOT_MS           100     // Slot width per robot; cycle = MAX_ROBOTS * TDMA_SLOT_MS = 6.4s for 64 robots
#define WATCHDOG_TIMEOUT_MS    1000    // Motor-Stop nach 1000ms ohne Paket
#define DEBUG_UPDATE_MS        250     // Debug-Daten Aktualisierung
#define LED_ON_DURATION_MS     5       // LED an für ms bei Empfang