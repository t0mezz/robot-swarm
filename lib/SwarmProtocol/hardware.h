// hardware.h
// Hardware-Konfiguration und Timing-Konstanten
// Gilt für Robot-ESP32 (Receiver)

#pragma once

// ─── Robot-ID ────────────────────────────────────────────────
// MUSS pro Roboter eindeutig sein (0-19)
// Kann hier oder per build-flag (-DROBOT_ID=3) gesetzt werden
#ifndef ROBOT_ID
#define ROBOT_ID          0
#endif

// ─── Pins (ESP32-S3 SuperMini) ───────────────────────────────
#define LED_PIN           8
#define LED_ON            LOW
#define LED_OFF           HIGH
#define UART_TX_PIN       43
#define UART_RX_PIN       44
#define UART_BAUD         921600
#define UART              Serial1

// ─── ESP-NOW ─────────────────────────────────────────────────
#define ESPNOW_CHANNEL    1

// ─── Timing ──────────────────────────────────────────────────
#define ANNOUNCE_INTERVAL_MS   500     // Announce-Rate im Registrierungsmodus
#define ANNOUNCE_TIMEOUT_MS    10000   // 10s ohne Swarm -> re-announce
#define TDMA_CYCLE_MS          1000    // Telemetrie-Zyklus
#define TDMA_SLOT_MS           50      // Slot-Breite pro Robot
#define WATCHDOG_TIMEOUT_MS    300     // Motor-Stop nach 300ms ohne Paket
#define DEBUG_REG_DELAY_MS     2000    // Debug-Felder nach Boot registrieren
#define DEBUG_UPDATE_MS        250     // Debug-Daten Aktualisierung
