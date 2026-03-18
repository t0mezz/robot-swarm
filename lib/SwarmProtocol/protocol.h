// protocol.h
// Gemeinsames Protokoll für Swarm Receiver und Dongle
// Nachrichtentypen, CRC-8, Frame-Builder, Status-Flags

#pragma once
#include <cstdint>
#include <cstring>

// ─── Frame ───────────────────────────────────────────────────
#define MAGIC_0           0xAA
#define MAGIC_1           0x55

// ─── Nachrichtentypen ────────────────────────────────────────
#define MSG_SPEED         0x01   // ESP32 -> RP2040 (UART): [left, right]
#define MSG_ROBOT_ID      0x04   // ESP32 -> RP2040 (UART): [robot_id]
#define MSG_SWARM         0x10   // PC -> Broadcast: [id|L|R] x MAX_ROBOTS
#define MSG_ANNOUNCE      0x20   // Robot -> Broadcast: [id, mac x6]
#define MSG_ANNOUNCE_ACK  0x21   // Dongle -> Broadcast: [id]
#define MSG_PING          0x22   // Bidirektional: [target_id, timestamp x4]
#define MSG_PONG          0x23   // Antwort: [id, roundtrip_us x2]
#define MSG_TELEMETRY     0x30   // Robot -> Dongle: [id, bat, flags, mL, mR, uptime x2]
                                 // TODO: add RSSI once ESP-IDF 5.x is used (esp_now_recv_info_t)
#define MSG_CONFIG        0x40   // PC -> Robot

// ─── Telemetrie Status-Flags ─────────────────────────────────
#define STATUS_LOW_BATTERY    0x04
#define STATUS_ANNOUNCING     0x08

// ─── Limits ──────────────────────────────────────────────────
#define MAX_ROBOTS        32

// ─── CRC-8 (Polynom 0x07) ────────────────────────────────────
static inline uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

// ─── Frame bauen ─────────────────────────────────────────────
static inline void buildFrame(uint8_t* buf, uint8_t type,
                                const uint8_t* payload, uint8_t payloadLen) {
    buf[0] = MAGIC_0;
    buf[1] = MAGIC_1;
    buf[2] = type;
    buf[3] = payloadLen;
    memcpy(&buf[4], payload, payloadLen);
    buf[4 + payloadLen] = crc8(&buf[2], payloadLen + 2);
}

// ─── Frame validieren ────────────────────────────────────────
static inline bool validateFrame(const uint8_t* data, uint8_t len) {
    if (len < 5) return false;
    if (data[0] != MAGIC_0 || data[1] != MAGIC_1) return false;
    uint8_t payloadLen = data[3];
    if ((int)(4 + payloadLen + 1) > len) return false;
    return crc8(&data[2], payloadLen + 2) == data[4 + payloadLen];
}

// ─── Frame-Grösse ────────────────────────────────────────────
static inline uint8_t frameSize(uint8_t payloadLen) {
    return 4 + payloadLen + 1;
}
