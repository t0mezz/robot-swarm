// debug_protocol.h
// Debug-Protokoll für UART-Kommunikation mit RP2040 screen_manager
// Registriert Felder (Graphen, Metriken) und sendet Wert-Updates

#pragma once
#include <cstdint>
#include <cstring>
#include "protocol.h"

// ─── Debug-Nachrichtentypen (UART zum RP2040) ────────────────
#define MSG_DEBUG_REG     0x22   // Feld registrieren
#define MSG_DEBUG_DATA    0x23   // Wert-Update

// ─── Display-Typen ───────────────────────────────────────────
#define DBG_METRIC        0x01   // Fixer Text an Position (x, y)
#define DBG_GRAPH         0x02   // LineGraph
#define DBG_LOG           0x03   // Log-Eintrag

// ─── Wert-Typen ──────────────────────────────────────────────
#define DBG_FLOAT32       0x01   // 4 Bytes IEEE 754
#define DBG_INT8          0x02   // 1 Byte signed
#define DBG_INT16         0x03   // 2 Bytes signed LE
#define DBG_STRING        0x04   // n Bytes UTF-8

// ─── Feld-IDs für den Robot-Debug-Screen ─────────────────────
#define DEBUG_FIELD_RSSI      0
#define DEBUG_FIELD_LATENCY   1
#define DEBUG_FIELD_STATUS    2
#define DEBUG_FIELD_MOTOR_L   3
#define DEBUG_FIELD_MOTOR_R   4

// ─── UART-Paket senden ──────────────────────────────────────
// Nutzt den gemeinsamen buildFrame aus protocol.h
// uart muss write(buf, len) unterstützen (Serial1 etc.)
namespace DebugScreen {

    // Prüft ob UART genug Platz hat und sendet Frame
    template<typename UartType>
    static void sendPacket(UartType& uart, uint8_t type,
                            const uint8_t* payload, uint8_t len) {
        if (uart.availableForWrite() < (int)(len + 5)) return;
        uint8_t frame[70];
        buildFrame(frame, type, payload, len);
        uart.write(frame, frameSize(len));
    }

    // ─── Felder registrieren ─────────────────────────────────
    template<typename UartType>
    static void registerField(UartType& uart, uint8_t fieldId,
                                uint8_t displayType, uint8_t x, uint8_t y,
                                const char* label) {
        uint8_t payload[20];
        payload[0] = fieldId;
        payload[1] = displayType;
        payload[2] = x;
        payload[3] = y;
        uint8_t labelLen = std::min((int)strlen(label), 15);
        memcpy(&payload[4], label, labelLen);
        sendPacket(uart, MSG_DEBUG_REG, payload, 4 + labelLen);
    }

    // ─── Alle Robot-Debug-Felder registrieren ────────────────
    template<typename UartType>
    static void registerAllFields(UartType& uart) {
        registerField(uart, DEBUG_FIELD_RSSI,    DBG_GRAPH,  0,  0,  "RSSI");
        registerField(uart, DEBUG_FIELD_LATENCY, DBG_METRIC, 0,  34, "LAT");
        registerField(uart, DEBUG_FIELD_STATUS,  DBG_METRIC, 64, 34, "STA");
        registerField(uart, DEBUG_FIELD_MOTOR_L, DBG_METRIC, 0,  44, "ML");
        registerField(uart, DEBUG_FIELD_MOTOR_R, DBG_METRIC, 64, 44, "MR");
    }

    // ─── Wert-Updates ────────────────────────────────────────
    template<typename UartType>
    static void sendInt8(UartType& uart, uint8_t fieldId, int8_t value) {
        uint8_t payload[3] = {fieldId, DBG_INT8, (uint8_t)value};
        sendPacket(uart, MSG_DEBUG_DATA, payload, 3);
    }

    template<typename UartType>
    static void sendString(UartType& uart, uint8_t fieldId, const char* str) {
        uint8_t payload[20];
        payload[0] = fieldId;
        payload[1] = DBG_STRING;
        uint8_t len = std::min((int)strlen(str), 17);
        memcpy(&payload[2], str, len);
        sendPacket(uart, MSG_DEBUG_DATA, payload, 2 + len);
    }

    // ─── Alle Debug-Werte auf einmal aktualisieren ───────────
    template<typename UartType>
    static void updateAll(UartType& uart, int8_t rssi, uint16_t latencyUs,
                           const char* status, int8_t motorL, int8_t motorR) {
        sendInt8(uart, DEBUG_FIELD_RSSI, rssi);
        sendInt8(uart, DEBUG_FIELD_MOTOR_L, motorL);
        sendInt8(uart, DEBUG_FIELD_MOTOR_R, motorR);

        char latBuf[10];
        snprintf(latBuf, sizeof(latBuf), "%uus", latencyUs);
        sendString(uart, DEBUG_FIELD_LATENCY, latBuf);
        sendString(uart, DEBUG_FIELD_STATUS, status);
    }
}
