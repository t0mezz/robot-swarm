// swarm_receiver.ino
// Robot Swarm Receiver — ESP32 auf dem Roboter
// Nutzt: protocol.h, hardware.h, debug_protocol.h
//
// Board: ESP32-S3 SuperMini | Arduino ESP32 Core 3.x
//
// Led blinkt 

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"
#include "hardware.h"
#include "debug_protocol.h"

// ═══════════════════════════════════════════════════════════════
// Transport Layer — für späteren Wechsel auf WiFi STA
// ═══════════════════════════════════════════════════════════════

namespace Transport {
    static uint8_t dongleMAC[6] = {0};
    static bool    dongleKnown  = false;
    static uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    void init() {
        WiFi.mode(WIFI_STA);
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

        if (esp_now_init() != ESP_OK) {
            Serial.println("[ERR] ESP-NOW init failed");
            return;
        }

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, broadcastMAC, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    void registerDongle(const uint8_t* mac) {
        if (dongleKnown) return;
        memcpy(dongleMAC, mac, 6);
        dongleKnown = true;

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, dongleMAC, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    bool sendBroadcast(const uint8_t* data, size_t len) {
        return esp_now_send(broadcastMAC, data, len) == ESP_OK;
    }

    bool sendToDongle(const uint8_t* data, size_t len) {
        if (!dongleKnown) return sendBroadcast(data, len);
        return esp_now_send(dongleMAC, data, len) == ESP_OK;
    }

    void resetDongle() {
        if (dongleKnown) {
            esp_now_del_peer(dongleMAC);
            dongleKnown = false;
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Zustand
// ═══════════════════════════════════════════════════════════════

enum class State { ANNOUNCING, ACTIVE };
static State state = State::ANNOUNCING;

static volatile bool     hasData  = false;
static uint8_t           rxBuf[250];
static volatile uint8_t  rxLen    = 0;

static unsigned long lastAnnounce      = 0;
static unsigned long lastSwarmReceived = 0;
static unsigned long lastSpeedApplied  = 0;
static unsigned long lastDebugUpdate   = 0;
static unsigned long bootTime          = 0;
static unsigned long ledOffAt          = 0;
static bool          debugRegistered   = false;

static int8_t   currentMotorL = 0;
static int8_t   currentMotorR = 0;
static uint16_t lastLatencyUs = 0;
static uint8_t  statusFlags   = STATUS_ANNOUNCING;

// ═══════════════════════════════════════════════════════════════
// ESP-NOW Callback
// ═══════════════════════════════════════════════════════════════

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (len > 0 && len <= (int)sizeof(rxBuf)) {
        memcpy(rxBuf, data, len);
        rxLen  = len;
        hasData = true;
        if(!Transport::dongleKnown) Transport::registerDongle(mac);
    }
}

// ═══════════════════════════════════════════════════════════════
// Announce
// ═══════════════════════════════════════════════════════════════

static void sendAnnounce() {
    uint8_t mac[6];
    WiFi.macAddress(mac);

    uint8_t payload[7] = {ROBOT_ID};
    memcpy(&payload[1], mac, 6);

    uint8_t frame[12];
    buildFrame(frame, MSG_ANNOUNCE, payload, 7);
    Transport::sendBroadcast(frame, 12);
    Serial.printf("[ANN] Announce sent (ID=%d)\n", ROBOT_ID);
}

static void uart_send_speed(int8_t left, int8_t right);  // forward declaration

// ═══════════════════════════════════════════════════════════════
// Paket-Handler
// ═══════════════════════════════════════════════════════════════

static void processIncoming(const uint8_t* data, uint8_t len) {
    if (!validateFrame(data, len)) return;

    uint8_t type       = data[2];
    uint8_t payloadLen = data[3];
    const uint8_t* payload = &data[4];

    switch (type) {
        case MSG_ANNOUNCE_ACK: {
            if (state != State::ANNOUNCING) break;
            if (payloadLen < 1 || payload[0] != ROBOT_ID) break;
            state = State::ACTIVE;
            statusFlags &= ~STATUS_ANNOUNCING;
            lastSwarmReceived = millis();
            Serial.printf("[ACK] Registered as Robot %d\n", ROBOT_ID);
            break;
        }

        // LED flashen und uart senden
        case MSG_SWARM: {
            lastSwarmReceived = millis();
            if (state == State::ANNOUNCING) break;
            // Eigene ID suchen
            uint8_t entries = payloadLen / 3;
            for (uint8_t i = 0; i < entries; i++) {
                if (payload[i * 3] == ROBOT_ID) {
                    currentMotorL = (int8_t)payload[i * 3 + 1];
                    currentMotorR = (int8_t)payload[i * 3 + 2];
                    uart_send_speed(currentMotorL, currentMotorR);
                    lastSpeedApplied = millis();
                    digitalWrite(LED_PIN, LED_ON);
                    ledOffAt = millis() + LED_ON_DURATION_MS;
                    break;
                }
            }
            break;
        }

        case MSG_PING: {
            if (payloadLen < 5) break;
            uint8_t targetId = payload[0];
            if (targetId != ROBOT_ID && targetId != 0xFF) break;

            // Echo: Timestamp zurücksenden
            uint8_t pongPayload[5] = {ROBOT_ID};
            memcpy(&pongPayload[1], &payload[1], 4);
            uint8_t frame[10];
            buildFrame(frame, MSG_PONG, pongPayload, 5);
            Transport::sendToDongle(frame, 10);
            break;
        }

        case MSG_PONG: {
            if (payloadLen < 3 || payload[0] != ROBOT_ID) break;
            lastLatencyUs = payload[1] | (payload[2] << 8);
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// UART an RP2040
// ═══════════════════════════════════════════════════════════════

static uint8_t uartRxBuf[64];
static uint8_t uartRxIdx = 0;

static void processUartFrame(const uint8_t* data, uint8_t len) {
    if (!validateFrame(data, len)) return;
    if (data[2] == MSG_DEBUG_PING && !debugRegistered) {
        DebugScreen::registerAllFields(UART);
        debugRegistered = true;
    }
}

static void pollUart() {
    while (UART.available()) {
        uint8_t b = (uint8_t)UART.read();
        if (uartRxIdx == 0 && b != MAGIC_0) continue;
        if (uartRxIdx == 1 && b != MAGIC_1) { uartRxIdx = 0; continue; }
        uartRxBuf[uartRxIdx++] = b;
        if (uartRxIdx >= 4) {
            uint8_t needed = frameSize(uartRxBuf[3]);
            if (uartRxIdx >= needed) {
                processUartFrame(uartRxBuf, needed);
                uartRxIdx = 0;
            }
        }
        if (uartRxIdx >= sizeof(uartRxBuf)) uartRxIdx = 0;
    }
}

static void uart_send_speed(int8_t left, int8_t right) {
    uint8_t payload[2] = {(uint8_t)left, (uint8_t)right};
    uint8_t frame[7];
    buildFrame(frame, MSG_SPEED, payload, 2);
    if (UART.availableForWrite() >= 7) UART.write(frame, 7);
}

// ═══════════════════════════════════════════════════════════════
// Telemetrie (TDMA)
// ═══════════════════════════════════════════════════════════════

static void sendTelemetry() {
    uint16_t uptime = (uint16_t)((millis() - bootTime) / 1000);

    uint8_t payload[7] = {
        ROBOT_ID,
        0,                       // Batterie (TODO)
        statusFlags,
        (uint8_t)currentMotorL,
        (uint8_t)currentMotorR,
        (uint8_t)(uptime & 0xFF),
        (uint8_t)(uptime >> 8)
    };

    uint8_t frame[12];
    buildFrame(frame, MSG_TELEMETRY, payload, 7);
    Transport::sendToDongle(frame, 12);
}

static bool isMyTDMASlot() {
    if (lastSwarmReceived == 0) return false;
    unsigned long cyclePos = (millis() - lastSwarmReceived) % TDMA_CYCLE_MS;
    unsigned long slotStart = ROBOT_ID * TDMA_SLOT_MS;
    return (cyclePos >= slotStart && cyclePos < slotStart + TDMA_SLOT_MS);
}

// ═══════════════════════════════════════════════════════════════
// Watchdog
// ═══════════════════════════════════════════════════════════════

static void checkWatchdog() {
    if (lastSpeedApplied == 0) return;
    if (millis() - lastSpeedApplied > WATCHDOG_TIMEOUT_MS) {
        if (currentMotorL != 0 || currentMotorR != 0) {
            currentMotorL = currentMotorR = 0;
            uart_send_speed(0, 0);
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Setup & Loop
// ═══════════════════════════════════════════════════════════════

void setup() {
    bootTime = millis();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    UART.setTxBufferSize(512);
    UART.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && millis() - t < 2000) delay(10);

    Serial.printf("Swarm Receiver ID:%d\n", ROBOT_ID);
    Serial.print("MAC: "); Serial.println(WiFi.macAddress());

    Transport::init();
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb([](const uint8_t*, esp_now_send_status_t) {});
}

void loop() {
    unsigned long now = millis();

    // Empfang
    if (hasData) {
        hasData = false;
        processIncoming(rxBuf, rxLen);
    }

    // State Machine
    if (state == State::ANNOUNCING) {
        if (now - lastAnnounce >= ANNOUNCE_INTERVAL_MS) {
            sendAnnounce();
            lastAnnounce = now;
        }
    } else {
        // Timeout -> re-announce
        if (now - lastSwarmReceived > ANNOUNCE_TIMEOUT_MS) {
            state = State::ANNOUNCING;
            statusFlags |= STATUS_ANNOUNCING;
            Transport::resetDongle();
            currentMotorL = currentMotorR = 0;
            uart_send_speed(0, 0);
        }

        // TDMA Telemetrie
        static unsigned long lastTelSent = 0;
        if (isMyTDMASlot() && now - lastTelSent >= TDMA_CYCLE_MS - 10) {
            sendTelemetry();
            lastTelSent = now;
        }
    }

    checkWatchdog();

    // Debug-Screen
    if (debugRegistered && now - lastDebugUpdate >= DEBUG_UPDATE_MS) {
        const char* st = (state == State::ANNOUNCING) ? "ANN" :
                         (currentMotorL != 0 || currentMotorR != 0) ? "RUN" : "IDL";
        DebugScreen::updateAll(UART, lastLatencyUs, st,
                                currentMotorL, currentMotorR);
        lastDebugUpdate = now;
    }

    // LED
    if (ledOffAt && now >= ledOffAt) { digitalWrite(LED_PIN, LED_OFF); ledOffAt = 0; }
}
