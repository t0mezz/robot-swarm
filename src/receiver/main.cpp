// swarm_receiver.ino
// Robot Swarm Receiver — ESP32 auf dem Roboter
// Nutzt: protocol.h, hardware.h, debug_protocol.h
//
// Board: ESP32-S3 SuperMini | Arduino ESP32 Core 3.x

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
        WiFi.disconnect();
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
static volatile int      rxRSSI   = 0;

static unsigned long lastAnnounce      = 0;
static unsigned long lastSwarmReceived = 0;
static unsigned long lastSpeedApplied  = 0;
static unsigned long lastDebugUpdate   = 0;
static unsigned long bootTime          = 0;
static unsigned long ledOffAt          = 0;
static bool          debugRegistered   = false;

static int8_t   currentMotorL = 0;
static int8_t   currentMotorR = 0;
static int8_t   lastRSSI      = 0;
static uint16_t lastLatencyUs = 0;
static uint8_t  statusFlags   = STATUS_ANNOUNCING;

// ═══════════════════════════════════════════════════════════════
// ESP-NOW Callback
// ═══════════════════════════════════════════════════════════════

void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len > 0 && len <= (int)sizeof(rxBuf)) {
        memcpy(rxBuf, data, len);
        rxLen  = len;
        rxRSSI = info->rx_ctrl->rssi;
        hasData = true;
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
}

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

        case MSG_SWARM: {
            lastSwarmReceived = millis();
            if (state == State::ANNOUNCING) {
                state = State::ACTIVE;
                statusFlags &= ~STATUS_ANNOUNCING;
            }
            // Eigene ID suchen
            uint8_t entries = payloadLen / 3;
            for (uint8_t i = 0; i < entries; i++) {
                if (payload[i * 3] == ROBOT_ID) {
                    currentMotorL = (int8_t)payload[i * 3 + 1];
                    currentMotorR = (int8_t)payload[i * 3 + 2];
                    uart_send_speed(currentMotorL, currentMotorR);
                    lastSpeedApplied = millis();
                    statusFlags |= STATUS_MOTOR_ACTIVE;
                    digitalWrite(LED_PIN, LED_ON);
                    ledOffAt = millis() + 5;
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

    uint8_t payload[8] = {
        ROBOT_ID,
        (uint8_t)lastRSSI,
        0,                       // Batterie (TODO)
        statusFlags,
        (uint8_t)currentMotorL,
        (uint8_t)currentMotorR,
        (uint8_t)(uptime & 0xFF),
        (uint8_t)(uptime >> 8)
    };

    uint8_t frame[13];
    buildFrame(frame, MSG_TELEMETRY, payload, 8);
    Transport::sendToDongle(frame, 13);
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
            statusFlags &= ~STATUS_MOTOR_ACTIVE;
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

    Serial.printf("═══ Swarm Receiver ID:%d ═══\n", ROBOT_ID);
    Serial.print("MAC: "); Serial.println(WiFi.macAddress());

    Transport::init();
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb([](const wifi_tx_info_t*, esp_now_send_status_t) {});
}

void loop() {
    unsigned long now = millis();

    // Empfang
    if (hasData) {
        hasData = false;
        lastRSSI = (int8_t)rxRSSI;
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
    if (!debugRegistered && now - bootTime > DEBUG_REG_DELAY_MS) {
        DebugScreen::registerAllFields(UART);
        debugRegistered = true;
    }
    if (debugRegistered && now - lastDebugUpdate >= DEBUG_UPDATE_MS) {
        const char* st = (state == State::ANNOUNCING) ? "ANN" :
                         (statusFlags & STATUS_MOTOR_ACTIVE) ? "RUN" : "IDL";
        DebugScreen::updateAll(UART, lastRSSI, lastLatencyUs, st,
                                currentMotorL, currentMotorR);
        lastDebugUpdate = now;
    }

    // UART vom RP2040 verwerfen (vorerst)
    while (UART.available()) UART.read();

    // LED
    if (ledOffAt && now >= ledOffAt) { digitalWrite(LED_PIN, LED_OFF); ledOffAt = 0; }
}
