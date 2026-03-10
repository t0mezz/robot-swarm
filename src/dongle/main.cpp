// swarm_dongle.ino
// Swarm Dongle — ESP32 am Controller-PC
// Nutzt: protocol.h
//
// Arduino ESP32 Core 3.x

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"

// ─── Hardware ────────────────────────────────────────────────
#define LED_PIN           8
#define LED_ON            LOW
#define LED_OFF           HIGH
#define ESPNOW_CHANNEL    1

// ─── Roboter-Registry ────────────────────────────────────────

struct Robot {
    bool          active;
    uint8_t       mac[6];
    unsigned long lastSeen;
    int8_t        rssi;
};

static Robot robots[MAX_ROBOTS] = {};
static uint8_t    broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ─── Ping-Tracking ──────────────────────────────────────────

struct PingTracker {
    bool     pending;
    uint8_t  robotId;
    uint32_t sentAt;       // micros()
};

static PingTracker pingTracker = {false, 0, 0};

// ─── Empfangs-Buffer ─────────────────────────────────────────

static volatile bool    hasData  = false;
static uint8_t          rxBuf[250];
static volatile uint8_t rxLen    = 0;
static volatile int     rxRSSI   = 0;
static uint8_t          rxMAC[6] = {};

void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len > 0 && len <= (int)sizeof(rxBuf)) {
        memcpy(rxBuf, data, len);
        rxLen  = len;
        rxRSSI = info->rx_ctrl->rssi;
        memcpy(rxMAC, info->src_addr, 6);
        hasData = true;
    }
}

// ─── Serial vom PC ───────────────────────────────────────────

#define SERIAL_TIMEOUT_MS  2
static uint8_t       serialBuf[250];
static uint8_t       serialLen = 0;
static unsigned long lastByteAt = 0;

// ─── Eingehende ESP-NOW Pakete routen ────────────────────────

static void routeIncoming(const uint8_t* data, uint8_t len, const uint8_t* mac) {
    if (!validateFrame(data, len)) return;

    uint8_t type       = data[2];
    uint8_t payloadLen = data[3];
    const uint8_t* payload = &data[4];

    switch (type) {
        case MSG_ANNOUNCE: {
            if (payloadLen < 7) break;
            uint8_t robotId = payload[0];
            if (robotId >= MAX_ROBOTS) break;

            // Registrieren
            robots[robotId].active = true;
            memcpy(robots[robotId].mac, mac, 6);
            robots[robotId].lastSeen = millis();
            robots[robotId].rssi = (int8_t)rxRSSI;

            // ACK
            uint8_t ackPayload[1] = {robotId};
            uint8_t ackFrame[6];
            buildFrame(ackFrame, MSG_ANNOUNCE_ACK, ackPayload, 1);
            esp_now_send(broadcastMAC, ackFrame, 6);

            // An PC: [id][mac x6][rssi]
            uint8_t regPayload[8] = {robotId};
            memcpy(&regPayload[1], mac, 6);
            regPayload[7] = (uint8_t)(int8_t)rxRSSI;
            uint8_t regFrame[13];
            buildFrame(regFrame, MSG_ANNOUNCE, regPayload, 8);
            Serial.write(regFrame, 13);

            digitalWrite(LED_PIN, LED_ON);
            break;
        }

        case MSG_TELEMETRY: {
            if (payloadLen >= 1 && payload[0] < MAX_ROBOTS) {
                robots[payload[0]].lastSeen = millis();
                robots[payload[0]].rssi = (int8_t)rxRSSI;
            }
            // 1:1 an PC
            Serial.write(data, len);
            break;
        }

        case MSG_PONG: {
            if (payloadLen < 5) break;
            uint8_t robotId = payload[0];

            if (pingTracker.pending && pingTracker.robotId == robotId) {
                uint32_t roundtripUs = micros() - pingTracker.sentAt;
                pingTracker.pending = false;

                uint16_t rtUs = (roundtripUs > 65535) ? 65535 : (uint16_t)roundtripUs;
                uint8_t pongPayload[3] = {robotId, (uint8_t)(rtUs & 0xFF), (uint8_t)(rtUs >> 8)};
                uint8_t pongFrame[8];
                buildFrame(pongFrame, MSG_PONG, pongPayload, 3);
                Serial.write(pongFrame, 8);
            }
            break;
        }

        default:
            Serial.write(data, len);
            break;
    }
}

// ─── PC-Kommandos verarbeiten ────────────────────────────────

static void processSerialPacket(const uint8_t* data, uint8_t len) {
    if (!validateFrame(data, len)) return;

    uint8_t type = data[2];

    switch (type) {
        case MSG_SWARM:
            esp_now_send(broadcastMAC, data, len);
            break;

        case MSG_PING: {
            uint8_t payloadLen = data[3];
            if (payloadLen >= 1) {
                pingTracker.robotId = data[4];
                pingTracker.sentAt = micros();
                pingTracker.pending = true;
                esp_now_send(broadcastMAC, data, len);
            }
            break;
        }

        default:
            esp_now_send(broadcastMAC, data, len);
            break;
    }
}

// ─── Timing ──────────────────────────────────────────────────

static unsigned long ledOffAt    = 0;
static unsigned long lastSerial  = 0;

// ─── Setup ───────────────────────────────────────────────────

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    Serial.begin(921600);
    unsigned long t = millis();
    while (!Serial && millis() - t < 2000) delay(10);

    Serial.println("═══ Swarm Dongle ═══");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.print("MAC: "); Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERR] ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb([](const wifi_tx_info_t*, esp_now_send_status_t status) {
        if (status == ESP_NOW_SEND_SUCCESS) {
            digitalWrite(LED_PIN, LED_ON);
            ledOffAt = millis() + 20;
        }
    });

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastMAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("Ready.");
}

// ─── Loop ────────────────────────────────────────────────────

void loop() {
    unsigned long now = millis();

    // ESP-NOW Empfang
    if (hasData) {
        hasData = false;
        routeIncoming(rxBuf, rxLen, rxMAC);
    }

    // Serial vom PC
    while (Serial.available()) {
        if (serialLen < sizeof(serialBuf))
            serialBuf[serialLen++] = Serial.read();
        else
            Serial.read();
        lastByteAt = now;
        lastSerial = now;
    }

    if (serialLen > 0 && (now - lastByteAt) >= SERIAL_TIMEOUT_MS) {
        processSerialPacket(serialBuf, serialLen);
        serialLen = 0;
    }

    // Dongle-Watchdog: Stop wenn PC weg
    if (lastSerial > 0 && now - lastSerial > 5000) {
        uint8_t payload[MAX_ROBOTS * 3] = {};
        for (int i = 0; i < MAX_ROBOTS; i++) payload[i * 3] = (uint8_t)i;
        uint8_t frame[5 + MAX_ROBOTS * 3];
        buildFrame(frame, MSG_SWARM, payload, MAX_ROBOTS * 3);
        esp_now_send(broadcastMAC, frame, sizeof(frame));
        lastSerial = now;
    }

    // LED
    if (ledOffAt && now >= ledOffAt) { digitalWrite(LED_PIN, LED_OFF); ledOffAt = 0; }
}
