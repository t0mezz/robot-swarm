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
// ESP32-S3: adjust LED_PIN to match your board (GPIO 2 = common default;
// ESP32-S3-DevKitC-1 has a WS2812 on GPIO 48 — replace with NeoPixel if needed)
#define LED_PIN           2
#define LED_ON            LOW
#define LED_OFF           HIGH
#define ESPNOW_CHANNEL    1

// ─── Roboter-Registry ────────────────────────────────────────

struct Robot {
    bool          active;
    uint8_t       mac[6];
    unsigned long lastSeen;
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

// ─── ESP-NOW Send-Queue ──────────────────────────────────────
// esp_now_send() must not be called while a previous send is in flight.
// We queue packets and dispatch the next one from the send callback.

#define TX_QUEUE_SIZE 8
struct TxPacket { uint8_t data[250]; uint8_t len; bool isPing; };

static TxPacket      txQueue[TX_QUEUE_SIZE];
static uint8_t       txHead     = 0;
static uint8_t       txTail     = 0;
static volatile bool txBusy     = false;
static portMUX_TYPE  txMux      = portMUX_INITIALIZER_UNLOCKED;

static void txDispatchNext() {
    // Fix: hold spinlock while touching txHead/txTail so Core 1 (enqueueSend)
    // sees a consistent queue state and the memcpy-before-txTail-update ordering
    // is enforced across both cores.
    portENTER_CRITICAL(&txMux);
    if (txHead == txTail) { txBusy = false; portEXIT_CRITICAL(&txMux); return; }
    uint8_t slot = txHead;
    txHead = (txHead + 1) % TX_QUEUE_SIZE;
    portEXIT_CRITICAL(&txMux);
    // esp_now_send copies data internally before returning, so slot can be
    // reused by enqueueSend immediately after this call.
    if (txQueue[slot].isPing) pingTracker.sentAt = micros();
    if (esp_now_send(broadcastMAC, txQueue[slot].data, txQueue[slot].len) != ESP_OK) {
        // Send failed — callback will NOT fire, so release the lock immediately
        txBusy = false;
    }
}

static void enqueueSend(const uint8_t* data, uint8_t len, bool isPing = false) {
    if (!txBusy) {
        txBusy = true;
        if (isPing) pingTracker.sentAt = micros();
        if (esp_now_send(broadcastMAC, data, len) != ESP_OK) {
            txBusy = false;  // callback won't fire on error — release immediately
        }
        return;
    }
    portENTER_CRITICAL(&txMux);
    uint8_t next = (txTail + 1) % TX_QUEUE_SIZE;
    if (next != txHead) {
        memcpy(txQueue[txTail].data, data, len);
        txQueue[txTail].len    = len;
        txQueue[txTail].isPing = isPing;
        txTail = next;
    } else if (!isPing && data[2] == MSG_SWARM) {
        // Fix: queue full but this is a SWARM (latest-wins) — overwrite the most
        // recently queued slot if it is also a SWARM, so the freshest direction
        // command always wins over a stale one.
        uint8_t last = (txTail - 1 + TX_QUEUE_SIZE) % TX_QUEUE_SIZE;
        if (txQueue[last].data[2] == MSG_SWARM) {
            memcpy(txQueue[last].data, data, len);
            txQueue[last].len = len;
        }
        // else: non-SWARM at tail (e.g. ping/pong) — drop new SWARM to preserve it
    }
    // else: queue full, non-SWARM packet — drop
    portEXIT_CRITICAL(&txMux);
}

// ─── Empfangs-Buffer ─────────────────────────────────────────
// The dongle is esp32dev (dual-core): onReceive fires on Core 0 (WiFi task)
// while loop() runs on Core 1.  Protect shared state with a spinlock.

static uint8_t          rxBuf[250];
static uint8_t          rxLen    = 0;
static uint8_t          rxMAC[6] = {};
static bool             hasData  = false;
static portMUX_TYPE     rxMux    = portMUX_INITIALIZER_UNLOCKED;

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (len <= 0 || len > (int)sizeof(rxBuf)) return;
    portENTER_CRITICAL(&rxMux);
    memcpy(rxBuf, data, len);
    rxLen = (uint8_t)len;
    memcpy(rxMAC, mac, 6);
    hasData = true;
    portEXIT_CRITICAL(&rxMux);
}

// ─── Serial vom PC ───────────────────────────────────────────

// State-machine parser: process frames as soon as they are complete.
// Never accumulates more than one frame; no timeout needed.
static uint8_t serialBuf[256];   // one max-size frame (5 + MAX_ROBOTS*3 + 1)
static uint8_t serialLen = 0;

// ─── Eingehende ESP-NOW Pakete routen ────────────────────────

// Non-blocking Serial write: HardwareSerial blocks loop() if the TX buffer is full.
// Guard every write so the dongle is never stalled waiting for the hub to drain bytes.
static inline void serialWrite(const uint8_t* data, uint8_t len) {
    if (Serial.availableForWrite() >= len) Serial.write(data, len);
}

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

            // ACK
            uint8_t ackPayload[1] = {robotId};
            uint8_t ackFrame[6];
            buildFrame(ackFrame, MSG_ANNOUNCE_ACK, ackPayload, 1);
            enqueueSend(ackFrame, 6);

            // An PC: [id][mac x6]
            uint8_t regPayload[7] = {robotId};
            memcpy(&regPayload[1], mac, 6);
            uint8_t regFrame[12];
            buildFrame(regFrame, MSG_ANNOUNCE, regPayload, 7);
            serialWrite(regFrame, 12);

            digitalWrite(LED_PIN, LED_ON);
            break;
        }

        case MSG_TELEMETRY: {
            if (payloadLen >= 1 && payload[0] < MAX_ROBOTS) {
                robots[payload[0]].lastSeen = millis();
            }
            // 1:1 an PC
            serialWrite(data, len);
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
                serialWrite(pongFrame, 8);
                enqueueSend(pongFrame, 8);   // also broadcast RTT back to robot
            }
            break;
        }

        default:
            serialWrite(data, len);
            break;
    }
}

// ─── PC-Kommandos verarbeiten ────────────────────────────────

static void processSerialPacket(const uint8_t* data, uint8_t len) {
    if (!validateFrame(data, len)) return;

    uint8_t type = data[2];

    switch (type) {
        case MSG_SWARM:
            enqueueSend(data, len);
            break;

        case MSG_PING: {
            uint8_t payloadLen = data[3];
            if (payloadLen >= 1) {
                pingTracker.robotId = data[4];
                pingTracker.pending = true;
                enqueueSend(data, len, /*isPing=*/true);
            }
            break;
        }

        default:
            enqueueSend(data, len);
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

    Serial.setRxBufferSize(512);   // MSG_SWARM frame is 101 bytes; default 128 is marginal
    Serial.setTxBufferSize(512);   // enough headroom for bursts of telemetry + pong frames
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && millis() - t < 2000) delay(10);

    Serial.println("═══ Swarm Dongle ═══");

    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.print("MAC: "); Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERR] ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb([](const uint8_t*, esp_now_send_status_t status) {
        if (status == ESP_NOW_SEND_SUCCESS) {
            digitalWrite(LED_PIN, LED_ON);
            ledOffAt = millis() + 20;
        }
        txDispatchNext();
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

    // ESP-NOW Empfang — copy under spinlock, then process outside it
    uint8_t localBuf[250];
    uint8_t localLen = 0;
    uint8_t localMAC[6];
    bool    got = false;
    portENTER_CRITICAL(&rxMux);
    if (hasData) {
        localLen = rxLen;
        memcpy(localBuf, rxBuf, localLen);
        memcpy(localMAC, rxMAC, 6);
        hasData = false;
        got = true;
    }
    portEXIT_CRITICAL(&rxMux);
    if (got) routeIncoming(localBuf, localLen, localMAC);

    // Serial vom PC — byte-by-byte frame state machine
    while (Serial.available()) {
        uint8_t b = Serial.read();
        lastSerial = now;

        // Hunt for frame header
        if (serialLen == 0) {
            if (b != MAGIC_0) continue;
        } else if (serialLen == 1) {
            if (b != MAGIC_1) { serialLen = 0; continue; }
        }

        serialBuf[serialLen++] = b;

        if (serialLen < 4) continue;  // need header to know length

        uint8_t needed = frameSize(serialBuf[3]);
        if (needed > sizeof(serialBuf)) { serialLen = 0; continue; }  // oversized, reset

        if (serialLen >= needed) {
            processSerialPacket(serialBuf, needed);
            serialLen = 0;  // ready for next frame
        }
    }

    // Dongle-Watchdog: Stop wenn PC weg
    if (lastSerial > 0 && now - lastSerial > 5000) {
        uint8_t payload[MAX_ROBOTS * 3] = {};
        for (int i = 0; i < MAX_ROBOTS; i++) payload[i * 3] = (uint8_t)i;
        uint8_t frame[5 + MAX_ROBOTS * 3];
        buildFrame(frame, MSG_SWARM, payload, MAX_ROBOTS * 3);
        enqueueSend(frame, sizeof(frame));
        lastSerial = now;
    }

    // LED
    if (ledOffAt && now >= ledOffAt) { digitalWrite(LED_PIN, LED_OFF); ledOffAt = 0; }
}
