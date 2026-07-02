// swarm_dongle.ino
// Swarm Dongle — ESP32 am Controller-PC
// Nutzt: protocol.h
//
// Arduino ESP32 Core 3.x

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>
#include "protocol.h"
#include "hardware.h"

// ─── Hardware ────────────────────────────────────────────────
// ESP32-S3-DevKitC-1 N16R8: WS2812 on GPIO 48
#define LED_PIN           48
#define ESPNOW_CHANNEL    1

static Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

static inline void ledOn()  { led.setPixelColor(0, led.Color(0, 20, 0)); led.show(); }
static inline void ledOff() { led.setPixelColor(0, 0);                   led.show(); }

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
struct TxPacket { uint8_t data[250]; uint8_t len; bool isPing; uint8_t dest[6]; };

static TxPacket      txQueue[TX_QUEUE_SIZE];
static uint8_t       txHead     = 0;
static uint8_t       txTail     = 0;
static volatile bool txBusy     = false;
static portMUX_TYPE  txMux      = portMUX_INITIALIZER_UNLOCKED;

// Register a robot MAC as an ESP-NOW peer so robot-targeted frames (ACK, ping,
// pong echo) go unicast — MAC-layer ACK+retries instead of fire-and-forget
// broadcast, and the other robots don't burn airtime/CPU on them. Returns false
// if the peer can't be registered (e.g. peer table full: ESP-NOW allows ~20
// unencrypted peers, MAX_ROBOTS is 32) — callers then fall back to broadcast.
static bool ensurePeer(const uint8_t* mac) {
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    return esp_now_add_peer(&peer) == ESP_OK;
}

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
    if (esp_now_send(txQueue[slot].dest, txQueue[slot].data, txQueue[slot].len) != ESP_OK) {
        // Send failed — callback will NOT fire, so release the lock immediately
        txBusy = false;
    }
}

// dest == nullptr → broadcast. Unicast dests must already be registered via
// ensurePeer(); on esp_now_send failure the frame is lost either way, so the
// caller-side fallback is choosing broadcast when ensurePeer() fails.
static void enqueueSend(const uint8_t* data, uint8_t len, bool isPing = false,
                        const uint8_t* dest = nullptr) {
    if (dest == nullptr) dest = broadcastMAC;
    if (!txBusy) {
        txBusy = true;
        if (isPing) pingTracker.sentAt = micros();
        if (esp_now_send(dest, data, len) != ESP_OK) {
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
        memcpy(txQueue[txTail].dest, dest, 6);
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
//
// SPSC ring buffer (mirrors src/receiver/main.cpp): a single-slot buffer
// here would silently overwrite/drop a packet (e.g. MSG_TELEMETRY) if a
// second ESP-NOW packet arrives before loop() drains the first — most
// likely during the idle-throttle vTaskDelay(10) below. When the queue is
// full the oldest slot is dropped so the consumer keeps up with the
// freshest traffic.
static const uint8_t RX_QUEUE_SIZE = 4;
struct RxSlot { uint8_t data[250]; uint8_t len; uint8_t mac[6]; };
static RxSlot        rxQueue[RX_QUEUE_SIZE];
static uint8_t       rxHead = 0;   // next write index (producer, Core 0)
static uint8_t       rxTail = 0;   // next read  index (consumer, Core 1)
static portMUX_TYPE  rxMux  = portMUX_INITIALIZER_UNLOCKED;
static uint32_t      diagRxDropCount = 0;  // RX queue overflow drops

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (len <= 0 || len > (int)sizeof(rxQueue[0].data)) return;
    portENTER_CRITICAL(&rxMux);
    uint8_t used = (rxHead - rxTail + RX_QUEUE_SIZE) % RX_QUEUE_SIZE;
    if (used == RX_QUEUE_SIZE - 1) {
        rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
        diagRxDropCount++;
    }
    memcpy(rxQueue[rxHead].data, data, len);
    rxQueue[rxHead].len = (uint8_t)len;
    memcpy(rxQueue[rxHead].mac, mac, 6);
    rxHead = (rxHead + 1) % RX_QUEUE_SIZE;
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
// If the TX buffer is full the frame is dropped — diagSerialDropCount tracks how
// often this happens so a saturated PC-side link is visible in diagnostics.
static uint32_t diagSerialDropCount = 0;

static inline void serialWrite(const uint8_t* data, uint8_t len) {
    if (Serial.availableForWrite() >= len) Serial.write(data, len);
    else diagSerialDropCount++;
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

            // ACK — unicast so the MAC layer retries it; a lost broadcast ACK
            // left the robot re-announcing (and without telemetry) for another
            // 500ms round.
            bool unicast = ensurePeer(mac);
            uint8_t ackPayload[1] = {robotId};
            uint8_t ackFrame[6];
            buildFrame(ackFrame, MSG_ANNOUNCE_ACK, ackPayload, 1);
            enqueueSend(ackFrame, 6, false, unicast ? mac : nullptr);

            // An PC: [id][mac x6]
            uint8_t regPayload[7] = {robotId};
            memcpy(&regPayload[1], mac, 6);
            uint8_t regFrame[12];
            buildFrame(regFrame, MSG_ANNOUNCE, regPayload, 7);
            serialWrite(regFrame, 12);

            ledOn();
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
                // RTT back to the robot — unicast to the sender we just heard from
                enqueueSend(pongFrame, 8, false, ensurePeer(mac) ? mac : nullptr);
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
                uint8_t targetId = data[4];
                pingTracker.robotId = targetId;
                pingTracker.pending = true;
                // Unicast to the target when its MAC is registered (0xFF = all →
                // broadcast). Broadcast pings woke every robot and had no MAC
                // retries, which showed up as pong gaps and RTT jitter.
                const uint8_t* dest = nullptr;
                if (targetId < MAX_ROBOTS && robots[targetId].active &&
                    ensurePeer(robots[targetId].mac)) {
                    dest = robots[targetId].mac;
                }
                enqueueSend(data, len, /*isPing=*/true, dest);
            }
            break;
        }

        default:
            enqueueSend(data, len);
            break;
    }
}

// ─── Timing ──────────────────────────────────────────────────

static unsigned long ledOffAt     = 0;
static unsigned long lastSerial   = 0;
static unsigned long lastDiagReport = 0;

// ─── Idle Throttle ───────────────────────────────────────────

static bool anyRobotActive() {
    for (int i = 0; i < MAX_ROBOTS; i++) if (robots[i].active) return true;
    return false;
}


// ─── Setup ───────────────────────────────────────────────────

void setup() {
    led.begin();
    ledOff();

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
            ledOn();
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

    // ESP-NOW Empfang — drain the whole ring buffer, copying one slot at a
    // time under the spinlock so routeIncoming() (which can call
    // serialWrite/enqueueSend) never runs while the lock is held.
    while (true) {
        uint8_t localBuf[250];
        uint8_t localLen = 0;
        uint8_t localMAC[6];
        bool    got = false;
        portENTER_CRITICAL(&rxMux);
        if (rxHead != rxTail) {
            localLen = rxQueue[rxTail].len;
            memcpy(localBuf, rxQueue[rxTail].data, localLen);
            memcpy(localMAC, rxQueue[rxTail].mac, 6);
            rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
            got = true;
        }
        portEXIT_CRITICAL(&rxMux);
        if (!got) break;
        routeIncoming(localBuf, localLen, localMAC);
    }

    // Expire robots that haven't sent anything in ROBOT_EXPIRY_MS.
    // This lets the idle throttle and WiFi PS kick back in when all robots
    // drop off (power-off, out of range) without requiring a dongle reboot.
    for (int i = 0; i < MAX_ROBOTS; i++) {
        if (robots[i].active && now - robots[i].lastSeen > ROBOT_EXPIRY_MS) {
            robots[i].active = false;
            esp_now_del_peer(robots[i].mac);  // free the peer slot (~20 available)
        }
    }

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
    if (ledOffAt && now >= ledOffAt) { ledOff(); ledOffAt = 0; }

    // Periodic diagnostics (every 2s) — only when a USB terminal is connected
    // so the Serial write overhead is zero when running standalone.
    if (now - lastDiagReport >= 2000) {
        if (Serial) {
            Serial.printf("[DIAG] rx_drops=%lu serial_drops=%lu\n",
                          (unsigned long)diagRxDropCount, (unsigned long)diagSerialDropCount);
        }
        diagRxDropCount     = 0;
        diagSerialDropCount = 0;
        lastDiagReport      = now;
    }

    // Idle management: throttle the loop when no robots are active to reduce
    // CPU load.  WiFi PS is intentionally kept at WIFI_PS_NONE at all times —
    // WIFI_PS_MIN_MODEM puts the radio to sleep between beacon intervals and
    // causes incoming ESP-NOW announce packets to be dropped, preventing
    // robots from ever re-registering after the registry expires.
    if (!anyRobotActive()) vTaskDelay(pdMS_TO_TICKS(10));
}
