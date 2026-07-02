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
#include <Adafruit_NeoPixel.h>
#include "protocol.h"
#include "hardware.h"
#include "debug_protocol.h"

// ─── RGB LED (WS2812, GRB, GPIO 8) ──────────────────────────
static Adafruit_NeoPixel rgbLed(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// HSV to RGB helper (hue 0-255, sat 0-255, val 0-255)
static uint32_t hsvToColor(uint8_t h, uint8_t s, uint8_t v) {
    uint8_t r, g, b;
    if (s == 0) { r = g = b = v; }
    else {
        uint8_t region = h / 43;
        uint8_t rem    = (h - region * 43) * 6;
        uint8_t p = (v * (255 - s)) >> 8;
        uint8_t q = (v * (255 - ((s * rem) >> 8))) >> 8;
        uint8_t t2 = (v * (255 - ((s * (255 - rem)) >> 8))) >> 8;
        switch (region) {
            case 0: r=v; g=t2; b=p; break;
            case 1: r=q; g=v;  b=p; break;
            case 2: r=p; g=v;  b=t2; break;
            case 3: r=p; g=q;  b=v; break;
            case 4: r=t2; g=p; b=v; break;
            default: r=v; g=p;  b=q; break;
        }
    }
    return rgbLed.Color(r, g, b);
}

static void updateRgbLed(unsigned long now, bool announcing) {
    static unsigned long lastUpdate = 0;
    static uint8_t hue = 0;
    if (now - lastUpdate < 40) return;
    lastUpdate = now;

    // TEMP: maximum white for ArUco readability testing
    (void)announcing; (void)hue;
    rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));
    rgbLed.show();
}

// ═══════════════════════════════════════════════════════════════
// Transport Layer — für späteren Wechsel auf WiFi STA
// ═══════════════════════════════════════════════════════════════

namespace Transport {
    static uint8_t dongleMAC[6] = {0};
    static bool    dongleKnown  = false;
    static uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    // Pending registration — set by onReceive(), consumed by loop().
    // esp_now_add_peer() must NOT be called from within the ESP-NOW receive
    // callback: it acquires internal driver locks that are already held during
    // the callback, leaving the receive path degraded (verified on C3).
    static volatile bool pendingRegister = false;
    static uint8_t       pendingMAC[6]   = {};

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

// ── Receive queue (SPSC ring buffer, Core-0 producer / Core-1 consumer) ──────
// When the queue is full the oldest slot is silently dropped so that the
// consumer always works through the most-recent commands without stale lag.
static const uint8_t RX_QUEUE_SIZE = 4;
struct RxSlot { uint8_t data[250]; uint8_t len; };
static RxSlot       rxQueue[RX_QUEUE_SIZE];
static uint8_t      rxHead   = 0;   // next write index  (producer, Core 0)
static uint8_t      rxTail   = 0;   // next read  index  (consumer, Core 1)
static portMUX_TYPE rxMux    = portMUX_INITIALIZER_UNLOCKED;

static unsigned long lastAnnounce      = 0;
static unsigned long lastSwarmReceived = 0;
// Any frame that only the dongle emits (SWARM/ACK/PING/PONG) proves the link is
// alive. The announce timeout keys off this instead of lastSwarmReceived: when no
// tool is driving motors there are no SWARM frames at all (the hub's round-robin
// pings keep the dongle's serial watchdog quiet), and keying off SWARM alone made
// every idle robot flap back to ANNOUNCING every ANNOUNCE_TIMEOUT_MS.
static unsigned long lastDongleSeen    = 0;
static unsigned long lastSpeedApplied  = 0;
static unsigned long lastDebugUpdate   = 0;
static unsigned long lastUartSpeed     = 0;  // last time uart_send_speed was called
static unsigned long bootTime          = 0;
static bool          debugRegistered   = false;

// Diagnostics
static uint32_t      diagSwarmCount    = 0;  // MSG_SWARM packets processed
static uint32_t      diagZeroCount     = 0;  // uart_send_speed(0,0) calls
static uint32_t      diagDropCount     = 0;  // RX queue overflow drops
static uint32_t      diagZeroCmdCount  = 0;  // motor-to-zero transitions
static unsigned long lastDiagReport    = 0;

static int8_t   currentMotorL = 0;
static int8_t   currentMotorR = 0;
static uint16_t lastLatencyUs = 0;
static uint8_t  statusFlags   = STATUS_ANNOUNCING;
static uint8_t  lastBattery   = 0;  // from MSG_METRICS, uint8 0-255 -> 0-5V

// ═══════════════════════════════════════════════════════════════
// ESP-NOW Callback
// ═══════════════════════════════════════════════════════════════

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (len <= 0 || len > 250) return;

    portENTER_CRITICAL(&rxMux);
    // If full, drop the oldest slot to make room for the freshest command.
    uint8_t used = (rxHead - rxTail + RX_QUEUE_SIZE) % RX_QUEUE_SIZE;
    if (used == RX_QUEUE_SIZE - 1) {
        rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
        diagDropCount++;
    }
    memcpy(rxQueue[rxHead].data, data, len);
    rxQueue[rxHead].len = (uint8_t)len;
    rxHead = (rxHead + 1) % RX_QUEUE_SIZE;
    portEXIT_CRITICAL(&rxMux);

    // Do NOT call esp_now_add_peer() here — unsafe inside the receive callback.
    // Store the MAC and let loop() register the peer.
    if (!Transport::dongleKnown && !Transport::pendingRegister) {
        memcpy(Transport::pendingMAC, mac, 6);
        Transport::pendingRegister = true;
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
    if (Serial) Serial.printf("[ANN] Announce sent (ID=%d)\n", ROBOT_ID);
}

static void uart_send_speed(int8_t left, int8_t right);  // forward declaration
static void uart_send_robot_id();                         // forward declaration

// ═══════════════════════════════════════════════════════════════
// Paket-Handler
// ═══════════════════════════════════════════════════════════════

static void processIncoming(const uint8_t* data, uint8_t len) {
    if (!validateFrame(data, len)) return;

    uint8_t type       = data[2];
    uint8_t payloadLen = data[3];
    const uint8_t* payload = &data[4];

    // Link liveness: these types are only ever sent by the dongle (robot-to-robot
    // broadcasts are ANNOUNCE/TELEMETRY), so any of them — even addressed to
    // another robot — means the dongle is reachable.
    if (type == MSG_SWARM || type == MSG_ANNOUNCE_ACK ||
        type == MSG_PING  || type == MSG_PONG) {
        lastDongleSeen = millis();
    }

    switch (type) {
        case MSG_ANNOUNCE_ACK: {
            if (state != State::ANNOUNCING) break;
            if (payloadLen < 1 || payload[0] != ROBOT_ID) break;
            state = State::ACTIVE;
            statusFlags &= ~STATUS_ANNOUNCING;
            lastSwarmReceived = millis();
            if (Serial) Serial.printf("[ACK] Registered as Robot %d\n", ROBOT_ID);
            break;
        }

        // LED flashen und uart senden
        case MSG_SWARM: {
            lastSwarmReceived = millis();
            if (state == State::ANNOUNCING) break;
            diagSwarmCount++;
            // Eigene ID suchen
            uint8_t entries = payloadLen / 3;
            for (uint8_t i = 0; i < entries; i++) {
                if (payload[i * 3] == ROBOT_ID) {
                    int8_t newL = (int8_t)payload[i * 3 + 1];
                    int8_t newR = (int8_t)payload[i * 3 + 2];
                    lastSpeedApplied = millis();
                    if (newL == 0 && newR == 0 && (currentMotorL != 0 || currentMotorR != 0)) {
                        diagZeroCmdCount++;
                    }
                    if (newL != currentMotorL || newR != currentMotorR) {
                        // Value changed — send immediately
                        currentMotorL = newL;
                        currentMotorR = newR;
                        uart_send_speed(currentMotorL, currentMotorR);
                        lastUartSpeed = millis();
                    }
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
    if (data[2] == MSG_DEBUG_PING) {
        if (Serial) Serial.println("[DBG] Debug screen ping received, registering fields...");
        uart_send_robot_id();
        DebugScreen::registerAllFields(UART);
        debugRegistered = true;
    } else if (data[2] == MSG_DEBUG) {
        // Debug-Log vom RP2040: [field_id][value_type][data...]
        // robot_id voranstellen und 1:1 an den Dongle weiterreichen (der Dongle
        // leitet unbekannte Typen per default an den PC weiter -> swarm_terminal).
        uint8_t payloadLen = data[3];
        if (payloadLen > 40) return;   // hält den Frame im UART/ESP-NOW-Budget
        uint8_t outPayload[42];
        outPayload[0] = ROBOT_ID;
        memcpy(&outPayload[1], &data[4], payloadLen);
        uint8_t frame[48];
        buildFrame(frame, MSG_DEBUG, outPayload, payloadLen + 1);
        Transport::sendToDongle(frame, frameSize(payloadLen + 1));
    } else if (data[2] == MSG_METRICS) {
        // Batteriespannung vom RP2040: [battery] (uint8, 40mV/LSB)
        if (data[3] >= 1) {
            lastBattery  = data[4];
            statusFlags |= STATUS_BAT_VALID;
        }
    }
}

static void pollUart() {
    while (UART.available()) {
        uint8_t b = (uint8_t)UART.read();
        if (uartRxIdx == 0 && b != MAGIC_0) continue;
        if (uartRxIdx == 1 && b != MAGIC_1) {
            // Resync: a stray 0xAA followed by the real frame start ("AA AA 55")
            // must keep this byte as a candidate MAGIC_0, not discard it.
            uartRxIdx = (b == MAGIC_0) ? 1 : 0;
            continue;
        }
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
    if (left == 0 && right == 0) diagZeroCount++;
    uint8_t payload[2] = {(uint8_t)left, (uint8_t)right};
    uint8_t frame[7];
    buildFrame(frame, MSG_SPEED, payload, 2);
    if (UART.availableForWrite() >= 7) UART.write(frame, 7);
}

static void uart_send_robot_id() {
    uint8_t payload[1] = {ROBOT_ID};
    uint8_t frame[6];
    buildFrame(frame, MSG_ROBOT_ID, payload, 1);
    if (UART.availableForWrite() >= 6) UART.write(frame, 6);
}

// ═══════════════════════════════════════════════════════════════
// Telemetrie (TDMA)
// ═══════════════════════════════════════════════════════════════

static void sendTelemetry() {
    uint16_t uptime = (uint16_t)((millis() - bootTime) / 1000);

    uint8_t payload[7] = {
        ROBOT_ID,
        lastBattery,
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
    // Use a free-running millis() cycle rather than anchoring to lastSwarmReceived.
    // Anchoring to lastSwarmReceived caused the slot to never be reached when SWARM
    // packets arrive faster than the slot width (e.g. 50ms controller rate vs 100ms
    // slot): each packet reset the cycle to 0, so robots with ID > 0 could never
    // reach their slot offset and never sent telemetry.
    if (lastDongleSeen == 0) return false;  // don't send before first dongle contact
    unsigned long cycleDuration = (unsigned long)MAX_ROBOTS * TDMA_SLOT_MS;
    unsigned long cyclePos  = millis() % cycleDuration;
    unsigned long slotStart = (unsigned long)ROBOT_ID * TDMA_SLOT_MS;
    return (cyclePos >= slotStart && cyclePos < slotStart + TDMA_SLOT_MS);
}

// ═══════════════════════════════════════════════════════════════
// Watchdog
// ═══════════════════════════════════════════════════════════════

static void checkWatchdog() {
    if (lastSpeedApplied == 0) return;
    unsigned long gap = millis() - lastSpeedApplied;
    if (gap > WATCHDOG_TIMEOUT_MS) {
        if (currentMotorL != 0 || currentMotorR != 0) {
            if (Serial) Serial.printf("[WDG] Fired! gap=%lums L=%d R=%d swarm_rx=%lu\n",
                                      gap, currentMotorL, currentMotorR,
                                      millis() - lastSwarmReceived);
            currentMotorL = currentMotorR = 0;
            uart_send_speed(0, 0);
            lastUartSpeed = millis();
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Setup & Loop
// ═══════════════════════════════════════════════════════════════

void setup() {
    bootTime = millis();
    rgbLed.begin();
    rgbLed.setPixelColor(0, 0);
    rgbLed.show();

    UART.setTxBufferSize(512);
    UART.setRxBufferSize(512);
    UART.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // never block on Serial write — drop bytes if no host connected

    Serial.printf("Swarm Receiver ID:%d\n", ROBOT_ID);
    Serial.print("MAC: "); Serial.println(WiFi.macAddress());

    Transport::init();
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb([](const uint8_t*, esp_now_send_status_t) {});
}

void loop() {
    unsigned long now = millis();

    // Deferred peer registration — must happen outside the ESP-NOW receive callback
    if (Transport::pendingRegister) {
        Transport::pendingRegister = false;
        Transport::registerDongle(Transport::pendingMAC);
    }

    // Drain the receive queue — process every pending packet before moving on.
    // Copying one slot at a time under the spinlock keeps the critical section
    // short (no processIncoming() work held under lock).
    while (true) {
        uint8_t localBuf[250];
        uint8_t localLen;
        portENTER_CRITICAL(&rxMux);
        bool got = (rxHead != rxTail);
        if (got) {
            localLen = rxQueue[rxTail].len;
            memcpy(localBuf, rxQueue[rxTail].data, localLen);
            rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
        }
        portEXIT_CRITICAL(&rxMux);
        if (!got) break;
        processIncoming(localBuf, localLen);
        // Refresh 'now' after each packet: processIncoming() updates lastSwarmReceived
        // via millis(). A stale 'now' could make (now - lastSwarmReceived) wrap and
        // falsely trigger the announce timeout.
        now = millis();
    }

    // State Machine
    if (state == State::ANNOUNCING) {
        if (now - lastAnnounce >= ANNOUNCE_INTERVAL_MS) {
            sendAnnounce();
            lastAnnounce = now;
        }
    } else {
        // Timeout -> re-announce (no frame of any kind from the dongle)
        if (now - lastDongleSeen > ANNOUNCE_TIMEOUT_MS) {
            state = State::ANNOUNCING;
            statusFlags |= STATUS_ANNOUNCING;
            Transport::resetDongle();
            currentMotorL = currentMotorR = 0;
            uart_send_speed(0, 0);
        }

        // TDMA Telemetrie
        static unsigned long lastTelSent = 0;
        const unsigned long cycleDuration = (unsigned long)MAX_ROBOTS * TDMA_SLOT_MS;
        if (isMyTDMASlot() && now - lastTelSent >= cycleDuration - 10) {
            sendTelemetry();
            lastTelSent = now;
        }

        // Re-announce periodically so swarm_terminal can find us after reconnect
        if (now - lastAnnounce >= REANNOUNCE_INTERVAL_MS) {
            sendAnnounce();
            lastAnnounce = now;
        }
    }

    checkWatchdog();

    // Periodic UART keepalive: resend current motor speed every 200ms even if unchanged.
    // Keeps the RP2040 motor driver alive if it has its own watchdog timeout.
    if (state == State::ACTIVE && now - lastUartSpeed >= 200) {
        uart_send_speed(currentMotorL, currentMotorR);
        lastUartSpeed = now;
    }

    pollUart();

    // Debug-Screen
    if (debugRegistered && now - lastDebugUpdate >= DEBUG_UPDATE_MS) {
        const char* st = (state == State::ANNOUNCING) ? "ANN" :
                         (currentMotorL != 0 || currentMotorR != 0) ? "RUN" : "IDL";
        DebugScreen::updateAll(UART, lastLatencyUs, st,
                                currentMotorL, currentMotorR);
        lastDebugUpdate = now;
    }

    // RGB LED
    updateRgbLed(now, state == State::ANNOUNCING);

    // Periodic diagnostics (every 2s) — only when a USB terminal is connected
    // so the Serial write overhead is zero when running standalone.
    if (now - lastDiagReport >= 2000) {
        if (Serial) {
            unsigned long gapMs = (lastSpeedApplied > 0) ? (now - lastSpeedApplied) : 0;
            Serial.printf("[DIAG] swarm_rx=%lu state=%s L=%d R=%d gap=%lums "
                          "swarm=%lu zeros=%lu zeroCmds=%lu drops=%lu\n",
                          now - lastSwarmReceived,
                          (state == State::ANNOUNCING) ? "ANN" : "ACT",
                          currentMotorL, currentMotorR, gapMs,
                          diagSwarmCount, diagZeroCount,
                          diagZeroCmdCount, diagDropCount);
        }
        diagSwarmCount   = 0;
        diagZeroCount    = 0;
        diagZeroCmdCount = 0;
        diagDropCount    = 0;
        lastDiagReport   = now;
    }
}
