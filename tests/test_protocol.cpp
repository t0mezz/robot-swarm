// test_protocol.cpp
// Unit tests for the pure functions in lib/SwarmProtocol/protocol.h
// (crc8, buildFrame, validateFrame, frameSize). No test framework — plain
// asserts with a pass/fail tally, run via `make test` (see tests/Makefile).
//
// CRC-8 test vectors were cross-checked against an independent Python
// implementation of the same poly-0x07, non-reflected CRC-8 (this matches
// the well-known CRC-8/SMBUS check value 0xF4 for "123456789").

#include "../lib/SwarmProtocol/protocol.h"
#include <cstdio>
#include <vector>

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_EQ(actual, expected, msg) \
    do { \
        auto _actual_val   = (actual); \
        auto _expected_val = (expected); \
        if (_actual_val == _expected_val) { \
            g_pass++; \
        } else { \
            g_fail++; \
            std::printf("FAIL %s: expected %lld, got %lld (%s)\n", \
                        __func__, (long long)_expected_val, (long long)_actual_val, msg); \
        } \
    } while (0)

#define EXPECT_TRUE(cond, msg)  EXPECT_EQ((bool)(cond), true, msg)
#define EXPECT_FALSE(cond, msg) EXPECT_EQ((bool)(cond), false, msg)

static void test_crc8_known_vectors() {
    EXPECT_EQ(crc8(nullptr, 0), 0x00, "empty input");

    const uint8_t check[] = {'1','2','3','4','5','6','7','8','9'};
    EXPECT_EQ(crc8(check, sizeof(check)), 0xF4, "CRC-8/SMBUS check value");

    const uint8_t v2[] = {0x01, 0x02};
    EXPECT_EQ(crc8(v2, sizeof(v2)), 0x1B, "two-byte vector");

    const uint8_t v6[] = {0xAA, 0x55, 0x01, 0x02, 0x10, 0x20};
    EXPECT_EQ(crc8(v6, sizeof(v6)), 0xAA, "six-byte vector");
}

static void test_frame_size() {
    EXPECT_EQ(frameSize(0), 5, "payload len 0 -> 5 byte frame");
    EXPECT_EQ(frameSize(2), 7, "payload len 2 -> 7 byte frame");
    EXPECT_EQ(frameSize(40), 45, "payload len 40 -> 45 byte frame");
}

static void test_build_then_validate_roundtrip() {
    const std::vector<std::vector<uint8_t>> payloads = {
        {},
        {0x42},
        {0x01, 0x02},
        {0xAA, 0x55, 0x01, 0x02, 0x10, 0x20},
    };

    for (const auto& payload : payloads) {
        uint8_t frame[64];
        buildFrame(frame, MSG_SPEED, payload.data(), (uint8_t)payload.size());
        uint8_t len = frameSize((uint8_t)payload.size());
        EXPECT_TRUE(validateFrame(frame, len), "freshly built frame validates");
        EXPECT_EQ(frame[2], MSG_SPEED, "type byte preserved");
        EXPECT_EQ(frame[3], (int)payload.size(), "len byte preserved");
    }
}

static void test_validate_rejects_corruption() {
    uint8_t payload[2] = {0x01, 0x02};
    uint8_t frame[7];
    buildFrame(frame, MSG_SPEED, payload, 2);

    EXPECT_TRUE(validateFrame(frame, 7), "uncorrupted frame is valid (sanity check)");

    uint8_t badCrc[7];
    memcpy(badCrc, frame, 7);
    badCrc[6] ^= 0xFF;
    EXPECT_FALSE(validateFrame(badCrc, 7), "flipped CRC byte rejected");

    uint8_t badPayload[7];
    memcpy(badPayload, frame, 7);
    badPayload[4] ^= 0xFF;
    EXPECT_FALSE(validateFrame(badPayload, 7), "flipped payload byte rejected");

    uint8_t badMagic[7];
    memcpy(badMagic, frame, 7);
    badMagic[0] = 0x00;
    EXPECT_FALSE(validateFrame(badMagic, 7), "wrong magic byte rejected");

    EXPECT_FALSE(validateFrame(frame, 6), "truncated buffer rejected");
    EXPECT_FALSE(validateFrame(frame, 4), "buffer shorter than header rejected");
}

int main() {
    test_crc8_known_vectors();
    test_frame_size();
    test_build_then_validate_roundtrip();
    test_validate_rejects_corruption();

    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
