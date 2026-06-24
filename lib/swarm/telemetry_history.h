// telemetry_history.h — generic per-robot, per-metric rolling sample history.
//
// Decoupled from rendering: call sample() once per poll tick with the latest
// SwarmClient::RobotState, then read back fixed-size windows via get() for
// whatever UI (ASCII plot, SFML, web) wants to draw them.

#pragma once

#include "SwarmClient.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <unordered_map>
#include <vector>

enum class Metric : size_t { Latency = 0, Battery, MotorL, MotorR, Count };

template <size_t N>
class RingBuffer {
public:
    void push(float v) {
        buf_[head_] = v;
        head_ = (head_ + 1) % N;
        if (count_ < N) count_++;
    }

    size_t size() const { return count_; }

    // index 0 = oldest sample currently held
    float at(size_t i) const {
        size_t start = (head_ + N - count_) % N;
        return buf_[(start + i) % N];
    }

    float latest() const { return count_ == 0 ? 0.0f : at(count_ - 1); }

    float min() const {
        float m = 0.0f;
        for (size_t i = 0; i < count_; i++) m = (i == 0) ? at(i) : std::min(m, at(i));
        return m;
    }

    float max() const {
        float m = 0.0f;
        for (size_t i = 0; i < count_; i++) m = (i == 0) ? at(i) : std::max(m, at(i));
        return m;
    }

    float avg() const {
        if (count_ == 0) return 0.0f;
        float sum = 0.0f;
        for (size_t i = 0; i < count_; i++) sum += at(i);
        return sum / (float)count_;
    }

private:
    std::array<float, N> buf_{};
    size_t head_  = 0;
    size_t count_ = 0;
};

class TelemetryHistory {
public:
    static constexpr size_t kWindow = 60;
    using Buffer = RingBuffer<kWindow>;

    // Call once per poll tick (e.g. right after swarm.poll()) for every
    // currently-known robot. Only pushes a new sample when the robot's
    // lastSeen timestamp has advanced, so repeated calls within one tick
    // (or robots with no fresh data) don't pollute the window.
    void sample(uint8_t robotId, const SwarmClient::RobotState& s) {
        auto& entry = perRobot_[robotId];
        if (s.lastSeen == entry.lastSampledAt) return;
        entry.lastSampledAt = s.lastSeen;

        entry.buffers[(size_t)Metric::Latency].push((float)s.latencyUs);
        entry.buffers[(size_t)Metric::Battery].push((float)s.battery);
        entry.buffers[(size_t)Metric::MotorL].push((float)s.motorL);
        entry.buffers[(size_t)Metric::MotorR].push((float)s.motorR);
    }

    const Buffer& get(uint8_t robotId, Metric m) const {
        static const Buffer empty;
        auto it = perRobot_.find(robotId);
        if (it == perRobot_.end()) return empty;
        return it->second.buffers[(size_t)m];
    }

    std::vector<uint8_t> knownIds() const {
        std::vector<uint8_t> ids;
        for (auto& [id, _] : perRobot_) ids.push_back(id);
        return ids;
    }

private:
    struct PerRobot {
        std::array<Buffer, (size_t)Metric::Count> buffers;
        std::chrono::steady_clock::time_point lastSampledAt{};
    };

    std::unordered_map<uint8_t, PerRobot> perRobot_;
};
