// pose_hub.h — share tracked poses between processes over a Unix socket.
//
// The Basler camera admits exactly one application: a second process opening
// it gets 0xE1018006 ("device is controlled by another application"). That is
// why a vision demo and a telemetry dashboard could not run at the same time,
// even though only one of them needs pixels.
//
// This is the same split swarm_hub already applies to the dongle — one owner,
// many readers — for poses instead of serial frames:
//
//   ArucoTracker::open() succeeds  ->  it publishes poses here, for free
//   another tool wants poses       ->  it subscribes, never touching pylon
//
// Deliberately free of OpenCV and pylon includes, so a subscriber links
// neither. That is the whole point: tools that only need to know where the
// robots are should not depend on the imaging stack to find out.
//
// Poses only. Frames (2048x2048 at ~116 fps is ~470 MB/s) would need shared
// memory, not a stream socket — see TODO.md "Webserver / Headless".

#pragma once

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define POSE_HUB_SOCK_PATH "/tmp/vision_hub.sock"

// Mirrors RobotPose (lib/ArucoTracker/aruco_tracker.h) field for field, but
// declared here so this header stays OpenCV-free. ArucoTracker converts on
// publish; subscribers work with this type directly.
struct HubPose {
    int32_t id;
    float   x, y;    // world coords (mm) if the publisher has a homography, else pixels
    float   yaw;     // degrees CCW from world +X
    float   px, py;  // raw pixel centroid — always meaningful, homography or not
};

// One snapshot header, followed by `count` HubPose records. Fixed-layout POD:
// publisher and subscriber are always the same machine and ABI, so there is no
// serialisation beyond a length-prefixed blob, and no CRC — a Unix stream
// socket does not corrupt or reorder.
struct PoseHubHeader {
    uint32_t magic;    // POSE_HUB_MAGIC
    uint32_t seq;      // increments per published snapshot; wraps harmlessly
    float    detFps;
    int32_t  frameW, frameH;
    uint16_t version;
    uint16_t count;
};

static constexpr uint32_t POSE_HUB_MAGIC   = 0x42554856;  // 'VHUB' little-endian
static constexpr uint16_t POSE_HUB_VERSION = 1;
static constexpr uint16_t POSE_HUB_MAX     = 256;         // sanity bound on count

static_assert(sizeof(HubPose) == 24, "HubPose must stay tightly packed");
static_assert(sizeof(PoseHubHeader) == 24, "PoseHubHeader must stay tightly packed");

// ── Publisher ────────────────────────────────────────────────────────────────

class PoseHubPublisher {
public:
    ~PoseHubPublisher() { stop(); }

    // Binds POSE_HUB_SOCK_PATH. Returns false if another publisher already
    // holds it, which in practice cannot happen — only the camera owner
    // publishes and the camera has one owner — but a crashed owner leaves the
    // socket file behind, so a live-probe distinguishes stale from occupied
    // rather than unlinking someone else's socket.
    bool start() {
        if (probeAlive()) return false;
        ::unlink(POSE_HUB_SOCK_PATH);

        listenFd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (listenFd_ < 0) return false;

        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, POSE_HUB_SOCK_PATH, sizeof(addr.sun_path) - 1);
        if (::bind(listenFd_, (sockaddr*)&addr, sizeof(addr)) < 0 ||
            ::listen(listenFd_, 8) < 0) {
            ::close(listenFd_);
            listenFd_ = -1;
            return false;
        }
        setNonBlocking(listenFd_);
        return true;
    }

    bool isRunning() const { return listenFd_ >= 0; }

    // Accepts anyone waiting. Separate from publish() so a caller can cheaply
    // ask "is anyone listening?" before doing the work of building a snapshot
    // — clientCount() cannot answer that until pending connections are taken.
    void poll() { if (listenFd_ >= 0) acceptPending(); }

    size_t clientCount() const { return clients_.size(); }

    // Called from the owner's main loop once per fresh detection. Never blocks
    // and never throws: a stalled subscriber must not be able to slow down a
    // control loop that is driving robots. A subscriber whose buffer is full
    // simply misses this snapshot — poses are absolute state, so the next one
    // it does read is just as good as the one it lost.
    void publish(const std::vector<HubPose>& poses, float detFps, int w, int h) {
        if (listenFd_ < 0) return;
        acceptPending();
        if (clients_.empty()) return;

        uint16_t n = (uint16_t)std::min<size_t>(poses.size(), POSE_HUB_MAX);
        PoseHubHeader hdr{POSE_HUB_MAGIC, seq_++, detFps, w, h, POSE_HUB_VERSION, n};

        buf_.resize(sizeof(hdr) + (size_t)n * sizeof(HubPose));
        std::memcpy(buf_.data(), &hdr, sizeof(hdr));
        if (n) std::memcpy(buf_.data() + sizeof(hdr), poses.data(), (size_t)n * sizeof(HubPose));

        for (size_t i = 0; i < clients_.size();) {
            ssize_t sent = ::send(clients_[i], buf_.data(), buf_.size(),
                                  MSG_NOSIGNAL | MSG_DONTWAIT);
            if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                i++;                      // slow reader: skip this snapshot, keep it
            } else if (sent != (ssize_t)buf_.size()) {
                ::close(clients_[i]);     // gone, or a partial write we can't resume
                clients_.erase(clients_.begin() + (long)i);
            } else {
                i++;
            }
        }
    }

    void stop() {
        for (int fd : clients_) ::close(fd);
        clients_.clear();
        if (listenFd_ >= 0) {
            ::close(listenFd_);
            listenFd_ = -1;
            ::unlink(POSE_HUB_SOCK_PATH);
        }
    }

private:
    static void setNonBlocking(int fd) {
        ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    }

    // Is a publisher actually listening, or is this a leftover socket file?
    static bool probeAlive() {
        int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) return false;
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, POSE_HUB_SOCK_PATH, sizeof(addr.sun_path) - 1);
        bool alive = ::connect(fd, (sockaddr*)&addr, sizeof(addr)) == 0;
        ::close(fd);
        return alive;
    }

    void acceptPending() {
        for (;;) {
            int fd = ::accept(listenFd_, nullptr, nullptr);
            if (fd < 0) return;
            setNonBlocking(fd);
            clients_.push_back(fd);
        }
    }

    int                  listenFd_ = -1;
    std::vector<int>     clients_;
    std::vector<uint8_t> buf_;
    uint32_t             seq_ = 0;
};

// ── Subscriber ───────────────────────────────────────────────────────────────

class PoseHubSubscriber {
public:
    ~PoseHubSubscriber() { disconnect(); }

    bool connect() {
        disconnect();
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) return false;
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, POSE_HUB_SOCK_PATH, sizeof(addr.sun_path) - 1);
        if (::connect(fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
        rx_.clear();
        return true;
    }

    void disconnect() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        poses_.clear();
        detFps_ = 0.f;
    }

    bool isConnected() const { return fd_ >= 0; }

    // Drains everything queued and keeps only the newest snapshot: a
    // subscriber that fell behind wants current positions, not a backlog of
    // stale ones. Returns true if at least one fresh snapshot was parsed.
    bool poll() {
        if (fd_ < 0) return false;

        uint8_t tmp[8192];
        for (;;) {
            ssize_t n = ::read(fd_, tmp, sizeof(tmp));
            if (n > 0) {
                rx_.insert(rx_.end(), tmp, tmp + n);
                if (rx_.size() > (1u << 20)) rx_.clear();  // desynced beyond recovery
                continue;
            }
            if (n == 0) { disconnect(); return false; }     // publisher exited
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            disconnect();
            return false;
        }

        bool fresh = false;
        size_t off = 0;
        while (rx_.size() - off >= sizeof(PoseHubHeader)) {
            PoseHubHeader hdr;
            std::memcpy(&hdr, rx_.data() + off, sizeof(hdr));
            if (hdr.magic != POSE_HUB_MAGIC || hdr.version != POSE_HUB_VERSION ||
                hdr.count > POSE_HUB_MAX) {
                rx_.clear();   // not a stream we understand; drop it
                return fresh;
            }
            size_t need = sizeof(hdr) + (size_t)hdr.count * sizeof(HubPose);
            if (rx_.size() - off < need) break;

            poses_.resize(hdr.count);
            if (hdr.count)
                std::memcpy(poses_.data(), rx_.data() + off + sizeof(hdr),
                            (size_t)hdr.count * sizeof(HubPose));
            detFps_ = hdr.detFps;
            frameW_ = hdr.frameW;
            frameH_ = hdr.frameH;
            off += need;
            fresh = true;
        }
        if (off) rx_.erase(rx_.begin(), rx_.begin() + (long)off);
        return fresh;
    }

    const std::vector<HubPose>& poses() const { return poses_; }
    float detectionFps() const { return detFps_; }
    int   frameWidth()   const { return frameW_; }
    int   frameHeight()  const { return frameH_; }

private:
    int                  fd_ = -1;
    std::vector<uint8_t> rx_;
    std::vector<HubPose> poses_;
    float                detFps_ = 0.f;
    int                  frameW_ = 0, frameH_ = 0;
};
