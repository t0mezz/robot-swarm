// mjpeg_server.h — serve JPEG frames as a browser-viewable video stream.
//
// Built for tools/vision/vision_hub.cpp: the hub owns the camera, and this is
// how its picture reaches a person on the other end of an SSH tunnel
// (`ssh -L 8081:localhost:8081 host`, then http://localhost:8081/).
//
//   GET /            a page holding one <img src="/stream">
//   GET /stream      multipart/x-mixed-replace — what <img> plays as video
//   GET /snapshot.jpg  one fresh frame (curl / scripts); waits for the next publish
//
// MJPEG rather than H.264/WebRTC: every browser plays it from a bare <img>, there
// is no codec negotiation, and each frame stands alone, so a dropped one costs
// nothing and a late viewer starts instantly. Same instinct as http_bridge.h.
//
// Takes finished JPEG bytes, so like pose_hub.h it includes neither OpenCV nor
// pylon. Single-threaded and non-blocking: the caller drives poll()/publish()
// from one thread. A viewer that cannot keep up simply misses frames — a frame
// is only queued for a client whose previous one has fully gone out, so a slow
// link never builds a backlog (and never adds latency).
//
// Loopback only, never INADDR_ANY: reach it through the SSH tunnel.

#pragma once

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

class MjpegServer {
public:
    ~MjpegServer() { stop(); }

    bool start(int port) {
        stop();
        listen_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_ < 0) return false;
        ::fcntl(listen_, F_SETFD, FD_CLOEXEC);
        int one = 1;
        ::setsockopt(listen_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in a{};
        a.sin_family      = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port        = htons((uint16_t)port);
        if (::bind(listen_, (sockaddr*)&a, sizeof(a)) < 0 || ::listen(listen_, 8) < 0) {
            ::close(listen_);
            listen_ = -1;
            return false;
        }
        setNonBlocking(listen_);
        return true;
    }

    void stop() {
        for (auto& c : conns_) ::close(c.fd);
        conns_.clear();
        viewers_ = 0;
        wanting_ = 0;
        if (listen_ >= 0) { ::close(listen_); listen_ = -1; }
    }

    bool isRunning() const { return listen_ >= 0; }

    // Streams currently attached.
    int viewers() const { return viewers_.load(std::memory_order_relaxed); }

    // Is anyone waiting for a frame — a stream, or a snapshot request? Atomic
    // because the producer asks from another thread, to decide whether
    // encoding is worth doing at all.
    bool wantsFrames() const { return wanting_.load(std::memory_order_relaxed) > 0; }

    // Accepts connections, answers requests, pushes queued bytes. Never blocks.
    void poll() {
        if (listen_ < 0) return;
        for (int fd; (fd = ::accept(listen_, nullptr, nullptr)) >= 0; ) {
            setNonBlocking(fd);
            ::fcntl(fd, F_SETFD, FD_CLOEXEC);
            conns_.push_back({fd, {}, {}, Kind::Request});
        }
        for (size_t i = 0; i < conns_.size();) {
            if (service(conns_[i])) {
                ++i;
            } else {
                ::close(conns_[i].fd);
                conns_.erase(conns_.begin() + (long)i);
            }
        }
        int streams = 0, waiting = 0;
        for (auto& c : conns_) {
            streams += c.kind == Kind::Stream;
            waiting += c.kind == Kind::Stream || c.kind == Kind::Snapshot;
        }
        viewers_  = streams;
        wanting_  = waiting;
    }

    // Hands one JPEG to every stream that is ready for it, and answers any
    // waiting snapshot request with it. Call poll() as well; this only queues,
    // poll() writes.
    void publish(const uint8_t* jpeg, size_t len) {
        std::string part, once;
        for (auto& c : conns_) {
            if (c.kind == Kind::Snapshot) {
                if (once.empty()) once = response("200 OK", "image/jpeg", std::string((const char*)jpeg, len));
                c.tx   = once;
                c.kind = Kind::Once;
            } else if (c.kind == Kind::Stream && c.tx.empty()) {     // busy: skip, don't queue
                if (part.empty())
                    part = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                           std::to_string(len) + "\r\n\r\n" + std::string((const char*)jpeg, len) + "\r\n";
                c.tx = part;
            }
        }
    }

private:
    enum class Kind { Request, Stream, Snapshot, Once };   // Snapshot = waiting for a frame; Once = flush, close

    struct Conn {
        int         fd;
        std::string rx;
        std::string tx;
        Kind        kind;
    };

    static void setNonBlocking(int fd) {
        ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    }

    // Returns false when the connection is finished and should be closed.
    bool service(Conn& c) {
        char    buf[2048];
        ssize_t n;
        while ((n = ::read(c.fd, buf, sizeof(buf))) > 0) {
            if (c.kind == Kind::Request) c.rx.append(buf, (size_t)n);   // later input is ignored
        }
        if (n == 0) return false;                                       // viewer closed
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;

        if (c.kind == Kind::Request) {
            if (c.rx.size() > 16384) return false;
            if (c.rx.find("\r\n\r\n") != std::string::npos) route(c);
        }

        if (!c.tx.empty()) {
            ssize_t w = ::send(c.fd, c.tx.data(), c.tx.size(), MSG_NOSIGNAL);
            if (w > 0) c.tx.erase(0, (size_t)w);
            else if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;
        }
        return !(c.kind == Kind::Once && c.tx.empty());
    }

    void route(Conn& c) {
        std::string path;
        size_t sp1 = c.rx.find(' ');
        size_t sp2 = sp1 == std::string::npos ? sp1 : c.rx.find(' ', sp1 + 1);
        if (sp2 != std::string::npos) path = c.rx.substr(sp1 + 1, sp2 - sp1 - 1);

        if (path == "/stream") {
            c.tx   = "HTTP/1.1 200 OK\r\n"
                     "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                     "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
            c.kind = Kind::Stream;
        } else if (path == "/snapshot.jpg") {
            c.kind = Kind::Snapshot;     // answered by the next publish()
        } else {
            c.tx   = response("200 OK", "text/html; charset=utf-8", kPage);
            c.kind = Kind::Once;
        }
    }

    static std::string response(const char* status, const char* type, const std::string& body) {
        return std::string("HTTP/1.1 ") + status + "\r\nContent-Type: " + type +
               "\r\nCache-Control: no-store\r\nContent-Length: " + std::to_string(body.size()) +
               "\r\nConnection: close\r\n\r\n" + body;
    }

    static constexpr const char* kPage =
        "<!doctype html><meta charset=utf-8><title>vision_hub</title>"
        "<body style=\"margin:0;background:#111\">"
        "<img src=\"/stream\" style=\"width:100vw;height:100vh;object-fit:contain\">";

    int               listen_ = -1;
    std::vector<Conn> conns_;
    std::atomic<int>  viewers_{0};
    std::atomic<int>  wanting_{0};
};
