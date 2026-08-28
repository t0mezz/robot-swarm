// http_bridge.h — a tiny localhost HTTP server, just big enough to serve one
// page and receive parameter updates back from it.
//
// Used by tools/vision/car_following.cpp to mirror the live NetLogo page: the
// page is served from here with a small script appended, and that script
// POSTs the current model + slider values back whenever they change.
//
// Plain HTTP rather than WebSockets on purpose. The traffic is one small POST
// per parameter change over loopback, so the handshake, SHA-1, base64 and
// frame masking a WebSocket would need buy nothing — this whole file is
// smaller than the handshake alone would have been. Same instinct as
// pose_hub.h: hand-roll the minimum rather than take a dependency.
//
// Non-blocking throughout, including writes: poll() is called from a control
// loop that is driving physical robots, and the page is several megabytes.

#pragma once

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

class HttpBridge {
public:
    ~HttpBridge() { stop(); }

    // Binds 127.0.0.1:port. Loopback only, never INADDR_ANY — a POST to this
    // server changes how real robots drive.
    bool start(int port, std::string page) {
        stop();
        page_ = std::move(page);
        listen_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_ < 0) return false;

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
        if (listen_ >= 0) { ::close(listen_); listen_ = -1; }
    }

    bool isRunning() const { return listen_ >= 0; }

    // Accepts new connections, answers any complete requests, and returns the
    // bodies of the POSTs received since the last call. Never blocks.
    std::vector<std::string> poll() {
        std::vector<std::string> posts;
        if (listen_ < 0) return posts;

        for (int fd; (fd = ::accept(listen_, nullptr, nullptr)) >= 0; ) {
            setNonBlocking(fd);
            conns_.push_back({fd, {}, {}, false});
        }

        for (size_t i = 0; i < conns_.size(); ) {
            Conn& c = conns_[i];
            bool  drop = false;

            if (!c.answered) {
                char    buf[4096];
                ssize_t n;
                while ((n = ::read(c.fd, buf, sizeof(buf))) > 0) c.rx.append(buf, (size_t)n);
                if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) drop = true;
                else if (c.rx.size() > (1u << 20)) drop = true;   // no request is this big
                else serve(c, posts);
            }

            if (!drop && !c.tx.empty()) {
                ssize_t n = ::write(c.fd, c.tx.data(), c.tx.size());
                if (n > 0) c.tx.erase(0, (size_t)n);
                else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) drop = true;
            }

            if (drop || (c.answered && c.tx.empty())) {
                ::close(c.fd);
                conns_.erase(conns_.begin() + (long)i);
            } else {
                ++i;
            }
        }
        return posts;
    }

private:
    struct Conn {
        int         fd;
        std::string rx;
        std::string tx;
        bool        answered;
    };

    static void setNonBlocking(int fd) {
        ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    }

    // Queues a response once the request is complete. Anything that is not a
    // POST gets the page, so both "/" and a browser's /favicon.ico are handled
    // without a router.
    void serve(Conn& c, std::vector<std::string>& posts) {
        size_t hdrEnd = c.rx.find("\r\n\r\n");
        if (hdrEnd == std::string::npos) return;          // headers still arriving

        size_t want = 0;
        size_t cl   = c.rx.find("Content-Length:");
        if (cl != std::string::npos)
            want = (size_t)strtoul(c.rx.c_str() + cl + 15, nullptr, 10);

        size_t bodyAt = hdrEnd + 4;
        if (c.rx.size() < bodyAt + want) return;          // body still arriving

        if (c.rx.compare(0, 5, "POST ") == 0) {
            posts.push_back(c.rx.substr(bodyAt, want));
            c.tx = response("204 No Content", nullptr, "");
        } else {
            c.tx = response("200 OK", "text/html; charset=utf-8", page_);
        }
        c.answered = true;
    }

    static std::string response(const char* status, const char* type,
                                const std::string& body) {
        std::string h = std::string("HTTP/1.1 ") + status + "\r\n";
        if (type) h += std::string("Content-Type: ") + type + "\r\n";
        h += "Content-Length: " + std::to_string(body.size()) + "\r\n"
             "Connection: close\r\n\r\n";
        return h + body;
    }

    int               listen_ = -1;
    std::string       page_;
    std::vector<Conn> conns_;
};
