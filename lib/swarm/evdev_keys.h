// evdev_keys.h — Linux global multi-key polling via evdev (/dev/input/eventN).
//
// XQueryKeymap (the previous approach) only reflects keys delivered to an X11
// surface. Under a Wayland session, XWayland's "core keyboard" never receives
// input meant for native Wayland clients (e.g. the terminal swarm_controller
// runs in, or GNOME's own surfaces), so XQueryKeymap silently reports nothing
// pressed — there is no error, WASD just does nothing. evdev reads straight
// from the kernel input layer, bypassing the compositor entirely, so it works
// the same under X11, Xwayland, and pure Wayland.
//
// Requires read access to /dev/input/eventN, which on stock Ubuntu is
// root:input 0660 — add yourself to the `input` group:
//   sudo usermod -aG input $USER   (then log out and back in)
#pragma once
#ifndef __APPLE__

#include <linux/input.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <cstring>
#include <string>
#include <vector>

class EvdevKeyboard {
public:
    // Opens every /dev/input/eventN that exposes a KEY_W key (our proxy for
    // "this is a keyboard, not a mouse/touchpad/etc."). Returns the count.
    int open() {
        close();
        DIR* dir = opendir("/dev/input");
        if (!dir) return 0;
        for (struct dirent* ent; (ent = readdir(dir)) != nullptr; ) {
            if (strncmp(ent->d_name, "event", 5) != 0) continue;
            std::string path = std::string("/dev/input/") + ent->d_name;
            int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0) continue;
            if (hasKey(fd, KEY_W)) fds_.push_back(fd);
            else ::close(fd);
        }
        closedir(dir);
        return (int)fds_.size();
    }

    // True if any open device currently reports `code` (a Linux KEY_* constant) held.
    bool down(int code) const {
        unsigned long bits[kKeyLongs] = {};
        for (int fd : fds_) {
            if (ioctl(fd, EVIOCGKEY(sizeof(bits)), bits) < 0) continue;
            if (bits[code / kBitsPerLong] & (1UL << (code % kBitsPerLong)))
                return true;
            memset(bits, 0, sizeof(bits));
        }
        return false;
    }

    void close() {
        for (int fd : fds_) ::close(fd);
        fds_.clear();
    }

    ~EvdevKeyboard() { close(); }

private:
    static constexpr size_t kBitsPerLong = 8 * sizeof(long);
    static constexpr size_t kKeyLongs    = KEY_MAX / kBitsPerLong + 1;

    static bool hasKey(int fd, int code) {
        unsigned long bits[kKeyLongs] = {};
        if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0) return false;
        return bits[code / kBitsPerLong] & (1UL << (code % kBitsPerLong));
    }

    std::vector<int> fds_;
};

#endif // !__APPLE__
