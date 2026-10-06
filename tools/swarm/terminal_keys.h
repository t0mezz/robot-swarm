// terminal_keys.h — WASD key state read from the terminal's own input stream.
//
// evdev_keys.h (and CoreGraphics on macOS) read the keyboard *attached to the
// machine the process runs on*. Over ssh that is the robot PC's keyboard, not
// the one the operator is typing on, so a remote session needs its keys from
// stdin instead. A terminal only sends characters, so "is W held" has to be
// reconstructed, two ways:
//
//  • Kitty keyboard protocol (kitty, WezTerm, foot, Ghostty, iTerm2, recent
//    Alacritty/Windows Terminal, ...). The terminal reports press / repeat /
//    release for every key, so state is exact, multi-key works, and nothing
//    depends on key-repeat settings. Probed at start-up (CSI ? u); if the
//    terminal answers, it is enabled with flags 1|2|8 and popped again on end().
//
//  • Timeout fallback (any other terminal, e.g. GNOME Terminal over ssh). A key
//    counts as held until its characters stop arriving. The OS sends one
//    character, pauses for the typematic delay (~0.25-0.6 s), then repeats at
//    ~30 Hz — so the first press is held for kInitialHoldMs, and once repeats
//    are seen the hold shrinks to kRepeatHoldMs. Two limits come with it: a
//    tap keeps driving for kInitialHoldMs, and the terminal only repeats the
//    *last* key pressed, so W+D combinations (arcs) drop the first key. Use a
//    kitty-protocol terminal when those matter.
//
// Everything that is not WASD is handed back as plain command characters so
// the caller's menu/select/quit handling is the same in both modes.
#pragma once

#include <unistd.h>
#include <sys/select.h>

#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>

class TerminalKeys {
public:
    using Clock = std::chrono::steady_clock;

    bool active() const { return active_; }
    bool kitty()  const { return kitty_; }

    // Call with the terminal already in raw mode. Returns true if the kitty
    // protocol was negotiated.
    bool begin() {
        active_ = true;
        // ?u is answered only by terminals that implement the protocol; DA1
        // (c) is answered by everything, so it marks "no more replies coming".
        writeOut("\033[?u\033[c");
        std::string reply;
        auto deadline = Clock::now() + std::chrono::milliseconds(700);   // ssh round trip
        while (Clock::now() < deadline && reply.find('c') == std::string::npos) {
            char b[64];
            fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
            timeval tv{0, 50000};
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;
            ssize_t n = read(STDIN_FILENO, b, sizeof(b));
            if (n > 0) reply.append(b, (size_t)n);
        }
        kitty_ = reply.find("\033[?") != std::string::npos &&
                 reply.find('u') != std::string::npos &&
                 reply.find('u') < reply.find('c');
        if (kitty_) writeOut("\033[>11u");   // 1 disambiguate | 2 event types | 8 all keys as escapes
        return kitty_;
    }

    void end() {
        if (kitty_) writeOut("\033[<u");
        kitty_ = false;
        active_ = false;
    }

    // Held state for a lowercase letter.
    bool down(char c) const {
        const Key* k = slot(c);
        if (!k) return false;
        if (kitty_) return k->held;
        if (!k->seen) return false;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - k->last).count();
        return ms < (k->repeating ? kRepeatHoldMs : kInitialHoldMs);
    }

    // Drops every held key (focus lost, stop requested).
    void releaseAll() { for (Key& k : keys_) k = Key{}; }

    // Reads whatever is waiting on stdin, updates the WASD state, and returns
    // the next non-WASD command character, or -1 when there is none.
    int nextCommand() {
        char b[64];
        ssize_t n = read(STDIN_FILENO, b, sizeof(b));
        if (n > 0) pending_.append(b, (size_t)n);

        while (!pending_.empty()) {
            unsigned char c = (unsigned char)pending_[0];
            if (c != 0x1B) {
                pending_.erase(0, 1);
                int cmd = plainByte(c);
                if (cmd >= 0) return cmd;
                continue;
            }
            if (pending_.size() == 1) {   // lone ESC (a split sequence would arrive within a tick)
                pending_.clear();
                return 0x1B;
            }
            if (pending_[1] != '[' && pending_[1] != 'O') { pending_.erase(0, 1); return 0x1B; }

            size_t end = 2;               // CSI/SS3: parameters, then one final byte 0x40-0x7E
            while (end < pending_.size() && !(pending_[end] >= 0x40 && pending_[end] <= 0x7E)) end++;
            if (end >= pending_.size()) break;   // incomplete — wait for the rest
            std::string seq = pending_.substr(0, end + 1);
            pending_.erase(0, end + 1);
            int cmd = escapeSeq(seq);
            if (cmd >= 0) return cmd;
        }
        return -1;
    }

private:
    static constexpr int kInitialHoldMs = 600;
    static constexpr int kRepeatHoldMs  = 150;
    static constexpr int kRepeatGapMs   = 200;   // a byte this soon after the last one is a repeat

    struct Key {
        bool held = false;               // kitty
        bool seen = false, repeating = false;   // fallback
        Clock::time_point last{};
    };

    Key keys_[4];                        // w a s d
    bool active_ = false, kitty_ = false;
    std::string pending_;

    static int idx(char c) {
        switch (c) { case 'w': return 0; case 'a': return 1; case 's': return 2; case 'd': return 3; }
        return -1;
    }
    const Key* slot(char c) const { int i = idx(c); return i < 0 ? nullptr : &keys_[i]; }
    Key*       slot(char c)       { int i = idx(c); return i < 0 ? nullptr : &keys_[i]; }

    static void writeOut(const char* s) { ssize_t r = write(STDOUT_FILENO, s, strlen(s)); (void)r; }

    // Fallback path: a plain byte. WASD updates state; anything else is a command.
    int plainByte(unsigned char c) {
        char lc = (c >= 'A' && c <= 'Z') ? (char)(c + 32) : (char)c;
        if (Key* k = slot(lc)) {
            if (!kitty_) {
                auto now = Clock::now();
                bool recent = k->seen &&
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - k->last).count() < kRepeatGapMs;
                k->repeating = recent;
                k->seen = true;
                k->last = now;
            }
            return -1;
        }
        return c;
    }

    // CSI <code>[:alt][;<mods>[:<event>]] u — kitty key event. Other sequences
    // (arrows, focus, the DA1 reply) carry nothing this tool uses.
    int escapeSeq(const std::string& s) {
        if (s.back() != 'u' || s[1] != '[') return -1;
        int code = 0, mods = 1, event = 1;
        if (sscanf(s.c_str() + 2, "%d", &code) < 1) return -1;
        size_t semi = s.find(';');
        if (semi != std::string::npos) {
            sscanf(s.c_str() + semi + 1, "%d", &mods);
            size_t colon = s.find(':', semi);
            if (colon != std::string::npos) sscanf(s.c_str() + colon + 1, "%d", &event);
        }
        if (code < 0 || code > 127) return -1;

        if (Key* k = slot((char)code)) {
            k->held = (event != 3);
            return -1;
        }
        if (event == 3) return -1;                       // other keys: act on press/repeat only
        if (((mods - 1) & 4) != 0) return code >= 'a' && code <= 'z' ? code - 'a' + 1 : -1;   // Ctrl+letter → control byte
        return code;
    }
};
