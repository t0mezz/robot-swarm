// controller.cpp
// ═══════════════════════════════════════════════════════════════
//  R O B O T   R A C E R   —   Retro Cockpit
// ═══════════════════════════════════════════════════════════════
//
// W = Gas (quadratisch gedämpft)  |  S = Bremse (kein Rückwärts)
// A/D = Lenkung  |  SPACE = Boost  |  SHIFT = Drift
//
// Kompilieren (macOS):
//   g++ controller.cpp -o controller -std=c++17 \
//       -I/opt/homebrew/opt/sfml@2/include -L/opt/homebrew/opt/sfml@2/lib \
//       -lsfml-graphics -lsfml-window -lsfml-system
//
// Kompilieren (Linux):
//   g++ controller.cpp -o controller -std=c++17 \
//       -lsfml-graphics -lsfml-window -lsfml-system
//
// Voraussetzung: swarm_hub muss laufen
//   ./swarm_hub /dev/tty.usbmodem*

#include <SFML/Graphics.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <deque>
#include <optional>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <chrono>

static const float PI = 3.14159265f;

// ─── Protokoll ────────────────────────────────────────────────
static const uint8_t MAGIC_0    = 0xAA;
static const uint8_t MAGIC_1    = 0x55;
static const uint8_t MSG_SWARM  = 0x10;
static const int     NUM_ROBOTS = 20;

static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

// ─── Farbpalette: Retro Racing ────────────────────────────────
namespace C {
    // Dunkler Hintergrund mit warmem Unterton
    const sf::Color bg         (15, 10, 25);
    const sf::Color bgAlt      (22, 16, 35);
    const sf::Color surface    (30, 22, 48);
    const sf::Color surfaceLit (42, 32, 65);
    const sf::Color panel      (25, 18, 40);
    const sf::Color border     (60, 45, 90);
    const sf::Color borderLit  (80, 60, 120);
    const sf::Color grid       (40, 30, 60);

    // Text
    const sf::Color textBright (255, 245, 220);
    const sf::Color textMid    (180, 160, 200);
    const sf::Color textDim    (100, 85, 130);

    // Retro Neon-Akzente — satte, warme Farben
    const sf::Color hotPink    (255, 40, 120);
    const sf::Color hotPinkDim (150, 20, 70);
    const sf::Color hotPinkGlow(255, 40, 120);  // Fuer Glow-Effekte

    const sf::Color cyan       (0, 245, 255);
    const sf::Color cyanDim    (0, 120, 130);

    const sf::Color yellow     (255, 230, 0);
    const sf::Color yellowDim  (160, 140, 0);
    const sf::Color yellowWarm (255, 200, 40);

    const sf::Color orange     (255, 120, 20);
    const sf::Color orangeDim  (160, 70, 10);
    const sf::Color orangeHot  (255, 80, 0);

    const sf::Color green      (0, 255, 100);
    const sf::Color greenDim   (0, 130, 50);

    const sf::Color violet     (160, 60, 255);
    const sf::Color violetDim  (90, 30, 150);
    const sf::Color violetGlow (180, 80, 255);

    const sf::Color red        (255, 30, 50);
    const sf::Color redDim     (150, 15, 25);

    // Tacho-Segmente
    const sf::Color tachoOff   (35, 28, 55);
    const sf::Color tachoLow   (0, 255, 100);
    const sf::Color tachoMid   (255, 230, 0);
    const sf::Color tachoHigh  (255, 40, 40);

    inline sf::Color alpha(sf::Color c, uint8_t a) {
        return sf::Color(c.r, c.g, c.b, a);
    }
    inline sf::Color lerp(sf::Color a, sf::Color b, float t) {
        t = std::max(0.f, std::min(1.f, t));
        return sf::Color(
            a.r + (int)((b.r - a.r)*t), a.g + (int)((b.g - a.g)*t),
            a.b + (int)((b.b - a.b)*t), a.a + (int)((b.a - a.a)*t));
    }
}

// ─── Hub Socket Connection ────────────────────────────────────
class HubSocket {
    int fd_ = -1;
    static constexpr const char* HUB_SOCK_PATH = "/tmp/swarm_hub.sock";
    
public:
    bool connect() {
        fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) { 
            std::cerr << "Cannot create socket: " << strerror(errno) << "\n"; 
            return false; 
        }
        
        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, HUB_SOCK_PATH, sizeof(addr.sun_path) - 1);
        
        if (::connect(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) { 
            std::cerr << "Cannot connect to swarm_hub at " << HUB_SOCK_PATH << ": " 
                      << strerror(errno) << "\n";
            close_fd();
            return false; 
        }
        
        int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        
        return true;
    }
    
    bool send(const uint8_t* d, size_t l) { 
        if (fd_ < 0) return false;
        ssize_t written = ::write(fd_, d, l);
        if (written < 0 && (errno == EPIPE || errno == EBADF)) {
            close_fd();
            return false;
        }
        return written == (ssize_t)l;
    }
    
    bool is_open() const { return fd_ >= 0; }
    
    void close() { close_fd(); }
    
    ~HubSocket() { close_fd(); }
    
private:
    void close_fd() { 
        if (fd_ >= 0) { 
            ::close(fd_); 
            fd_ = -1; 
        } 
    }
};

// ─── Swarm-Paket ──────────────────────────────────────────────
static void send_swarm(HubSocket& hub, int ctrl_id, int8_t left, int8_t right) {
    uint8_t pl = NUM_ROBOTS * 3;
    std::vector<uint8_t> f(5 + pl, 0);
    f[0] = MAGIC_0; f[1] = MAGIC_1; f[2] = MSG_SWARM; f[3] = pl;
    for (int i = 0; i < NUM_ROBOTS; i++) {
        f[4 + i*3] = (uint8_t)i;
        if (i == ctrl_id) { f[4+i*3+1] = (uint8_t)left; f[4+i*3+2] = (uint8_t)right; }
    }
    f[4+pl] = crc8(&f[2], pl+2);
    hub.send(f.data(), f.size());
}

// ─── Fahrzeugphysik ──────────────────────────────────────────
struct Vehicle {
    float throttle     = 0.f;   // 0..1 nur vorwaerts!
    float steer        = 0.f;
    float speed        = 0.f;
    float boost        = 1.f;
    float boostActive  = 0.f;
    bool  boosting     = false;
    bool  drifting     = false;
    bool  braking      = false;
    float heading      = 0.f;
    float gForceX      = 0.f;
    float gForceY      = 0.f;
    float leftMotor    = 0.f;
    float rightMotor   = 0.f;

    std::deque<std::pair<float,float>> trail;
    static const int TRAIL_MAX = 100;

    // ── Tuning ───────────────────────────────────────────
    float maxSpeed     = 0.5f;

    float accelRate    = 1.0f;
    float brakeRate    = 1.5f;    // Starkes Bremsen
    float coastRate    = 0.6f;

    float steerSpeed   = 6.0f;
    float steerReturn  = 10.0f;
    float turnFactor   = 0.15f;
    float spinFactor   = 0.2f;
    float driftMult    = 2.8f;

    float boostPower   = 2.0f;
    float boostDrain   = 0.45f;
    float boostRegen   = 0.08f;

    void update(bool w, bool s, bool a, bool d, bool space, bool shift, float dt) {
        // ── Throttle: nur vorwaerts, quadratisch gedaempft ──
        braking = s && throttle > 0.001f;

        if (w && !s) {
            float remaining = 1.f - throttle;
            float delta = accelRate * (0.15f + remaining * remaining) * dt;
            throttle = std::min(1.f, throttle + delta);
        } else if (s) {
            // S = nur bremsen, nie unter 0
            float delta = brakeRate * dt;
            throttle = std::max(0.f, throttle - delta);
        } else {
            // Coast: langsam ausrollen
            float remaining = 1.f - throttle;
            float delta = coastRate * (0.15f + remaining * remaining) * dt;
            throttle = std::max(0.f, throttle - delta);
        }

        // ── Lenkung ──────────────────────────────────────
        float steerTarget = 0.f;
        if (a && !d) steerTarget = -1.f;
        else if (d && !a) steerTarget = 1.f;

        if (steerTarget != 0.f) {
            float dir = (steerTarget > steer) ? 1.f : -1.f;
            steer += dir * steerSpeed * dt;
            if ((dir > 0 && steer > steerTarget) || (dir < 0 && steer < steerTarget))
                steer = steerTarget;
        } else {
            if (steer > 0.f) { steer -= steerReturn * dt; if (steer < 0.f) steer = 0.f; }
            else if (steer < 0.f) { steer += steerReturn * dt; if (steer > 0.f) steer = 0.f; }
        }

        // ── Drift ────────────────────────────────────────
        drifting = shift && std::abs(steer) > 0.1f && throttle > 0.1f;
        float effectiveTurn = turnFactor;
        if (drifting) effectiveTurn *= driftMult;

        // ── Boost ────────────────────────────────────────
        boosting = space && boost > 0.01f;
        if (boosting) {
            boostActive = std::min(boostActive + 6.f * dt, 1.f);
            boost -= boostDrain * dt;
            if (boost < 0.f) boost = 0.f;
        } else {
            boostActive = std::max(0.f, boostActive - 3.f * dt);
            boost = std::min(1.f, boost + boostRegen * dt);
        }

        // ── Power = Throttle + Boost (kann ueber 1.0 hinaus!) ──
        float power = throttle + boostActive * boostPower;
        power = std::min(1.7f, power);   // Boost erlaubt bis 130%

        // ── Differential Drive ───────────────────────────
        if (power < 0.01f && std::abs(steer) > 0.01f) {
            leftMotor  =  steer * spinFactor;
            rightMotor = -steer * spinFactor;
        } else {
            float sf = steer * effectiveTurn;
            leftMotor  = power + sf;
            rightMotor = power - sf;
        }
        float mx = std::max(std::abs(leftMotor), std::abs(rightMotor));
        if (mx > 1.3f) { leftMotor *= 1.3f / mx; rightMotor *= 1.3f / mx; }

        // ── Abgeleitete Werte ────────────────────────────
        float prevSpeed = speed;
        speed += (power - speed) * 3.f * dt;
        if (speed < 0.005f) speed = 0.f;

        float turnRate = steer * speed * 3.f * (drifting ? 1.8f : 1.f);
        heading += turnRate * dt;

        gForceX += (turnRate * speed * 2.f - gForceX) * 6.f * dt;
        gForceY += ((speed - prevSpeed) / std::max(dt, 0.001f) * 0.3f - gForceY) * 4.f * dt;

        trail.push_back({heading, speed});
        while ((int)trail.size() > TRAIL_MAX) trail.pop_front();
    }

    int8_t motorL() const { return (int8_t)std::round(std::max(-1.3f, std::min(1.3f, leftMotor)) * 127.f * maxSpeed); }
    int8_t motorR() const { return (int8_t)std::round(std::max(-1.3f, std::min(1.3f, rightMotor)) * 127.f * maxSpeed); }
};

static void draw_tacho(sf::RenderWindow& win, sf::Font& font,
                        const Vehicle& v, sf::Vector2f center, float radius) {
    // ... (Glow und Hintergrund-Kreis bleiben gleich) ...

    // Definition der Winkel (in Bogenmaß)
    float arcStart = 140.f * PI / 180.f;
    float arcEnd   = 400.f * PI / 180.f; // Das ist das "normale" Ende (bei 100%)
    float totalArc = arcEnd - arcStart;

    // Tacho-Logik: 
    // 0.0 bis 1.0 (100%) füllt den normalen Bogen.
    // 1.0 bis 1.3 (Boost) lässt die Nadel "überdrehen".
    float speedFrac = v.speed; 
    
    // Segmente zeichnen
    int totalSegs = 40; 
    int litSegs = (int)(std::min(1.0f, speedFrac) * totalSegs);

    for (int i = 0; i < totalSegs; i++) {
        float t = (float)i / totalSegs;
        float a0 = arcStart + totalArc * t;
        float a1 = arcStart + totalArc * (t + 1.f / totalSegs);
        float gap = 0.012f;

        sf::Color col;
        if (i < litSegs) {
            float normT = (float)i / totalSegs;
            if (normT < 0.45f) col = C::green;
            else if (normT < 0.7f) col = C::yellow;
            else col = C::red;
        } else {
            col = C::tachoOff;
        }

        float inner = radius * 0.74f;
        float outer = radius * 0.94f;

        sf::Vector2f p0i = center + sf::Vector2f(std::cos(a0+gap), std::sin(a0+gap)) * inner;
        sf::Vector2f p0o = center + sf::Vector2f(std::cos(a0+gap), std::sin(a0+gap)) * outer;
        sf::Vector2f p1i = center + sf::Vector2f(std::cos(a1-gap), std::sin(a1-gap)) * inner;
        sf::Vector2f p1o = center + sf::Vector2f(std::cos(a1-gap), std::sin(a1-gap)) * outer;

        sf::ConvexShape seg(4);
        seg.setPoint(0, p0i); seg.setPoint(1, p0o);
        seg.setPoint(2, p1o); seg.setPoint(3, p1i);
        seg.setFillColor(col);
        win.draw(seg);
    }

    // --- NADEL LOGIK FÜR ÜBERDREHEN ---
    // Bei speed = 1.0 ist die Nadel bei 400° (arcEnd)
    // Bei speed = 1.3 ist die Nadel bei ca. 500° (fast eine ganze Umdrehung ab Start)
    float needleAngle = arcStart + (speedFrac * totalArc); 
    
    // Optischer Effekt: Wenn über 100%, färbe Nadel orange-glühend
    sf::Color needleCol = v.speed > 1.0f ? C::orangeHot : (v.boosting ? C::orange : C::hotPink);

    float needleLen = radius * 0.66f;
    sf::Vector2f tip = center + sf::Vector2f(std::cos(needleAngle), std::sin(needleAngle)) * needleLen;
    sf::Vector2f perp(-std::sin(needleAngle), std::cos(needleAngle));

    sf::ConvexShape needle(3);
    needle.setPoint(0, tip);
    needle.setPoint(1, center + perp * 3.f);
    needle.setPoint(2, center - perp * 3.f);
    needle.setFillColor(needleCol);
    win.draw(needle);

    sf::CircleShape hub(6);
    hub.setOrigin(sf::Vector2f(6, 6));
    hub.setPosition(center);
    hub.setFillColor(C::hotPink);
    win.draw(hub);

    // Speed-Zahl (kann ueber 100 gehen bei Boost)
    int displaySpeed = (int)(v.speed * 100);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", displaySpeed);
    sf::Text spd(font, buf, 34);
    spd.setOrigin(sf::Vector2f(spd.getLocalBounds().size.x / 2, spd.getLocalBounds().size.y));
    spd.setPosition(sf::Vector2f(center.x, center.y + radius * 0.28f));
    bool overdrive = v.speed > 1.0f;
    spd.setFillColor(overdrive ? C::orangeHot : (v.boosting ? C::orange : C::textBright));
    win.draw(spd);

    sf::Text unit(font, "SPD", 10);
    unit.setOrigin(sf::Vector2f(unit.getLocalBounds().size.x / 2, 0));
    unit.setPosition(sf::Vector2f(center.x, center.y + radius * 0.32f));
    unit.setFillColor(C::textDim);
    win.draw(unit);

    // Brake Indikator
    if (v.braking) {
        sf::Text brk(font, "BRAKE", 10);
        brk.setOrigin(sf::Vector2f(brk.getLocalBounds().size.x / 2, 0));
        brk.setPosition(sf::Vector2f(center.x, center.y + radius * 0.52f));
        brk.setFillColor(C::red);
        win.draw(brk);
    }
}

// ─── Boost-Balken ────────────────────────────────────────────
static void draw_boost(sf::RenderWindow& win, sf::Font& font,
                        const Vehicle& v, sf::Vector2f pos, float w = 180.f) {
    float h = 18.f;

    // Hintergrund mit Glow wenn aktiv
    sf::RectangleShape frame({w, h});
    frame.setPosition(pos);
    frame.setFillColor(C::surface);
    frame.setOutlineThickness(1.5f);
    frame.setOutlineColor(v.boosting ? C::orange : C::border);
    win.draw(frame);

    float fill = v.boost * (w - 4);
    if (fill > 1.f) {
        sf::RectangleShape bar({fill, h - 4});
        bar.setPosition(sf::Vector2f(pos.x + 2, pos.y + 2));
        sf::Color col = C::orange;
        if (v.boost < 0.25f) col = C::red;
        if (v.boosting) col = C::lerp(col, C::yellow, 0.4f);
        bar.setFillColor(col);
        win.draw(bar);
    }

    if (v.boosting) {
        sf::Text fb(font, "NITRO!", 10);
        fb.setPosition(sf::Vector2f(pos.x + w + 10, pos.y));
        fb.setFillColor(C::orangeHot);
        win.draw(fb);
    }

    sf::Text lbl(font, "BOOST  [SPACE]", 9);
    lbl.setPosition(sf::Vector2f(pos.x, pos.y - 16));
    lbl.setFillColor(C::textDim);
    win.draw(lbl);
}

// ─── G-Force ─────────────────────────────────────────────────
static void draw_gforce(sf::RenderWindow& win, sf::Font& font,
                          const Vehicle& v, sf::Vector2f center, float radius) {
    sf::CircleShape ring(radius);
    ring.setOrigin(sf::Vector2f(radius, radius));
    ring.setPosition(center);
    ring.setFillColor(C::surface);
    ring.setOutlineThickness(1.5f);
    ring.setOutlineColor(C::border);
    win.draw(ring);

    for (int i = 0; i < 2; i++) {
        sf::RectangleShape line(i == 0 ? sf::Vector2f{radius*1.4f, 1} : sf::Vector2f{1, radius*1.4f});
        line.setOrigin(sf::Vector2f(line.getSize().x/2, line.getSize().y/2));
        line.setPosition(center);
        line.setFillColor(C::grid);
        win.draw(line);
    }

    float gx = std::max(-1.f, std::min(1.f, v.gForceX));
    float gy = std::max(-1.f, std::min(1.f, -v.gForceY));
    float mag = std::sqrt(gx*gx + gy*gy);
    sf::Vector2f ballPos = center + sf::Vector2f(gx, gy) * (radius * 0.7f);

    sf::CircleShape glow(10);
    glow.setOrigin(sf::Vector2f(10, 10));
    glow.setPosition(ballPos);
    glow.setFillColor(C::alpha(mag > 0.5f ? C::hotPink : C::cyan, 30));
    win.draw(glow);

    sf::CircleShape ball(5);
    ball.setOrigin(sf::Vector2f(5, 5));
    ball.setPosition(ballPos);
    ball.setFillColor(mag > 0.5f ? C::hotPink : C::cyan);
    win.draw(ball);

    sf::Text lbl(font, "G-FORCE", 8);
    lbl.setOrigin(sf::Vector2f(lbl.getLocalBounds().size.x/2, lbl.getLocalBounds().size.y));
    lbl.setPosition(sf::Vector2f(center.x, center.y - radius - 8));
    lbl.setFillColor(C::textDim);
    win.draw(lbl);
}

// ─── Trail ───────────────────────────────────────────────────
static void draw_trail(sf::RenderWindow& win, sf::Font& font, const Vehicle& v,
                        sf::Vector2f center, float scale) {
    if (v.trail.size() < 2) return;

    // Retro-Grid Hintergrund
    sf::RectangleShape panel({scale * 2.4f, scale * 2.4f});
    panel.setOrigin(sf::Vector2f(scale * 1.2f, scale * 1.2f));
    panel.setPosition(center);
    panel.setFillColor(C::bgAlt);
    panel.setOutlineThickness(1.5f);
    panel.setOutlineColor(C::border);
    win.draw(panel);

    // Grid-Linien
    for (int i = -2; i <= 2; i++) {
        float off = i * scale * 0.4f;
        sf::RectangleShape h({scale * 2.2f, 1});
        h.setOrigin(sf::Vector2f(scale * 1.1f, 0));
        h.setPosition(sf::Vector2f(center.x, center.y + off));
        h.setFillColor(C::grid);
        win.draw(h);
        sf::RectangleShape vl({1, scale * 2.2f});
        vl.setOrigin(sf::Vector2f(0, scale * 1.1f));
        vl.setPosition(sf::Vector2f(center.x + off, center.y));
        vl.setFillColor(C::grid);
        win.draw(vl);
    }

    float px = 0.f, py = 0.f;
    std::vector<sf::Vector2f> pts;
    pts.push_back({0, 0});

    for (int i = (int)v.trail.size() - 1; i >= 0; i--) {
        auto [h, s] = v.trail[i];
        px -= std::cos(h) * s * scale * 0.5f;
        py -= std::sin(h) * s * scale * 0.5f;
        pts.push_back({px, py});
    }

    for (int i = 0; i < (int)pts.size() - 1; i++) {
        float t = 1.f - (float)i / pts.size();
        uint8_t a = (uint8_t)(t * 220);
        sf::Color col = v.drifting ? C::alpha(C::hotPink, a) : C::alpha(C::cyan, a);

        sf::Vertex line[] = {
            {center + pts[i], col},
            {center + pts[i+1], C::alpha(col, a / 3)}
        };
        win.draw(line, 2, sf::PrimitiveType::Lines);
    }

    // Roboter-Punkt
    sf::CircleShape dot(5);
    dot.setOrigin(sf::Vector2f(5, 5));
    dot.setPosition(center);
    dot.setFillColor(v.drifting ? C::hotPink : C::cyan);
    win.draw(dot);

    // Glow
    sf::CircleShape dotGlow(9);
    dotGlow.setOrigin(sf::Vector2f(9, 9));
    dotGlow.setPosition(center);
    dotGlow.setFillColor(C::alpha(v.drifting ? C::hotPink : C::cyan, 40));
    win.draw(dotGlow);

    sf::Text lbl(font, "TRAIL", 8);
    lbl.setOrigin(sf::Vector2f(lbl.getLocalBounds().size.x/2, lbl.getLocalBounds().size.y));
    lbl.setPosition(sf::Vector2f(center.x, center.y - scale * 1.2f - 8));
    lbl.setFillColor(C::textDim);
    win.draw(lbl);
}

// ─── Motor-Balken ────────────────────────────────────────────
static void draw_motor(sf::RenderWindow& win, sf::Font& font,
                        float value, sf::Vector2f pos, const std::string& label) {
    float bh = 110.f, bw = 22.f;

    sf::RectangleShape bg({bw, bh});
    bg.setOrigin(sf::Vector2f(bw/2, bh/2));
    bg.setPosition(pos);
    bg.setFillColor(C::surface);
    bg.setOutlineThickness(1.f);
    bg.setOutlineColor(C::border);
    win.draw(bg);

    float fill = std::abs(value) * (bh/2 - 2);
    if (fill > 0.5f) {
        sf::RectangleShape bar({bw - 6, fill});
        bar.setOrigin(sf::Vector2f((bw-6)/2, 0));
        bar.setPosition(sf::Vector2f(pos.x, value >= 0 ? pos.y - fill : pos.y));
        bar.setFillColor(value >= 0 ? C::green : C::red);
        win.draw(bar);
    }

    sf::RectangleShape mid({bw+4, 1});
    mid.setOrigin(sf::Vector2f((bw+4)/2, 0));
    mid.setPosition(pos);
    mid.setFillColor(C::border);
    win.draw(mid);

    sf::Text lbl(font, label, 10);
    lbl.setOrigin(sf::Vector2f(lbl.getLocalBounds().size.x/2, 0));
    lbl.setPosition(sf::Vector2f(pos.x, pos.y + bh/2 + 6));
    lbl.setFillColor(C::textDim);
    win.draw(lbl);
}

// ─── Taste ───────────────────────────────────────────────────
static void draw_key(sf::RenderWindow& win, sf::Font& font,
                      sf::Vector2f pos, const std::string& key, bool active,
                      sf::Color col, float w = 32.f, float h = 32.f) {
    sf::RectangleShape bg({w, h});
    bg.setOrigin(sf::Vector2f(w/2, h/2));
    bg.setPosition(pos);
    bg.setFillColor(active ? C::alpha(col, 35) : C::surface);
    bg.setOutlineThickness(active ? 2.f : 1.f);
    bg.setOutlineColor(active ? col : C::border);
    win.draw(bg);

    sf::Text txt(font, key, key.size() > 2 ? 8 : 12);
    sf::FloatRect tb = txt.getLocalBounds();
    txt.setOrigin(sf::Vector2f(tb.position.x + tb.size.x/2, tb.position.y + tb.size.y/2));
    txt.setPosition(pos);
    txt.setFillColor(active ? col : C::textDim);
    win.draw(txt);
}

// ─── Drift Badge ─────────────────────────────────────────────
static void draw_drift_badge(sf::RenderWindow& win, sf::Font& font,
                               bool drifting, sf::Vector2f pos) {
    if (!drifting) return;

    sf::RectangleShape badge({64, 22});
    badge.setOrigin(sf::Vector2f(32, 11));
    badge.setPosition(pos);
    badge.setFillColor(C::alpha(C::hotPink, 30));
    badge.setOutlineThickness(1.5f);
    badge.setOutlineColor(C::hotPink);
    win.draw(badge);

    sf::Text txt(font, "DRIFT!", 11);
    txt.setOrigin(sf::Vector2f(txt.getLocalBounds().size.x/2, txt.getLocalBounds().size.y/2));
    txt.setPosition(sf::Vector2f(pos.x, pos.y - 1));
    txt.setFillColor(C::hotPink);
    win.draw(txt);
}

// ─── Main ─────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    int         send_ms   = (argc > 1) ? std::stoi(argv[1]) : 50;
    int         robot_id  = (argc > 2) ? std::stoi(argv[2]) : 0;
    if (robot_id < 0 || robot_id >= NUM_ROBOTS) robot_id = 0;

    HubSocket hub;
    std::cout << "Connecting to swarm_hub at /tmp/swarm_hub.sock...\n";
    
    // Try to connect with a few retries in case swarm_hub is just starting
    bool connected = hub.connect();
    if (!connected) {
        for (int i = 0; i < 5 && !connected; i++) {
            std::cout << "Retrying connection...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            connected = hub.connect();
        }
    }
    
    if (!connected) {
        std::cerr << "Error: Could not connect to swarm_hub. Make sure it's running:\n";
        std::cerr << "  ./swarm_hub /dev/tty.usbmodem*\n";
        return 1;
    }
    
    std::cout << "Connected to swarm_hub\n";
    std::cout << "Robot ID: " << robot_id << "  |  60B MSG_SWARM  |  " << send_ms << "ms\n";
    std::cout << "F11: Fullscreen  |  ESC: Exit\n";

    sf::ContextSettings settings;
    settings.antiAliasingLevel = 8;

    sf::VideoMode desktopMode = sf::VideoMode::getDesktopMode();
    sf::VideoMode windowedMode({800u, 520u});
    bool fullscreen = false;

    sf::RenderWindow win(windowedMode, "ROBOT RACER",
                        sf::Style::Default, sf::State::Windowed, settings); // Default statt Titlebar|Close
    win.setFramerateLimit(60);

    auto recreateWindow = [&]() {
        win.close();
        if (fullscreen) {
            // Wir nehmen die Desktop-Auflösung
            sf::VideoMode mode = sf::VideoMode::getDesktopMode();
            win.create(mode, "ROBOT RACER", sf::Style::None, sf::State::Fullscreen, settings);
        } else {
            win.create(windowedMode, "ROBOT RACER", sf::Style::Default, sf::State::Windowed, settings);
        }

        // WICHTIG: View manuell auf die tatsächliche Fenstergröße (in Pixeln) setzen
        sf::Vector2u size = win.getSize();
        sf::View view(sf::FloatRect({0.f, 0.f}, {static_cast<float>(size.x), static_cast<float>(size.y)}));
        win.setView(view);

        win.setFramerateLimit(60);
    };

    sf::Font font;
    bool hf = font.openFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf");
    if (!hf) hf = font.openFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
    if (!hf) hf = font.openFromFile("/System/Library/Fonts/Helvetica.ttc");

    Vehicle car;
    sf::Clock frameClock, sendClock;

    while (win.isOpen()) {
        float dt = frameClock.restart().asSeconds();
        if (dt > 0.1f) dt = 0.1f;


        while (const std::optional<sf::Event> ev = win.pollEvent()) {
            if (ev->is<sf::Event::Closed>()) win.close();

            // Hinzufügen: Wenn das Fenster seine Größe ändert (z.B. beim Übergang in Fullscreen)
            if (const auto* resized = ev->getIf<sf::Event::Resized>()) {
                sf::FloatRect visibleArea({0.f, 0.f},
                    {static_cast<float>(resized->size.x), static_cast<float>(resized->size.y)});
                win.setView(sf::View(visibleArea));
            }

            if (const auto* keyPressed = ev->getIf<sf::Event::KeyPressed>()) {
                if (keyPressed->code == sf::Keyboard::Key::F11) {
                    fullscreen = !fullscreen;
                    recreateWindow();
                }
                // ... restliche Keys ...
            }
        }

        sf::Vector2u ws = win.getSize();
        float sx = ws.x / 800.f, sy = ws.y / 520.f;
        float sc = std::min(sx, sy);
        float ox = (ws.x - 800.f * sc) / 2.f;
        float oy = (ws.y - 520.f * sc) / 2.f;
        auto S = [&](float x, float y) -> sf::Vector2f { return {ox + x*sc, oy + y*sc}; };
        auto Sf = [&](float v) -> float { return v * sc; };

        bool w     = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
        bool s     = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S);
        bool a     = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A);
        bool d     = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D);
        bool space = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space);
        bool shift = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift);

        car.update(w, s, a, d, space, shift, dt);

        if (sendClock.getElapsedTime().asMilliseconds() >= send_ms) {
            send_swarm(hub, robot_id, car.motorL(), car.motorR());
            sendClock.restart();
        }

        // ═══ ZEICHNEN ═══════════════════════════════════════
        win.clear(C::bg);

        // Dezente Scanlines fuer Retro-Feeling
        for (int y = 0; y < (int)ws.y; y += 4) {
            sf::RectangleShape scanline({(float)ws.x, 1});
            scanline.setPosition(sf::Vector2f(0, y));
            scanline.setFillColor(sf::Color(0, 0, 0, 12));
            win.draw(scanline);
        }

        if (hf) {
            draw_tacho(win, font, car, S(170.f, 240.f), Sf(130.f));
            draw_boost(win, font, car, S(80.f, 415.f), Sf(180.f));
            draw_trail(win, font, car, S(460.f, 220.f), Sf(110.f));
            draw_gforce(win, font, car, S(640.f, 140.f), Sf(48.f));
            draw_drift_badge(win, font, car.drifting, S(460.f, 370.f));

            // Motor Output
            sf::RectangleShape sep({Sf(1.5f), Sf(300.f)});
            sep.setOrigin(sf::Vector2f(0, Sf(150.f)));
            sep.setPosition(S(710.f, 260.f));
            sep.setFillColor(C::border);
            win.draw(sep);

            draw_motor(win, font, car.leftMotor,  S(740.f, 220.f), "L");
            draw_motor(win, font, car.rightMotor,  S(775.f, 220.f), "R");

            sf::Text oLbl(font, "OUTPUT", 9);
            oLbl.setOrigin(sf::Vector2f(oLbl.getLocalBounds().size.x/2, oLbl.getLocalBounds().size.y));
            oLbl.setPosition(S(757.f, 140.f));
            oLbl.setFillColor(C::textDim);
            win.draw(oLbl);

            // Tasten
            float kx = 460.f, ky = 470.f;
            draw_key(win, font, S(kx, ky-36),  "W",  w,     C::green);
            draw_key(win, font, S(kx-36, ky),   "A",  a,     C::cyan);
            draw_key(win, font, S(kx, ky),       "S",  s,     C::red);
            draw_key(win, font, S(kx+36, ky),    "D",  d,     C::cyan);
            draw_key(win, font, S(kx+90, ky),    "SPC", space, C::orange, 50, 32);
            draw_key(win, font, S(kx+148, ky),   "SHF", shift, C::hotPink, 50, 32);

            // Status
            sf::Text addr(font, "swarm_hub", 9);
            addr.setPosition(S(12.f, 10.f));
            addr.setFillColor(C::textDim);
            win.draw(addr);

            sf::CircleShape dot(4);
            dot.setOrigin(sf::Vector2f(4, 4));
            dot.setPosition(sf::Vector2f(S(12.f, 10.f).x + addr.getLocalBounds().size.x + 12, S(0.f, 17.f).y));
            dot.setFillColor(hub.is_open() ? C::green : C::red);
            win.draw(dot);

            char info[64];
            snprintf(info, sizeof(info), "%dms   ID:%d   60B", send_ms, robot_id);
            sf::Text inf(font, info, 9);
            inf.setPosition(sf::Vector2f(dot.getPosition().x + 12, S(0.f, 10.f).y));
            inf.setFillColor(C::textDim);
            win.draw(inf);

            sf::Text title(font, "ROBOT RACER", 14);
            title.setPosition(sf::Vector2f(S(800.f, 8.f).x - title.getLocalBounds().size.x - 14, S(0.f, 8.f).y));
            title.setFillColor(C::hotPink);
            win.draw(title);

            sf::Text fsHint(font, "F11: Fullscreen", 8);
            fsHint.setPosition(sf::Vector2f(S(800.f, 8.f).x - fsHint.getLocalBounds().size.x - 14, S(0.f, 26.f).y));
            fsHint.setFillColor(C::alpha(C::textDim, 120));
            win.draw(fsHint);
        }

        win.display();
    }

    send_swarm(hub, robot_id, 0, 0);
    return 0;
}