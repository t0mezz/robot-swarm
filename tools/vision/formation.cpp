// formation.cpp — Save named robot formations, then drive the swarm into them.
//
// Usage:
//   ./formation                      pick a formation from a CLI menu, run headless
//   ./formation --list               list known formations and exit
//   ./formation --add NAME           open the placement frontend, save slots as NAME
//   ./formation --run NAME           run NAME directly — no menu, no window
//   ./formation --frontend           force the camera window during a run
//   ./formation --delete NAME        remove a formation from the library
//
//   --hold        keep station-keeping after the formation is reached (default:
//                 stop the motors and exit once every robot has settled)
//   --speed PCT   global speed scale        --calibrate  redo the homography
//   --serial SN / --ip IP                   camera selection
//   --lib PATH    use an alternate formation library file
//
// Placement frontend controls (--add):
//   left-click   place a slot        right-click  remove the slot under cursor
//   u = undo last   x = clear all    ENTER = save   ESC/q = cancel
//
// A formation is just a named list of world-space (mm) slot positions. Which
// robot goes to which slot is NOT stored — it is solved at run time from where
// the robots actually are (see assignSlots).

#include "aruco_tracker.h"
#include "SwarmClient.h"
#include "DemoHud.h"
#include "goto_controller.h"
#include "avoidance.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <thread>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

// ── Tunables ──────────────────────────────────────────────────────────────────
//
// Gains and avoidance parameters are lifted from drag_drop_demo.cpp: the motion
// problem is the same one (each robot has a single terminal goal it should ease
// into), so the tuning transfers. See lib/SwarmControl/goto_controller.h and
// avoidance.h for what each knob actually does before changing any of them —
// MAX_SPEED / MAX_TURN / DANGER_MM in particular are three ends of one
// inequality and cannot be moved independently.
static constexpr float K_DIST        = 0.40f;
static constexpr float K_ANGLE       = 0.50f;
static constexpr float K_YAW_D       = 0.08f;
static constexpr float MAX_SPEED     = 90.0f;
static constexpr float MAX_TURN      = 24.0f;
static constexpr float ARRIVAL_MM    = 20.0f;
static constexpr float SEND_INT_S    = 0.05f;
static constexpr float YAW_TAU_S     = 0.70f;
static constexpr float DANGER_MM     = 500.0f;
static constexpr float SAFE_MM       = 1000.0f;
static constexpr float AVOID_BLEND   = 0.95f;
static constexpr float DODGE_SPEED_FRAC = 0.70f;

static constexpr float SLOT_RADIUS_PX = 30.0f;  // pixel hit radius for slot removal
static constexpr int   MAX_ROBOTS     = 32;

// How long every robot must sit inside ARRIVAL_MM before the formation counts as
// reached. Without a dwell requirement a single frame of marker jitter that
// happens to land everyone inside the radius ends the run early, mid-approach.
static constexpr float SETTLE_S       = 1.0f;

// Re-solving the robot→slot assignment on every frame would let a robot swap
// targets mid-drive whenever two costs cross, so it is only redone when the set
// of visible robots changes — and then only after this long, since markers drop
// out for a frame or two routinely and each blink would otherwise reshuffle the
// whole swarm.
static constexpr float REASSIGN_DEBOUNCE_S = 0.75f;

using swarmctl::normAngle;
using swarmctl::clampf;

static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";

// ── Globals ───────────────────────────────────────────────────────────────────

static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

static cv::Mat g_H;
static bool    g_hasH = false;

static std::unordered_map<int, RobotPose> g_poses;

// ── World coordinate helpers ──────────────────────────────────────────────────

static cv::Point2f pixelToWorld(cv::Point2f px) {
    if (!g_hasH) return px;
    std::vector<cv::Point2f> src = {px}, dst;
    cv::perspectiveTransform(src, dst, g_H);
    return dst[0];
}
static cv::Point2f worldToPixel(cv::Point2f w) {
    if (!g_hasH) return w;
    cv::Mat Hinv = g_H.inv();
    std::vector<cv::Point2f> src = {w}, dst;
    cv::perspectiveTransform(src, dst, Hinv);
    return dst[0];
}

// ── Formation library ─────────────────────────────────────────────────────────
//
// Stored under ~/.config, not /tmp like the homography and the shape_demo
// scratch file: those are per-session artifacts that are cheap to regenerate,
// whereas a formation is hand-placed by the user and losing it on reboot would
// mean redoing that work.

struct Formation {
    std::string              name;
    std::vector<cv::Point2f> slots;   // world mm
};

static std::string defaultLibPath() {
    const char* home = getenv("HOME");
    if (!home) return "formations.yml";
    std::string dir = std::string(home) + "/.config/robot-swarm";
    mkdir((std::string(home) + "/.config").c_str(), 0755);
    mkdir(dir.c_str(), 0755);
    return dir + "/formations.yml";
}

static std::vector<Formation> loadLibrary(const std::string& path) {
    std::vector<Formation> out;
    // Probed with access() first: having no library yet is the normal first-run
    // state, and handing a missing path to FileStorage makes OpenCV print an
    // [ERROR] banner for it, which reads like something broke.
    if (access(path.c_str(), R_OK) != 0) return out;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return out;
    cv::FileNode node = fs["formations"];
    for (auto it = node.begin(); it != node.end(); ++it) {
        Formation f;
        (*it)["name"]  >> f.name;
        (*it)["slots"] >> f.slots;
        if (!f.name.empty() && !f.slots.empty()) out.push_back(std::move(f));
    }
    return out;
}

static bool saveLibrary(const std::string& path, const std::vector<Formation>& lib) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        fprintf(stderr, "[formation] cannot write %s\n", path.c_str());
        return false;
    }
    fs << "formations" << "[";
    for (const auto& f : lib)
        fs << "{" << "name" << f.name << "slots" << f.slots << "}";
    fs << "]";
    return true;
}

static const Formation* findFormation(const std::vector<Formation>& lib,
                                      const std::string& name) {
    for (const auto& f : lib) if (f.name == name) return &f;
    return nullptr;
}

// Bounding-box span, printed by --list so a formation can be sanity-checked
// against the arena size without opening the frontend.
static void formationExtent(const Formation& f, float& w, float& h) {
    float x0 = f.slots[0].x, x1 = x0, y0 = f.slots[0].y, y1 = y0;
    for (const auto& p : f.slots) {
        x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
        y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
    }
    w = x1 - x0; h = y1 - y0;
}

// ── Robot → slot assignment ───────────────────────────────────────────────────
//
// Solved as a rectangular linear assignment problem (Hungarian / Jonker-Volgenant
// shortest augmenting path, O(n³)) rather than by greedy nearest-slot matching.
//
// Greedy is not merely suboptimal here, it is visibly wrong: it hands the best
// slot to whichever robot is considered first, and the robots left over get the
// leftovers, which routinely means two robots driving across each other's path
// to reach slots the other one was already next to. Crossing paths are exactly
// what the avoidance engine then has to untangle, so a bad assignment converts
// directly into dodging, stalling and a slower, messier formation.
//
// The cost is plain Euclidean distance, NOT squared distance, and the
// difference is not cosmetic — a sum-of-distances optimum is provably
// non-crossing, and a squared one is not.
//
// If two assignments a→c and b→d cross at p, swapping them to a→d, b→c changes
// the cost by (with w=|ap| x=|pc| y=|bp| z=|pd|):
//   linear : |ad|+|bc| <= (w+z)+(y+x) = (w+x)+(y+z) = |ac|+|bd|  — never worse,
//            by the triangle inequality, so no optimum can contain a crossing.
//   squared: the same swap changes cost by 2(w-y)(z-x), which is positive for
//            half of all geometries — so crossings survive into the optimum.
//
// That is not theoretical. Over 20k random equal-count layouts, squared cost
// left a crossing in 15.9% of formations versus 0.000% for linear, and bought
// only ~8% shorter worst-case travel for it (1089mm vs 1174mm mean). A crossing
// is much more expensive than that 8%: it puts two robots on a collision course,
// which latches the avoidance engine, drops the dodger to dodgeSpeedFrac and
// sends it on a detour — and can escalate to an emergency stop. Paying 8% more
// travel to remove one conflict in six is a trade worth making every time.
//
// n = robots, m = slots; the two need not be equal. The matrix is padded to
// square with zero-cost dummies, so with more robots than slots the surplus
// robots come back unassigned (-1) and are left parked, and with more slots than
// robots the surplus slots simply stay empty.
static std::vector<int> assignSlots(const std::vector<cv::Point2f>& from,
                                    const std::vector<cv::Point2f>& to)
{
    const int n = (int)from.size(), m = (int)to.size();
    if (n == 0 || m == 0) return std::vector<int>(n, -1);

    const int N = std::max(n, m);
    const float INF = 1e30f;

    // 1-indexed cost matrix, padded to N×N with zero-cost dummy rows/cols.
    std::vector<std::vector<float>> a(N + 1, std::vector<float>(N + 1, 0.f));
    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= m; j++) {
            const float dx = from[i-1].x - to[j-1].x;
            const float dy = from[i-1].y - to[j-1].y;
            a[i][j] = std::sqrt(dx*dx + dy*dy);
        }
    }

    std::vector<float> u(N + 1, 0.f), v(N + 1, 0.f);
    std::vector<int>   p(N + 1, 0), way(N + 1, 0);   // p[j] = row matched to col j

    for (int i = 1; i <= N; i++) {
        p[0] = i;
        int j0 = 0;
        std::vector<float> minv(N + 1, INF);
        std::vector<char>  used(N + 1, false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            int   j1 = -1;
            float delta = INF;
            for (int j = 1; j <= N; j++) {
                if (used[j]) continue;
                const float cur = a[i0][j] - u[i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            for (int j = 0; j <= N; j++) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else         { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do { const int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
    }

    std::vector<int> slotOf(n, -1);
    for (int j = 1; j <= N; j++) {
        const int i = p[j];
        if (i >= 1 && i <= n && j <= m) slotOf[i-1] = j - 1;   // drop dummy pairings
    }
    return slotOf;
}

// ── Avoidance ─────────────────────────────────────────────────────────────────
// Pair scan, priority rules and detour math live in lib/SwarmControl/avoidance.h.
// This tool supplies only its own policy: what counts as "moving" (has a slot,
// and is not yet within the arrival radius).
//
// The engine is stateful — it latches each dodge so the maneuver holds instead
// of cancelling itself — so it must persist across frames, not be rebuilt.

static const swarmctl::AvoidParams AVOID_PARAMS = {
    .dangerMm = DANGER_MM,
    .safeMm   = SAFE_MM,
    .blend    = AVOID_BLEND,
    .faceDot  = 0.25f,
    .dodgeSpeedFrac = DODGE_SPEED_FRAC,
    .emergencyMm = 240.0f,
};

static swarmctl::AvoidanceEngine g_avoid(AVOID_PARAMS);

// ── Placement frontend (--add) ────────────────────────────────────────────────

struct PlaceState {
    std::vector<cv::Point2f> slotsPx;   // kept in pixels while placing so the
                                        // markers track the frame exactly
    bool done      = false;
    bool cancelled = false;
};

static void onPlaceMouse(int event, int x, int y, int, void* ud) {
    auto* s = (PlaceState*)ud;
    cv::Point2f px((float)x, (float)y);

    if (event == cv::EVENT_LBUTTONDOWN) {
        s->slotsPx.push_back(px);
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        int   best = -1;
        float bestD = SLOT_RADIUS_PX;
        for (int i = 0; i < (int)s->slotsPx.size(); i++) {
            const float d = (float)cv::norm(s->slotsPx[i] - px);
            if (d < bestD) { bestD = d; best = i; }
        }
        if (best >= 0) s->slotsPx.erase(s->slotsPx.begin() + best);
    }
}

// ── Calibration ───────────────────────────────────────────────────────────────

struct CalibState { std::vector<cv::Point2f> pixPts; bool done = false; };
static void onCalibMouse(int event, int x, int y, int, void* ud) {
    auto* s = (CalibState*)ud;
    if (event == cv::EVENT_LBUTTONDOWN && s->pixPts.size() < 4) {
        s->pixPts.push_back({(float)x, (float)y});
        printf("  corner %d: (%d, %d)\n", (int)s->pixPts.size(), x, y);
        if (s->pixPts.size() == 4) s->done = true;
    }
}

static bool runCalibration(ArucoTracker& tracker, const char* win) {
    printf("\nCalibration: click 4 corners TL TR BR BL\n");
    for (int attempts = 0; !tracker.update(); ) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (++attempts > 100) { fprintf(stderr, "[calib] timeout\n"); return false; }
    }
    cv::Mat frame = tracker.debugFrame().clone();
    CalibState cs;
    cv::setMouseCallback(win, onCalibMouse, &cs);
    while (!cs.done) {
        cv::Mat disp = frame.clone();
        for (auto& p : cs.pixPts) cv::circle(disp, p, 8, {0,0,255}, -1);
        ArucoTracker::drawText(disp,
            "Click corners TL TR BR BL  " + std::to_string(cs.pixPts.size()) + "/4",
            {10, 40}, 20, {0, 255, 0});
        cv::imshow(win, disp);
        if (cv::waitKey(30) == 27) { cv::setMouseCallback(win, nullptr, nullptr); return false; }
    }
    cv::setMouseCallback(win, nullptr, nullptr);

    printf("Arena width height mm (e.g. 800 600): ");
    float W = 0, H = 0;
    if (scanf("%f %f", &W, &H) != 2 || W <= 0 || H <= 0) return false;

    std::vector<cv::Point2f> worldPts = {{0,0},{W,0},{W,H},{0,H}};
    g_H    = cv::findHomography(cs.pixPts, worldPts);
    g_hasH = !g_H.empty();
    if (g_hasH) {
        tracker.setHomography(cs.pixPts, worldPts);
        tracker.saveHomography(HOMOGRAPHY_FILE);
        printf("Saved homography.\n");
    }
    return g_hasH;
}

// ── Camera bring-up ───────────────────────────────────────────────────────────
//
// Shared by --add and the run loop: both need the tracker and the same
// homography, and neither can do anything useful without world coordinates.
static bool openTracker(ArucoTracker& tracker) {
    if (!tracker.open()) { fprintf(stderr, "Could not open camera.\n"); return false; }
    printf("Camera: %dx%d\n", tracker.frameSize().width, tracker.frameSize().height);
    return true;
}

static void loadHomographyInto(ArucoTracker& tracker) {
    if (!tracker.loadHomography(HOMOGRAPHY_FILE)) return;
    cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
    if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
    if (g_hasH) printf("Loaded homography.\n");
}

// ── HUD ───────────────────────────────────────────────────────────────────────

static void drawTelHud(cv::Mat& disp,
    const std::unordered_map<int, swarmctl::AvoidState>& avoidance,
    const std::unordered_map<int, cv::Point2f>& goals,
    const int8_t motors[][2], SwarmClient& swarm,
    const std::string& formName, int slotCount, int settled, float fps)
{
    DemoHud hud;
    hud.title(DemoHud::fmt(
        "loop_fps:%.0f  Formation:%s  Slots:%d  InPlace:%d/%d  HUB:%s",
        fps, formName.c_str(), slotCount, settled, (int)goals.size(),
        swarm.isConnected() ? "OK" : "OFFLINE"),
        swarm.isConnected() ? DemoHud::COL_OK : DemoHud::COL_BAD);
    hud.header({"ID", "Vision", "Battery", "Latency", "Mot-L", "Mot-R", "Status"});

    std::set<int> allIds;
    for (auto& [id, _] : g_poses) allIds.insert(id);
    for (int id : swarm.knownIds()) allIds.insert(id);

    for (int id : allIds) {
        const auto& ss = swarm.robotState((uint8_t)id);
        const bool vis   = g_poses.count(id) > 0;
        const bool dodge = vis && avoidance.count(id) && avoidance.at(id).dodging;
        const bool prio  = vis && avoidance.count(id) && avoidance.at(id).priority && !dodge;

        bool arrived = false;
        if (vis && goals.count(id)) {
            const auto& p = g_poses.at(id);
            const auto& g = goals.at(id);
            arrived = sqrtf((g.x-p.x)*(g.x-p.x) + (g.y-p.y)*(g.y-p.y)) < ARRIVAL_MM;
        }

        cv::Scalar col = !vis ? (ss.known ? DemoHud::COL_WARN : DemoHud::COL_TEXT)
                              : dodge ? DemoHud::COL_WARN : DemoHud::COL_OK;

        // DODGE / HOLD are the two halves of one conflict — seeing which robot
        // got which is the fastest way to tell whether the nomination was sane.
        const char* status = !vis ? (ss.known ? "RADIO" : "UNSEEN")
                           : dodge   ? "DODGE"
                           : prio    ? "HOLD"
                           : arrived ? "IN-PLACE"
                           : goals.count(id) ? "MOVING" : "SPARE";

        hud.row({
            DemoHud::fmt("%d", id),
            vis ? "YES" : "NO",
            ss.known ? DemoHud::formatBattery(ss.battery) : "--",
            ss.known ? DemoHud::formatLatency(ss.latencyUs) : "--",
            vis ? DemoHud::fmt("%+d", (int)motors[id][0]) : "--",
            vis ? DemoHud::fmt("%+d", (int)motors[id][1]) : "--",
            status,
        }, col);
    }
    hud.drawTopRight(disp);
}

// ── Mode: --list ──────────────────────────────────────────────────────────────

static int cmdList(const std::vector<Formation>& lib, const std::string& libPath) {
    if (lib.empty()) {
        printf("No formations in %s\n", libPath.c_str());
        printf("Create one with:  ./formation --add NAME\n");
        return 0;
    }
    printf("Formations in %s:\n\n", libPath.c_str());
    printf("  %-20s %6s  %s\n", "NAME", "SLOTS", "EXTENT (mm)");
    for (const auto& f : lib) {
        float w, h; formationExtent(f, w, h);
        printf("  %-20s %6d  %.0f x %.0f\n", f.name.c_str(), (int)f.slots.size(), w, h);
    }
    printf("\n");
    return 0;
}

// ── Mode: --add ───────────────────────────────────────────────────────────────

static int cmdAdd(const std::string& name, std::vector<Formation>& lib,
                  const std::string& libPath, ArucoTracker& tracker, bool doCalib)
{
    if (!openTracker(tracker)) return 1;

    // ASCII only. Qt highgui round-trips the window title through a different
    // string conversion in namedWindow() than in setMouseCallback(), so a
    // non-ASCII byte here makes the lookup miss and abort with a NULL window
    // handler — an em-dash in this name is exactly how that was found.
    const char* WIN = "Formation - place slots";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);

    if (!doCalib) loadHomographyInto(tracker);
    if (doCalib || !g_hasH) {
        if (!runCalibration(tracker, WIN)) {
            // Slots are saved in world mm, so without a homography every
            // position placed here would be meaningless the moment the camera
            // moved. Refuse rather than write a formation that cannot be run.
            fprintf(stderr, "[formation] calibration required to place slots.\n");
            return 1;
        }
    }

    PlaceState ps;
    cv::setMouseCallback(WIN, onPlaceMouse, &ps);

    printf("\nPlacing '%s'.  left-click = add slot   right-click = remove\n"
           "u = undo   x = clear   ENTER = save   ESC = cancel\n\n", name.c_str());

    while (g_running && !ps.done && !ps.cancelled) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        cv::Mat disp = tracker.debugFrame().clone();

        // Slots, numbered in placement order. The numbers are cosmetic — slot
        // order carries no meaning at run time, since assignSlots re-derives
        // which robot takes which — but they make it easy to talk about a
        // specific one while placing.
        for (int i = 0; i < (int)ps.slotsPx.size(); i++) {
            const cv::Point2f& p = ps.slotsPx[i];
            cv::circle(disp, p, 12, {0,200,255}, 2, cv::LINE_AA);
            cv::line(disp, p - cv::Point2f(16,0), p + cv::Point2f(16,0), {0,200,255}, 1, cv::LINE_AA);
            cv::line(disp, p - cv::Point2f(0,16), p + cv::Point2f(0,16), {0,200,255}, 1, cv::LINE_AA);
            ArucoTracker::drawText(disp, std::to_string(i + 1),
                                   cv::Point2f(p.x + 16, p.y - 12), 16, {0,200,255});
        }

        DemoHud hud;
        hud.title(DemoHud::fmt("PLACING '%s'  Slots:%d  Robots seen:%d",
                               name.c_str(), (int)ps.slotsPx.size(),
                               (int)tracker.robots().size()), DemoHud::COL_WARN);
        hud.row("left-click", "add slot");
        hud.row("right-click", "remove slot");
        hud.row("u / x", "undo / clear");
        hud.row("ENTER", "save");
        hud.row("ESC", "cancel");
        hud.drawTopRight(disp);

        cv::imshow(WIN, disp);
        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q')            ps.cancelled = true;
        if (key == 13 || key == 10)             ps.done = true;
        if (key == 'u' && !ps.slotsPx.empty())  ps.slotsPx.pop_back();
        if (key == 'x')                         ps.slotsPx.clear();
    }
    cv::setMouseCallback(WIN, nullptr, nullptr);
    cv::destroyWindow(WIN);

    if (ps.cancelled || !g_running) { printf("[formation] cancelled.\n"); return 1; }
    if (ps.slotsPx.empty())         { printf("[formation] no slots placed — nothing saved.\n"); return 1; }

    Formation f;
    f.name = name;
    for (const auto& p : ps.slotsPx) f.slots.push_back(pixelToWorld(p));

    // Replace in place if the name already exists, so --add doubles as an edit.
    bool replaced = false;
    for (auto& existing : lib) {
        if (existing.name == name) { existing = f; replaced = true; break; }
    }
    if (!replaced) lib.push_back(f);

    if (!saveLibrary(libPath, lib)) return 1;
    printf("[formation] %s '%s' with %d slot(s).\n",
           replaced ? "Updated" : "Saved", name.c_str(), (int)f.slots.size());
    return 0;
}

// ── Mode: CLI picker ──────────────────────────────────────────────────────────

static const Formation* pickFormation(const std::vector<Formation>& lib) {
    printf("\nFormations:\n\n");
    for (int i = 0; i < (int)lib.size(); i++) {
        float w, h; formationExtent(lib[i], w, h);
        printf("  [%d] %-20s %2d slots   %.0f x %.0f mm\n",
               i + 1, lib[i].name.c_str(), (int)lib[i].slots.size(), w, h);
    }
    printf("\nSelect [1-%d, q to quit]: ", (int)lib.size());
    fflush(stdout);

    char buf[64];
    if (!fgets(buf, sizeof(buf), stdin)) return nullptr;
    if (buf[0] == 'q' || buf[0] == '\n') return nullptr;

    // Accept a name as readily as an index — typing "wedge" is the obvious thing
    // to try, and failing that with "invalid selection" would be needless.
    char* end = nullptr;
    const long idx = strtol(buf, &end, 10);
    if (end != buf && idx >= 1 && idx <= (long)lib.size()) return &lib[idx - 1];

    std::string name(buf);
    while (!name.empty() && (name.back() == '\n' || name.back() == '\r')) name.pop_back();
    const Formation* f = findFormation(lib, name);
    if (!f) fprintf(stderr, "[formation] no such formation: %s\n", name.c_str());
    return f;
}

// ── Mode: run ─────────────────────────────────────────────────────────────────

static int cmdRun(const Formation& form, ArucoTracker& tracker,
                  bool frontend, bool doCalib, bool hold, float speedPct)
{
    if (!openTracker(tracker)) return 1;

    const char* WIN = "Formation";
    if (frontend) {
        cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
        cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
    }

    if (!doCalib) loadHomographyInto(tracker);
    if (doCalib) {
        if (!frontend) {
            fprintf(stderr, "[formation] --calibrate needs a window; add --frontend.\n");
            return 1;
        }
        runCalibration(tracker, WIN);
    }
    if (!g_hasH) {
        // Slots are world mm. Running without a homography would compare them to
        // pixel poses and drive the swarm somewhere arbitrary.
        fprintf(stderr, "[formation] no homography — run --add or --calibrate first.\n");
        return 1;
    }

    SwarmClient swarm;
    if (swarm.connect()) printf("[hub] Connected.\n");
    else                 printf("[hub] Not available — will retry.\n");

    const float speedScale = clampf(speedPct / 100.f, 0.05f, 2.f);

    swarmctl::YawSmoother smoothedYaw(YAW_TAU_S);
    std::unordered_map<int, swarmctl::GotoState> gotoState;
    std::unordered_map<int, cv::Point2f>         goals;     // id → assigned slot

    const swarmctl::GotoParams GOTO_PARAMS = [] {
        swarmctl::GotoParams p;
        p.kDist = K_DIST; p.kAngle = K_ANGLE; p.kYawD = K_YAW_D;
        p.maxTurn = MAX_TURN; p.arrivalMm = ARRIVAL_MM;
        p.brake = true;   // single terminal goal — ease into it
        return p;
    }();

    int8_t motors[MAX_ROBOTS][2]     = {};
    int8_t lastMotors[MAX_ROBOTS][2] = {};

    auto t0        = std::chrono::steady_clock::now();
    auto lastSend  = t0;
    auto lastFpsT  = t0;
    auto lastCtrlT = t0;
    auto lastRetry = t0 - std::chrono::seconds(10);
    auto settledAt = t0;
    int   frameCount = 0;
    float fps        = 0.f;
    bool  everSettled = false;

    std::set<int> lastVisible;
    auto  visibleChangedAt = t0;
    bool  pendingReassign  = true;

    printf("\n[formation] Running '%s' (%d slots)%s. Ctrl-C to stop.\n\n",
           form.name.c_str(), (int)form.slots.size(), hold ? ", holding station" : "");

    while (g_running) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const auto now = std::chrono::steady_clock::now();
        ++frameCount;
        const float dt        = std::chrono::duration<float>(now - lastCtrlT).count();
        const float controlDt = clampf(dt, 0.01f, 0.2f);
        lastCtrlT = now;

        if (!swarm.isConnected() &&
            std::chrono::duration<float>(now - lastRetry).count() >= 2.f) {
            lastRetry = now;
            if (swarm.connect()) printf("[hub] Connected.\n");
        }
        swarm.poll();

        const float fpsDt = std::chrono::duration<float>(now - lastFpsT).count();
        if (fpsDt >= 1.f) { fps = frameCount / fpsDt; frameCount = 0; lastFpsT = now; }

        // ── Pose map (smoothed yaw) ──────────────────────────────────────────
        g_poses.clear();
        for (auto& r : tracker.robots()) {
            if (r.id < 0 || r.id >= MAX_ROBOTS) continue;
            RobotPose pose = r;
            pose.yaw = smoothedYaw.update(r.id, r.yaw, controlDt);
            g_poses[r.id] = pose;
            swarm.registerRobot((uint8_t)r.id);
        }

        // ── Assignment ───────────────────────────────────────────────────────
        // Debounced on a change in the visible set: markers blink out for a
        // frame or two routinely, and reshuffling the whole swarm on every
        // blink would keep it permanently re-planning instead of driving.
        std::set<int> visible;
        for (auto& [id, _] : g_poses) visible.insert(id);
        if (visible != lastVisible) {
            lastVisible      = visible;
            visibleChangedAt = now;
            pendingReassign  = true;
        }
        if (pendingReassign && !visible.empty() &&
            std::chrono::duration<float>(now - visibleChangedAt).count() >= REASSIGN_DEBOUNCE_S) {
            std::vector<int>         ids;
            std::vector<cv::Point2f> pts;
            for (int id : visible) {
                ids.push_back(id);
                pts.push_back({g_poses.at(id).x, g_poses.at(id).y});
            }
            const std::vector<int> slotOf = assignSlots(pts, form.slots);

            goals.clear();
            for (int i = 0; i < (int)ids.size(); i++) {
                if (slotOf[i] < 0) continue;               // surplus robot: no slot
                goals[ids[i]] = form.slots[slotOf[i]];
            }
            pendingReassign = false;
            everSettled     = false;                        // re-earn the dwell
            printf("[formation] Assigned %d robot(s) to %d slot(s).\n",
                   (int)goals.size(), (int)form.slots.size());
            if ((int)ids.size() > (int)form.slots.size())
                printf("[formation] %d robot(s) have no slot and will stay parked.\n",
                       (int)ids.size() - (int)form.slots.size());
        }

        // ── Avoidance ────────────────────────────────────────────────────────
        // Called unconditionally, even with 0/1 robots visible: the engine holds
        // latch state, and skipping the call would strand stale dodges when
        // robots drop out of tracking.
        auto isMoving = [&](int id) -> bool {
            auto git = goals.find(id);
            if (git == goals.end()) return false;
            const auto& p = g_poses.at(id);
            const float dx = git->second.x - p.x, dy = git->second.y - p.y;
            return sqrtf(dx*dx + dy*dy) > ARRIVAL_MM;
        };
        auto avoidance = g_avoid.update(g_poses, isMoving, controlDt);

        // ── Motor control ────────────────────────────────────────────────────
        memcpy(motors, lastMotors, sizeof(motors));

        int settled = 0;
        for (auto& [id, pose] : g_poses) {
            auto& av = avoidance[id];
            const auto act = swarmctl::applyAvoidance(av, MAX_SPEED * speedScale,
                                                      AVOID_PARAMS);

            if (act.hardStop) { motors[id][0] = motors[id][1] = 0; continue; }

            auto git = goals.find(id);
            if (git == goals.end()) { motors[id][0] = motors[id][1] = 0; continue; }

            float dx = git->second.x - pose.x;
            float dy = git->second.y - pose.y;
            const float dist = sqrtf(dx*dx + dy*dy);

            if (dist < ARRIVAL_MM) {
                motors[id][0] = motors[id][1] = 0;
                settled++;
                continue;
            }

            // Bend toward arc detour direction
            dx += act.arcX * dist * act.arcBlend;
            dy += act.arcY * dist * act.arcBlend;

            const auto cmd = swarmctl::computeGoto(dx, dy, pose.yaw, act.maxSpd,
                                                   GOTO_PARAMS, gotoState[id], controlDt);
            motors[id][0] = cmd.left;
            motors[id][1] = cmd.right;
        }

        // Silence robots not seen
        for (int id = 0; id < MAX_ROBOTS; id++)
            if (!g_poses.count(id)) motors[id][0] = motors[id][1] = 0;

        if (std::chrono::duration<float>(now - lastSend).count() >= SEND_INT_S) {
            for (int id = 0; id < MAX_ROBOTS; id++)
                swarm.setSpeed((uint8_t)id, motors[id][0], motors[id][1]);
            swarm.flush();
            lastSend = now;
            memcpy(lastMotors, motors, sizeof(motors));
        }

        // ── Completion ───────────────────────────────────────────────────────
        // Everyone with a slot is in it. Requires SETTLE_S of dwell so a single
        // jittery frame cannot end the run mid-approach.
        const bool allIn = !goals.empty() && settled == (int)goals.size() && !pendingReassign;
        if (allIn) {
            if (!everSettled) { everSettled = true; settledAt = now; }
            if (!hold && std::chrono::duration<float>(now - settledAt).count() >= SETTLE_S) {
                printf("[formation] '%s' reached — %d robot(s) in place.\n",
                       form.name.c_str(), settled);
                break;
            }
        } else {
            everSettled = false;
        }

        // ── Draw ─────────────────────────────────────────────────────────────
        if (frontend) {
            cv::Mat disp = tracker.debugFrame().clone();

            // Every slot, whether or not it is claimed — an empty slot is the
            // fastest way to see that the swarm is short a robot.
            for (int i = 0; i < (int)form.slots.size(); i++) {
                const cv::Point2f sp = worldToPixel(form.slots[i]);
                bool claimed = false, occupied = false;
                for (auto& [id, g] : goals) {
                    if (cv::norm(g - form.slots[i]) > 1e-3) continue;
                    claimed = true;
                    if (g_poses.count(id)) {
                        const auto& p = g_poses.at(id);
                        occupied = sqrtf((g.x-p.x)*(g.x-p.x) + (g.y-p.y)*(g.y-p.y)) < ARRIVAL_MM;
                    }
                    break;
                }
                const cv::Scalar col = occupied ? cv::Scalar(0,255,80)
                                     : claimed  ? cv::Scalar(0,200,255)
                                                : cv::Scalar(90,90,90);
                cv::circle(disp, sp, 12, col, 2, cv::LINE_AA);
                cv::line(disp, sp - cv::Point2f(16,0), sp + cv::Point2f(16,0), col, 1, cv::LINE_AA);
                cv::line(disp, sp - cv::Point2f(0,16), sp + cv::Point2f(0,16), col, 1, cv::LINE_AA);
                ArucoTracker::drawText(disp, std::to_string(i + 1),
                                       cv::Point2f(sp.x + 16, sp.y - 12), 15, col);
            }

            // Assignment lines — the whole point of the Hungarian solve is that
            // these do not cross, which is only checkable by looking at them.
            for (auto& [id, g] : goals) {
                if (!g_poses.count(id)) continue;
                const auto& pose = g_poses.at(id);
                const cv::Point2f gp = worldToPixel(g);
                const float d = sqrtf((g.x-pose.x)*(g.x-pose.x) + (g.y-pose.y)*(g.y-pose.y));
                if (d < ARRIVAL_MM) continue;
                cv::arrowedLine(disp, {(int)pose.px,(int)pose.py}, gp,
                                {0,200,255}, 2, cv::LINE_AA, 0, 0.12);
            }

            for (auto& [id, pose] : g_poses) {
                if (avoidance.count(id) && avoidance.at(id).dodging)
                    cv::circle(disp, {(int)pose.px,(int)pose.py}, 28, {0,60,255}, 2, cv::LINE_AA);
            }

            drawTelHud(disp, avoidance, goals, motors, swarm,
                       form.name, (int)form.slots.size(), settled, fps);
            cv::imshow(WIN, disp);

            const int key = cv::waitKey(1);
            if (key == 'q' || key == 27) g_running = false;
            if (key == 's') {
                goals.clear();
                memset(motors, 0, sizeof(motors));
                swarm.stopAll(); swarm.flush();
                printf("[formation] All stopped.\n");
            }
            if (key == 'r') pendingReassign = true;
        }
    }

    for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed((uint8_t)id, 0, 0);
    swarm.flush();
    return 0;
}

// ── Main ──────────────────────────────────────────────────────────────────────

static void usage() {
    printf(
      "Usage: formation [options]\n\n"
      "  --list             list known formations and exit\n"
      "  --add NAME         place slots in the camera view and save as NAME\n"
      "  --run NAME         run NAME directly (no menu, no window)\n"
      "  --delete NAME      remove NAME from the library\n"
      "  --frontend         show the camera window while running\n"
      "  --hold             keep station-keeping instead of exiting when formed\n"
      "  --speed PCT        global speed scale (default 40)\n"
      "  --calibrate        redo the arena homography\n"
      "  --lib PATH         alternate formation library file\n"
      "  --serial SN | --ip IP    camera selection\n\n"
      "With no mode flag: pick a formation from a menu and run it headless.\n");
}

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    std::string serial, ip, libPath, addName, runName, delName;
    float speedPct = 40.f;
    bool  doList = false, frontend = false, doCalib = false, hold = false;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--serial")  && i+1<argc) serial   = argv[++i];
        else if (!strcmp(argv[i], "--ip")      && i+1<argc) ip       = argv[++i];
        else if (!strcmp(argv[i], "--lib")     && i+1<argc) libPath  = argv[++i];
        else if (!strcmp(argv[i], "--add")     && i+1<argc) addName  = argv[++i];
        else if (!strcmp(argv[i], "--run")     && i+1<argc) runName  = argv[++i];
        else if (!strcmp(argv[i], "--delete")  && i+1<argc) delName  = argv[++i];
        else if (!strcmp(argv[i], "--speed")   && i+1<argc) speedPct = atof(argv[++i]);
        else if (!strcmp(argv[i], "--list"))                doList   = true;
        else if (!strcmp(argv[i], "--frontend"))            frontend = true;
        else if (!strcmp(argv[i], "--calibrate"))           doCalib  = true;
        else if (!strcmp(argv[i], "--hold"))                hold     = true;
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) { usage(); return 0; }
        else { fprintf(stderr, "Unknown option: %s\n\n", argv[i]); usage(); return 1; }
    }

    if (libPath.empty()) libPath = defaultLibPath();
    std::vector<Formation> lib = loadLibrary(libPath);

    if (doList) return cmdList(lib, libPath);

    if (!delName.empty()) {
        const size_t before = lib.size();
        lib.erase(std::remove_if(lib.begin(), lib.end(),
                  [&](const Formation& f) { return f.name == delName; }), lib.end());
        if (lib.size() == before) {
            fprintf(stderr, "[formation] no such formation: %s\n", delName.c_str());
            return 1;
        }
        if (!saveLibrary(libPath, lib)) return 1;
        printf("[formation] Deleted '%s'.\n", delName.c_str());
        return 0;
    }

    // The camera is opened by whichever mode needs it, but the config is
    // assembled once so --serial/--ip apply uniformly.
    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;

    if (!addName.empty()) {
        cfg.debugOverlay = true;              // placement is always visual
        ArucoTracker tracker(cfg);
        return cmdAdd(addName, lib, libPath, tracker, doCalib);
    }

    // ── Run modes ────────────────────────────────────────────────────────────
    const Formation* form = nullptr;
    if (!runName.empty()) {
        form = findFormation(lib, runName);
        if (!form) {
            fprintf(stderr, "[formation] no such formation: %s\n", runName.c_str());
            fprintf(stderr, "Known: ");
            for (const auto& f : lib) fprintf(stderr, "%s ", f.name.c_str());
            fprintf(stderr, "\n");
            return 1;
        }
    } else {
        if (lib.empty()) {
            printf("No formations in %s\n", libPath.c_str());
            printf("Create one with:  ./formation --add NAME\n");
            return 1;
        }
        form = pickFormation(lib);
        if (!form) return 0;                  // user quit the menu
    }

    // The overlay is drawn into debugFrame() by the tracker; skip the work
    // entirely when there is no window to show it in.
    cfg.debugOverlay = frontend;
    ArucoTracker tracker(cfg);
    return cmdRun(*form, tracker, frontend, doCalib, hold, speedPct);
}
