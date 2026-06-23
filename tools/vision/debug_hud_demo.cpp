// debug_hud_demo.cpp — standalone visual test for lib/DebugHud/DebugHud.h.
//
// Renders the shared HUD panel against synthetic data only: no camera, no
// Basler/pylon dependency, no swarm_hub/SwarmClient connection. Useful for
// iterating on DebugHud layout/sizing/colors without hardware attached.
//
// Controls: q/Esc quit, +/- scale font, h toggle simulated hub state.

#include "DebugHud.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

int main() {
    const char* WIN = "DebugHud Demo";
    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);

    DebugHud::LoopFps loopFps;
    bool hubOk = true;
    int frame = 0;
    auto start = std::chrono::steady_clock::now();

    printf("DebugHud demo: q/Esc quit, h toggle simulated hub state.\n");

    while (true) {
        cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(40, 40, 40));
        loopFps.tick();
        ++frame;

        double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();

        // ── Single-line status HUD (vision_controller/circle_demo style) ──────
        {
            DebugHud hud;
            hud.title(DebugHud::fmt("loop_fps:%.0f  Robots:%d  HUB:%s  t:%.1fs",
                      loopFps.fps(), 4, hubOk ? "OK" : "INACTIVE", t),
                      hubOk ? DebugHud::COL_OK : DebugHud::COL_BAD);
            hud.draw(img, {10, img.rows - 36});
        }

        // ── Stacked-rows HUD (wingman style) ───────────────────────────────────
        {
            DebugHud hud;
            hud.title("WINGMAN FORMATION (synthetic)");
            hud.row("Leader", "Robot 2  (WASD)", DebugHud::COL_WARN);
            hud.row("Spacing", "180 mm   Robots: 4");
            hud.row("loop_fps", DebugHud::fmt("%.1f", loopFps.fps()), DebugHud::COL_OK);
            hud.row("Hub", hubOk ? "OK" : "DISCONNECTED",
                    hubOk ? DebugHud::COL_OK : DebugHud::COL_BAD);
            hud.draw(img, {10, 10});
        }

        // ── Tabular HUD (shape_demo/drag_drop_demo style) ──────────────────────
        {
            DebugHud hud;
            hud.title(DebugHud::fmt("loop_fps:%.0f  Robots:%d/4  HUB:%s",
                      loopFps.fps(), 3, hubOk ? "OK" : "OFFLINE"),
                      hubOk ? DebugHud::COL_OK : DebugHud::COL_BAD);
            hud.header({"ID", "Vision", "Battery", "Latency", "Mot-L", "Mot-R", "Status"});
            for (int id = 0; id < 4; ++id) {
                bool visible = id != 3;
                uint8_t battery = (uint8_t)std::max(0.0, 255 - 20.0 * id - 30 * sin(t + id));
                uint16_t latencyUs = (uint16_t)(2000 + 800 * id + 500 * sin(t * 2 + id));
                cv::Scalar col = visible ? DebugHud::COL_OK : DebugHud::COL_WARN;
                hud.row({
                    DebugHud::fmt("%d", id),
                    visible ? "YES" : "NO",
                    DebugHud::formatBattery(battery),
                    visible ? DebugHud::formatLatency(latencyUs) : "--",
                    visible ? DebugHud::fmt("%+d", (id % 2 ? 40 : -40)) : "--",
                    visible ? DebugHud::fmt("%+d", (id % 2 ? -40 : 40)) : "--",
                    visible ? "ACTIVE" : "UNSEEN",
                }, col);
            }
            hud.draw(img, {10, 260});
        }

        cv::imshow(WIN, img);
        int key = cv::waitKey(16) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 'h') hubOk = !hubOk;
    }
    return 0;
}
