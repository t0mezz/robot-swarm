// demo_hud_preview.cpp — standalone visual preview/test for lib/DemoHud/DemoHud.h.
//
// Renders the shared HUD panel against synthetic data only: no camera, no
// Basler/pylon dependency, no swarm_hub/SwarmClient connection. Useful for
// iterating on DemoHud layout/sizing/colors without hardware attached.
//
// Controls: q/Esc quit, +/- scale font, h toggle simulated hub state.

#include "DemoHud.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

int main() {
    const char* WIN = "DemoHud Demo";
    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);

    DemoHud::LoopFps loopFps;
    bool hubOk = true;
    auto start = std::chrono::steady_clock::now();

    printf("DemoHud demo: q/Esc quit, h toggle simulated hub state.\n");

    while (true) {
        cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(40, 40, 40));
        loopFps.tick();

        double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();

        // ── The uniform demo HUD: summary title line + per-robot table, docked
        //    top-right. This mirrors exactly what every demo tool now renders
        //    (vision_controller, circle_demo, wingman, shape_demo, drag_drop). ──
        DemoHud hud;
        hud.title(DemoHud::fmt("loop_fps:%.0f  Robots:%d/4  HUB:%s  t:%.1fs",
                  loopFps.fps(), 3, hubOk ? "OK" : "OFFLINE", t),
                  hubOk ? DemoHud::COL_OK : DemoHud::COL_BAD);
        hud.header({"ID", "Vision", "Battery", "Latency", "Mot-L", "Mot-R", "Status"});
        for (int id = 0; id < 4; ++id) {
            bool visible = id != 3;
            uint8_t battery = (uint8_t)std::max(0.0, 255 - 20.0 * id - 30 * sin(t + id));
            uint16_t latencyUs = (uint16_t)(2000 + 800 * id + 500 * sin(t * 2 + id));
            cv::Scalar col = visible ? DemoHud::COL_OK : DemoHud::COL_WARN;
            hud.row({
                DemoHud::fmt("%d", id),
                visible ? "YES" : "NO",
                DemoHud::formatBattery(battery),
                visible ? DemoHud::formatLatency(latencyUs) : "--",
                visible ? DemoHud::fmt("%+d", (id % 2 ? 40 : -40)) : "--",
                visible ? DemoHud::fmt("%+d", (id % 2 ? -40 : 40)) : "--",
                visible ? "ACTIVE" : "UNSEEN",
            }, col);
        }
        hud.drawTopRight(img);

        cv::imshow(WIN, img);
        int key = cv::waitKey(16) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 'h') hubOk = !hubOk;
    }
    return 0;
}
