// vision_hub.cpp — opt-in camera daemon: owns the Basler, publishes poses, and
// streams the tracked picture to a browser.
//
//   vision_hub [--stream-port N] [--stream-fps F] [--stream-width W]
//              [--stream-quality Q] [--no-stream] [--shm-fps F] [--no-shm]
//              [--robots N] [--serial SN] [--ip IP] [--homography FILE] [--daemon]
//   vision_hub --stop
//
// Start it by hand, and only when you want it. Nothing launches it for you, so
// by default every tool keeps opening the camera itself, with no extra hop.
//
// What it is: the same process any vision demo already is when it owns the
// camera — ArucoTracker::open() publishes poses on /tmp/vision_hub.sock for
// free — minus the demo, plus an MJPEG server for the picture. Pose-only tools
// (swarm_telemetry_json) subscribe exactly as they do to a demo, unchanged.
//
// Watching from another machine:
//
//   ssh -L 8081:localhost:8081 user@robot-pc
//   http://localhost:8081/          (or /stream, or /snapshot.jpg)
//
// The stream is the tracker's own frame (marker outlines, ids, headings) —
// raw sensor pixels, so the same image `RobotPose::px/py` refer to. Overlays a
// demo draws for itself are not in it: the demo has its own copy of the frame.
//
// Tools that draw on the frame (circle_demo, wingman, …) get it through shared
// memory (frame_shm.h). They own the camera when it is free and fall back to
// ArucoTracker::openOrAttach(true) only when it is held, so with the hub running
// they run beside it instead of failing, and with no hub nothing changes for them. The copy is only
// made while a reader is asking.
//
// Neither the stream nor the shared frames may cost detection or control
// anything, which is why the work is split the way it is:
//   • this thread only calls tracker.update() (which publishes poses) and, when
//     someone wants a frame, hands over a reference to the newest one;
//   • one worker thread does the resize, JPEG encode and all socket I/O;
//   • another does the copy into shared memory.
// Nobody watching and nobody reading means no handoff and no work at all.
//
// --daemon detaches it like `swarm_hub --daemon`: PID in /tmp/vision_hub.pid,
// output in /tmp/vision_hub.log, and `vision_hub --stop` ends it. Still opt-in.
//
// The camera admits one application. If a demo already holds it, open() fails
// and the hub says so; if a hub is already running, a second one refuses to
// start rather than fighting over the pose socket.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "aruco_tracker.h"
#include "frame_shm.h"
#include "mjpeg_server.h"
#include "pose_hub.h"

using Clock = std::chrono::steady_clock;

static std::atomic<bool> g_running{true};
static void onSignal(int) { g_running = false; }

static const char* HUB_PID_PATH = "/tmp/vision_hub.pid";
static const char* HUB_LOG_PATH = "/tmp/vision_hub.log";

static const std::string HOMOGRAPHY_FILE_S = arucoVisionDataPath("aruco_homography.yml");

// The newest frame, passed from the main thread to a worker. Holds a Mat
// header only — the tracker never writes into a frame it has handed out, it
// moves a new buffer in on the next update() — so no pixel is copied here.
class FrameMailbox {
public:
    void put(const cv::Mat& frame) {
        { std::lock_guard<std::mutex> lk(m_); slot_ = frame; fresh_ = true; }
        cv_.notify_one();
    }
    // Waits up to `ms` for a frame; empty if none arrived (or wake() was called).
    cv::Mat take(int ms) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, std::chrono::milliseconds(ms), [&] { return fresh_ || woken_; });
        woken_ = false;
        cv::Mat f;
        if (fresh_) { f = slot_; slot_.release(); fresh_ = false; }
        return f;
    }
    void wake() { { std::lock_guard<std::mutex> lk(m_); woken_ = true; } cv_.notify_all(); }

private:
    std::mutex              m_;
    std::condition_variable cv_;
    cv::Mat                 slot_;
    bool                    fresh_ = false, woken_ = false;
};

// Encoder-thread statistics, read once a second by the main thread's status line.
struct StreamStats {
    std::atomic<int>  frames{0};
    std::atomic<long> bytes{0};
    std::atomic<int>  encodeUs{0};   // last frame
};

// Everything past "a frame exists": downscale, encode, serve.
class StreamThread {
public:
    StreamThread(MjpegServer& server, int width, int quality)
        : server_(server), width_(width), quality_(quality) {}

    ~StreamThread() { stop(); }

    void start() { thread_ = std::thread(&StreamThread::run, this); }

    void stop() {
        running_ = false;
        box_.wake();
        if (thread_.joinable()) thread_.join();
    }

    void submit(const cv::Mat& frame) { box_.put(frame); }

    StreamStats stats;

private:
    void run() {
        std::vector<uchar> jpg;
        const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality_};
        cv::Mat small;

        while (running_) {
            server_.poll();

            cv::Mat frame = box_.take(5);
            if (frame.empty()) continue;

            auto t0 = Clock::now();
            const cv::Mat* src = &frame;
            if (width_ > 0 && frame.cols > width_) {
                int h = (int)((long)frame.rows * width_ / frame.cols);
                cv::resize(frame, small, {width_, h}, 0, 0, cv::INTER_AREA);
                src = &small;
            }
            if (!cv::imencode(".jpg", *src, jpg, params)) continue;
            server_.publish(jpg.data(), jpg.size());
            server_.poll();   // start the write now rather than on the next wake-up

            stats.frames++;
            stats.bytes += (long)jpg.size();
            stats.encodeUs = (int)std::chrono::duration_cast<std::chrono::microseconds>(
                                 Clock::now() - t0).count();
        }
    }

    MjpegServer&            server_;
    int                     width_, quality_;
    std::thread             thread_;
    std::atomic<bool>       running_{true};
    FrameMailbox            box_;
};

// Copies frames into shared memory for attached tools (frame_shm.h). The segment
// is created on the first frame, when the geometry is known; it is replaced, not
// resized, if that ever changes.
class ShmThread {
public:
    ~ShmThread() { stop(); }

    void start() { thread_ = std::thread(&ShmThread::run, this); }

    void stop() {
        running_ = false;
        box_.wake();
        if (thread_.joinable()) thread_.join();
    }

    void submit(const cv::Mat& frame) { box_.put(frame); }

    // Is a reader asking? Safe from the main thread: one atomic load. Before the
    // first frame there is no segment and so no reader — the main thread hands
    // one over unconditionally until the segment exists.
    bool wanted() const { return !created_ || writer_.wanted(); }

    std::atomic<int>  frames{0};
    std::atomic<int>  copyUs{0};

private:
    void run() {
        while (running_) {
            cv::Mat f = box_.take(50);
            if (f.empty()) continue;
            if ((f.depth() != CV_8U) || f.channels() < 1 || f.channels() > 4) continue;

            if (!created_) {
                if (!writer_.create((uint32_t)f.cols, (uint32_t)f.rows, (uint32_t)f.channels())) {
                    fprintf(stderr, "[hub] could not create shared-memory frames (" SWARM_FRAMES_SHM_NAME
                                    ") — attached tools that need frames will not work\n");
                    running_ = false;
                    break;
                }
                created_ = true;
                printf("[hub] sharing %dx%dx%d frames as " SWARM_FRAMES_SHM_NAME " (copied only while a tool asks)\n",
                       f.cols, f.rows, f.channels());
                continue;   // that frame was only for the geometry; wait for demand
            }
            auto t0 = Clock::now();
            if (writer_.write(f.data, (uint32_t)f.cols, (uint32_t)f.rows, (uint32_t)f.channels(), f.step[0])) {
                frames++;
                copyUs = (int)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count();
            }
        }
    }

    ShmFrameWriter    writer_;
    std::atomic<bool> created_{false};
    std::thread       thread_;
    std::atomic<bool> running_{true};
    FrameMailbox      box_;
};

// Detach before anything threaded exists (camera, stream) — fork() only keeps
// the calling thread. The parent waits briefly so a start-up failure (camera
// busy, port taken) is reported on the terminal instead of only in the log.
static void daemonize() {
    fflush(stdout);
    fflush(stderr);
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); exit(1); }
    if (pid > 0) {
        for (int i = 0; i < 40; i++) {   // up to ~4 s
            int status = 0;
            if (waitpid(pid, &status, WNOHANG) == pid) {
                fprintf(stderr, "[hub] daemon exited during start-up — see %s\n", HUB_LOG_PATH);
                exit(WIFEXITED(status) && WEXITSTATUS(status) ? WEXITSTATUS(status) : 1);
            }
            usleep(100000);
        }
        {   // closed before exit(): the stream is flushed by its destructor
            std::ofstream pidFile(HUB_PID_PATH);
            if (pidFile) pidFile << pid << "\n";
        }
        printf("[hub] Daemon started (PID %d)\n[hub] Log: %s\n", (int)pid, HUB_LOG_PATH);
        exit(0);
    }
    setsid();
    int logFd = open(HUB_LOG_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (logFd >= 0) { dup2(logFd, STDOUT_FILENO); dup2(logFd, STDERR_FILENO); close(logFd); }
    int nullFd = open("/dev/null", O_RDONLY);
    if (nullFd >= 0) { dup2(nullFd, STDIN_FILENO); close(nullFd); }
    setvbuf(stdout, nullptr, _IOLBF, 0);   // line-buffered: stdout is a file now
}

static int hubStop() {
    std::ifstream pidFile(HUB_PID_PATH);
    pid_t pid = 0;
    if (!(pidFile >> pid) || pid <= 0) {
        fprintf(stderr, "no usable %s — is a daemon running? (a foreground hub stops with Ctrl-C)\n",
                HUB_PID_PATH);
        return 1;
    }
    if (kill(pid, SIGTERM) < 0) {
        if (errno == ESRCH) {
            fprintf(stderr, "vision_hub (PID %d) not running — stale PID file removed.\n", (int)pid);
            unlink(HUB_PID_PATH);
            return 0;
        }
        perror("kill");
        return 1;
    }
    printf("Sent SIGTERM to vision_hub (PID %d), waiting...", (int)pid);
    fflush(stdout);
    for (int i = 0; i < 50 && kill(pid, 0) == 0; i++) usleep(100000);
    if (kill(pid, 0) == 0) {
        printf(" escalating to SIGKILL...");
        kill(pid, SIGKILL);
        usleep(300000);
    }
    printf(" done.\n");
    unlink(HUB_PID_PATH);
    return 0;
}

static void usage(const char* argv0) {
    fprintf(stderr,
        "usage: %s [--stream-port N] [--stream-fps F] [--stream-width W]\n"
        "          [--stream-quality Q] [--no-stream] [--shm-fps F] [--no-shm]\n"
        "          [--robots N]\n"
        "          [--serial SN] [--ip IP] [--homography FILE] [--daemon]\n"
        "       %s --stop\n"
        "\n"
        "  --stream-port N     MJPEG port on 127.0.0.1 (default 8081)\n"
        "  --stream-fps F      stream frame-rate cap (default 30)\n"
        "  --stream-width W    downscale to W px wide before encoding, 0 = full (default 960)\n"
        "  --stream-quality Q  JPEG quality 1-100 (default 70)\n"
        "  --no-stream         no MJPEG stream\n"
        "  --shm-fps F         cap on frames copied to shared memory for attached tools (default 30)\n"
        "  --no-shm            share no frames (attached tools that draw on the frame cannot attach)\n"
        "  --robots N          pin the tracker's robot count (default: from config)\n"
        "  --homography FILE   world-coordinate calibration (default: the shared one)\n"
        "  --daemon, -d        detach; PID in %s, log in %s\n"
        "  --stop              stop the daemon\n",
        argv0, argv0, HUB_PID_PATH, HUB_LOG_PATH);
}

int main(int argc, char** argv) {
    int         streamPort = 8081, streamWidth = 960, streamQuality = 70, robots = 0;
    float       streamFps  = 30.f, shmFps = 30.f;
    bool        stream     = true, shm = true, daemonMode = false;
    std::string serial, ip, homographyFile = HOMOGRAPHY_FILE_S;

    for (int i = 1; i < argc; i++) {
        auto arg  = [&](const char* name) { return strcmp(argv[i], name) == 0 && i + 1 < argc; };
        if      (arg("--stream-port"))    streamPort    = atoi(argv[++i]);
        else if (arg("--stream-fps"))     streamFps     = (float)atof(argv[++i]);
        else if (arg("--stream-width"))   streamWidth   = atoi(argv[++i]);
        else if (arg("--stream-quality")) streamQuality = atoi(argv[++i]);
        else if (arg("--shm-fps"))        shmFps        = (float)atof(argv[++i]);
        else if (arg("--robots"))         robots        = atoi(argv[++i]);
        else if (arg("--serial"))         serial        = argv[++i];
        else if (arg("--ip"))             ip            = argv[++i];
        else if (arg("--homography"))     homographyFile = argv[++i];
        else if (strcmp(argv[i], "--no-stream") == 0) stream = false;
        else if (strcmp(argv[i], "--no-shm") == 0)    shm    = false;
        else if (strcmp(argv[i], "--daemon") == 0 || strcmp(argv[i], "-d") == 0) daemonMode = true;
        else if (strcmp(argv[i], "--stop") == 0) return hubStop();
        else { usage(argv[0]); return strcmp(argv[i], "--help") == 0 ? 0 : 2; }
    }
    if (streamPort < 1 || streamPort > 65535 || streamFps <= 0.f || shmFps <= 0.f ||
        streamQuality < 1 || streamQuality > 100 || streamWidth < 0) {
        usage(argv[0]);
        return 2;
    }

    // A hub that is already up owns the pose socket. open() would ignore the
    // refusal and carry on, leaving two publishers and one of them silent.
    {
        PoseHubSubscriber probe;
        if (probe.connect()) {
            fprintf(stderr, "[hub] a publisher already holds " POSE_HUB_SOCK_PATH
                            " — a vision_hub (or a demo) is running. Not starting.\n");
            return 1;
        }
    }

    if (daemonMode) daemonize();

    ArucoConfig cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    if (robots > 0)      cfg.robotCount   = robots;

    ArucoTracker tracker(cfg);
    if (!tracker.open()) {
        fprintf(stderr, "[hub] could not open the Basler camera. If it reports "
                        "0xE1018006 another application holds it — close that "
                        "demo first, or use the hub instead of it.\n");
        return 1;
    }
    printf("[hub] camera open at %dx%d\n", tracker.frameSize().width, tracker.frameSize().height);

    // Subscribers get world coordinates only if the publisher has a homography.
    if (tracker.loadHomography(homographyFile))
        printf("[hub] homography loaded from %s — poses are in mm\n", homographyFile.c_str());
    else
        printf("[hub] no usable homography at %s — poses are in pixels\n", homographyFile.c_str());
    printf("[hub] publishing poses on " POSE_HUB_SOCK_PATH "\n");

    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);
    signal(SIGPIPE, SIG_IGN);

    MjpegServer                   server;
    std::unique_ptr<StreamThread> streamer;
    if (stream) {
        if (server.start(streamPort)) {
            streamer = std::make_unique<StreamThread>(server, streamWidth, streamQuality);
            streamer->start();
            printf("[hub] stream on http://127.0.0.1:%d/  "
                   "(ssh -L %d:localhost:%d <host>)  %.0f fps max, %s px wide, q%d\n",
                   streamPort, streamPort, streamPort, streamFps,
                   streamWidth ? std::to_string(streamWidth).c_str() : "full", streamQuality);
        } else {
            fprintf(stderr, "[hub] port %d is busy — continuing without a stream "
                            "(poses are still published)\n", streamPort);
        }
    }

    std::unique_ptr<ShmThread> sharer;
    if (shm) {
        sharer = std::make_unique<ShmThread>();
        sharer->start();
        printf("[hub] shared-memory frames for attached tools: %.0f fps max, only while one is reading\n", shmFps);
    }

    auto every = [](float fps) {
        return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>(1.f / fps));
    };
    const auto handoffEvery = every(streamFps), shmEvery = every(shmFps);
    auto lastHandoff = Clock::now() - handoffEvery;
    auto lastShm     = Clock::now() - shmEvery;
    auto lastStatus  = Clock::now();
    int  lastFrames  = 0, lastShmFrames = 0;
    long lastBytes   = 0;

    while (g_running) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        } else {
            // One fresh result: hand its frame to whichever workers want one.
            // debugFrame() is a header copy, so asking twice costs nothing.
            auto now = Clock::now();
            if (streamer && server.wantsFrames() && now - lastHandoff >= handoffEvery) {
                lastHandoff = now;
                streamer->submit(tracker.debugFrame());
            }
            if (sharer && sharer->wanted() && now - lastShm >= shmEvery) {
                lastShm = now;
                sharer->submit(tracker.debugFrame());
            }
        }

        auto now = Clock::now();
        if (now - lastStatus >= std::chrono::seconds(1)) {
            float dt = std::chrono::duration<float>(now - lastStatus).count();
            lastStatus = now;
            std::string line = "[hub] det " + std::to_string((int)tracker.detectionFps()) + " fps  tags " +
                               std::to_string(tracker.robots().size());
            char buf[160];
            if (streamer) {
                int  f = streamer->stats.frames;
                long b = streamer->stats.bytes;
                snprintf(buf, sizeof(buf), "  viewers %d  stream %.0f fps  %.0f kB/s  encode %.1f ms",
                         server.viewers(), (f - lastFrames) / dt, (b - lastBytes) / dt / 1024.f,
                         streamer->stats.encodeUs / 1000.f);
                line += buf;
                lastFrames = f;
                lastBytes  = b;
            }
            if (sharer) {
                int f = sharer->frames;
                snprintf(buf, sizeof(buf), "  shm %.0f fps  copy %.1f ms", (f - lastShmFrames) / dt,
                         sharer->copyUs / 1000.f);
                line += buf;
                lastShmFrames = f;
            }
            fprintf(stderr, "%s\n", line.c_str());
        }
    }

    printf("[hub] shutting down\n");
    if (streamer) streamer->stop();
    if (sharer)   sharer->stop();
    if (daemonMode) unlink(HUB_PID_PATH);
    return 0;
}
