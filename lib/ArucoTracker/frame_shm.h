// frame_shm.h — share camera frames between processes through POSIX shared memory.
//
// pose_hub.h shares *where the robots are*; this shares the picture they were
// found in, for tools that draw on it or click on it (circle_demo, wingman,
// shape_demo, …). A 2048x2048 BGR frame is 12.6 MB and the tracker makes one
// per detection, so ~116 fps is ~1.4 GB/s — far past what a stream socket
// should carry, and exactly what a shared mapping is for: the writer copies a
// frame in once, and any number of readers copy it out when they want it.
//
//   vision_hub  ──write()──►  [slot 0][slot 1][slot 2]  ◄──copyLatest()──  tools
//
// Deliberately free of OpenCV and pylon, like pose_hub.h: it moves bytes with a
// width, a height and a channel count, and the tracker turns those into a Mat.
//
// ── Demand-driven ───────────────────────────────────────────────────────────
// Copying 12 MB costs a millisecond or two, and nobody should pay that for a
// frame no one will look at. Readers therefore *ask*: touch() stamps a
// heartbeat in the header, and the writer's wanted() is true only while that
// stamp is recent. No reader, no copy — the hub's cost with nothing attached is
// a single atomic load. A crashed reader leaves a stamp that simply expires, so
// there is no count to leak.
//
// ── Why readers never see half a frame ──────────────────────────────────────
// A seqlock per slot. Each slot carries the id of the frame it holds; the writer
// sets it to 0, copies, then publishes the new id. A reader notes the id, copies,
// and checks the id again: if it moved, the writer got there first and the
// reader retries. With three slots the writer is writing into the slot *after*
// the newest, so a reader normally never even collides — the check is for the
// reader that stalls for two whole frames mid-copy.
//
// The writer never waits for a reader and a reader never blocks the writer.
//
// The segment is created by the writer and unlinked when it exits. A reader
// that outlives its writer keeps a mapping of a dead segment; ids stop moving,
// which is how the tracker notices and re-opens (see ArucoTracker).

#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <new>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#define SWARM_FRAMES_SHM_NAME "/swarm_vision_frames"

static constexpr uint32_t FRAME_SHM_MAGIC   = 0x4D524653;   // 'SFRM' little-endian
static constexpr uint32_t FRAME_SHM_VERSION = 1;
static constexpr uint32_t FRAME_SHM_SLOTS   = 3;

// Lives at offset 0. Everything but the two atomics is written once, before the
// segment is announced, and never changes — a frame geometry change means a new
// segment.
struct ShmFrameHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t slots;
    uint32_t width, height, channels;   // 8-bit samples; frames are tightly packed
    uint32_t writerPid;
    uint64_t slotBytes;                 // width * height * channels
    std::atomic<uint64_t> latestId;     // newest complete frame, 0 = none yet
    std::atomic<uint64_t> wantedNs;     // reader heartbeat, CLOCK_MONOTONIC
};

// Precedes each slot's pixels. `id` is the frame stored there, 0 while the
// writer is inside it.
struct ShmSlotHeader {
    std::atomic<uint64_t> id;
};

static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "the seqlock needs lock-free 64-bit atomics (also required across processes)");

namespace frame_shm {

inline uint64_t nowNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

// Header and slot headers are padded to a cache line so the writer's stores to
// one slot don't share a line with a reader's load of another.
constexpr size_t kAlign = 64;
constexpr size_t alignUp(size_t n) { return (n + kAlign - 1) / kAlign * kAlign; }

inline size_t headerBytes()                    { return alignUp(sizeof(ShmFrameHeader)); }
inline size_t slotStride(uint64_t slotBytes)   { return alignUp(sizeof(ShmSlotHeader)) + alignUp((size_t)slotBytes); }
inline size_t totalBytes(uint32_t slots, uint64_t slotBytes) {
    return headerBytes() + (size_t)slots * slotStride(slotBytes);
}

inline ShmSlotHeader* slotAt(void* base, uint32_t index, uint64_t slotBytes) {
    return reinterpret_cast<ShmSlotHeader*>((char*)base + headerBytes() +
                                            (size_t)index * slotStride(slotBytes));
}
inline uint8_t* slotPixels(ShmSlotHeader* s) {
    return reinterpret_cast<uint8_t*>(s) + alignUp(sizeof(ShmSlotHeader));
}

} // namespace frame_shm

// ── Writer ───────────────────────────────────────────────────────────────────

class ShmFrameWriter {
public:
    ~ShmFrameWriter() { destroy(); }

    // Creates the segment for one fixed frame geometry. A leftover segment from
    // a crashed writer is replaced, never truncated in place: truncating a file
    // some reader still has mapped would SIGBUS that reader, whereas unlinking
    // just leaves it a dead-but-valid mapping it will notice and drop.
    bool create(uint32_t width, uint32_t height, uint32_t channels,
                const char* name = SWARM_FRAMES_SHM_NAME) {
        destroy();
        if (!width || !height || channels < 1 || channels > 4) return false;

        const uint64_t slotBytes = (uint64_t)width * height * channels;
        const size_t   total     = frame_shm::totalBytes(FRAME_SHM_SLOTS, slotBytes);

        ::shm_unlink(name);
        fd_ = ::shm_open(name, O_CREAT | O_EXCL | O_RDWR, 0600);
        if (fd_ < 0) return false;
        if (::ftruncate(fd_, (off_t)total) < 0) { destroy(); return false; }
        struct stat st;
        if (::fstat(fd_, &st) < 0) { destroy(); return false; }
        ino_ = (uint64_t)st.st_ino;

        base_ = ::mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (base_ == MAP_FAILED) { base_ = nullptr; destroy(); return false; }

        name_  = name;
        total_ = total;
        hdr_   = new (base_) ShmFrameHeader;
        hdr_->magic     = FRAME_SHM_MAGIC;
        hdr_->version   = FRAME_SHM_VERSION;
        hdr_->slots     = FRAME_SHM_SLOTS;
        hdr_->width     = width;
        hdr_->height    = height;
        hdr_->channels  = channels;
        hdr_->writerPid = (uint32_t)::getpid();
        hdr_->slotBytes = slotBytes;
        hdr_->latestId.store(0);
        hdr_->wantedNs.store(0);
        for (uint32_t i = 0; i < FRAME_SHM_SLOTS; i++)
            new (frame_shm::slotAt(base_, i, slotBytes)) ShmSlotHeader{{0}};
        return true;
    }

    bool isOpen() const { return hdr_ != nullptr; }

    // Is a reader asking for frames? `windowMs` is how stale a heartbeat may be.
    bool wanted(int windowMs = 1000) const {
        if (!hdr_) return false;
        uint64_t t = hdr_->wantedNs.load(std::memory_order_relaxed);
        return t != 0 && frame_shm::nowNs() - t < (uint64_t)windowMs * 1000000ull;
    }

    // Copies one frame in. `stride` is the source's bytes per row, so a
    // non-contiguous cv::Mat works without a prior clone. Refuses a frame whose
    // geometry is not the one the segment was created for. Returns the new id.
    uint64_t write(const void* data, uint32_t width, uint32_t height,
                   uint32_t channels, size_t stride) {
        if (!hdr_ || !data || width != hdr_->width || height != hdr_->height ||
            channels != hdr_->channels)
            return 0;

        const uint64_t id   = ++lastId_;
        ShmSlotHeader* slot = frame_shm::slotAt(base_, (uint32_t)(id % hdr_->slots), hdr_->slotBytes);

        slot->id.store(0, std::memory_order_seq_cst);          // readers: this slot is no longer valid
        std::atomic_thread_fence(std::memory_order_seq_cst);

        const size_t rowBytes = (size_t)width * channels;
        uint8_t*     dst      = frame_shm::slotPixels(slot);
        const uint8_t* src    = static_cast<const uint8_t*>(data);
        if (stride == rowBytes) {
            std::memcpy(dst, src, rowBytes * height);
        } else {
            for (uint32_t y = 0; y < height; y++)
                std::memcpy(dst + (size_t)y * rowBytes, src + (size_t)y * stride, rowBytes);
        }

        slot->id.store(id, std::memory_order_release);
        hdr_->latestId.store(id, std::memory_order_release);
        return id;
    }

    void destroy() {
        if (base_) { ::munmap(base_, total_); base_ = nullptr; }
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        if (hdr_) { unlinkIfMine(); hdr_ = nullptr; }
    }

private:
    // The name may by now belong to a successor that replaced this segment
    // (a restarted hub): unlinking by name alone would delete *its* segment.
    void unlinkIfMine() {
        int fd = ::shm_open(name_, O_RDONLY, 0);
        if (fd < 0) return;
        struct stat st;
        bool mine = ::fstat(fd, &st) == 0 && (uint64_t)st.st_ino == ino_;
        ::close(fd);
        if (mine) ::shm_unlink(name_);
    }

    int              fd_    = -1;
    void*            base_  = nullptr;
    size_t           total_ = 0;
    const char*      name_  = SWARM_FRAMES_SHM_NAME;
    uint64_t         ino_   = 0;
    ShmFrameHeader*  hdr_   = nullptr;
    uint64_t         lastId_ = 0;
};

// ── Reader ───────────────────────────────────────────────────────────────────

class ShmFrameReader {
public:
    ~ShmFrameReader() { close(); }

    // Maps an existing segment. Fails if there is none, or if what is there does
    // not describe itself consistently (wrong version, or a size that does not
    // match its own geometry — a half-created or foreign file).
    bool open(const char* name = SWARM_FRAMES_SHM_NAME) {
        close();
        fd_   = ::shm_open(name, O_RDWR, 0);   // RDWR: the heartbeat is a write
        if (fd_ < 0) return false;

        struct stat st;
        if (::fstat(fd_, &st) < 0 || (size_t)st.st_size < frame_shm::headerBytes()) { close(); return false; }

        size_t total = (size_t)st.st_size;
        base_ = ::mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (base_ == MAP_FAILED) { base_ = nullptr; close(); return false; }
        total_ = total;
        hdr_   = static_cast<ShmFrameHeader*>(base_);

        if (hdr_->magic != FRAME_SHM_MAGIC || hdr_->version != FRAME_SHM_VERSION ||
            hdr_->slots != FRAME_SHM_SLOTS || hdr_->channels < 1 || hdr_->channels > 4 ||
            hdr_->slotBytes != (uint64_t)hdr_->width * hdr_->height * hdr_->channels ||
            total != frame_shm::totalBytes(hdr_->slots, hdr_->slotBytes)) {
            close();
            return false;
        }
        return true;
    }

    void close() {
        hdr_ = nullptr;
        if (base_) { ::munmap(base_, total_); base_ = nullptr; }
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool isOpen() const { return hdr_ != nullptr; }

    uint32_t width()    const { return hdr_ ? hdr_->width : 0; }
    uint32_t height()   const { return hdr_ ? hdr_->height : 0; }
    uint32_t channels() const { return hdr_ ? hdr_->channels : 0; }
    size_t   frameBytes() const { return hdr_ ? (size_t)hdr_->slotBytes : 0; }

    // Newest complete frame's id, 0 if none has been written yet.
    uint64_t latestId() const { return hdr_ ? hdr_->latestId.load(std::memory_order_acquire) : 0; }

    // "I am looking at frames" — keeps the writer copying for the next second.
    void touch() { if (hdr_) hdr_->wantedNs.store(frame_shm::nowNs(), std::memory_order_relaxed); }

    // Copies the newest frame into `dst` (frameBytes() long) and returns its id,
    // or 0 if there is none or the writer kept overtaking the copy.
    uint64_t copyLatest(void* dst) {
        if (!hdr_ || !dst) return 0;
        for (int attempt = 0; attempt < 8; attempt++) {
            const uint64_t id = hdr_->latestId.load(std::memory_order_acquire);
            if (id == 0) return 0;
            ShmSlotHeader* slot = frame_shm::slotAt(base_, (uint32_t)(id % hdr_->slots), hdr_->slotBytes);
            if (slot->id.load(std::memory_order_acquire) != id) continue;

            std::memcpy(dst, frame_shm::slotPixels(slot), (size_t)hdr_->slotBytes);

            std::atomic_thread_fence(std::memory_order_seq_cst);
            if (slot->id.load(std::memory_order_acquire) == id) return id;   // nobody wrote under us
        }
        return 0;
    }

private:
    int              fd_    = -1;
    void*            base_  = nullptr;
    size_t           total_ = 0;
    ShmFrameHeader*  hdr_   = nullptr;
};
