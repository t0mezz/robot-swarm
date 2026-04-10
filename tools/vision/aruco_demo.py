import cv2
import numpy as np
import os
import re
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

# Thread pool for parallel ROI detection — created once, reused every frame.
# OpenCV's detectMarkers releases the GIL, so threads run in true parallel.
_roi_executor = ThreadPoolExecutor(max_workers=min(8, os.cpu_count() or 4))

# Physical size of one marker side in metres — adjust to match your printed tags.
MARKER_SIZE_M = 0.06  # 6 cm — match your printed tag size

# GoPro hardware model prefix used by system_profiler to identify the device
# regardless of what display name macOS has assigned it.
GOPRO_MODEL_PREFIX = "HERO"

# ROI cache: expand each cached bounding box by this fraction on each side.
ROI_PADDING = 0.6

# Run a full-frame detection sweep every N frames to pick up new / moved markers.
# Keep this low when markers are first being discovered; raise it once the scene
# is stable to trade off discovery latency for CPU headroom.
FULL_FRAME_INTERVAL = 5


# ---------------------------------------------------------------------------
# Capture thread — decouples frame grabbing from detection / display so that
# cap.read() blocking never stalls the processing loop.
# ---------------------------------------------------------------------------

class CaptureThread(threading.Thread):
    """Continuously grabs frames via AVFoundation; always exposes the latest one."""

    def __init__(self, cap: cv2.VideoCapture) -> None:
        super().__init__(daemon=True)
        self._cap = cap
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._running = True

    def run(self) -> None:
        while self._running:
            ret, frame = self._cap.read()
            if ret and frame is not None and frame.size > 0:
                with self._lock:
                    self._frame = frame

    def read(self) -> np.ndarray | None:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def stop(self) -> None:
        self._running = False


# ---------------------------------------------------------------------------
# ROI-first detection
# ---------------------------------------------------------------------------

def _detect_with_roi(
    detector_full: cv2.aruco.ArucoDetector,
    detector_roi: cv2.aruco.ArucoDetector,
    gray: np.ndarray,
    roi_cache: dict,
    frame_count: int,
    prev_centers: dict,
) -> tuple[list, np.ndarray | None]:
    """Detect markers using a full-frame sweep as the authoritative source,
    supplemented by ROI crops for any cached marker the full sweep missed.

    The previous approach filled seen_ids from ROI pass 1, then filtered pass 2
    against it — silently dropping markers whose ROI overlapped a neighbour's.
    Now the full-frame result is always primary; ROI is only used to recover
    markers the full sweep missed (e.g. motion blur on one threshold level).
    """
    h, w = gray.shape

    # Half-resolution image for full-frame sweep: ~4x fewer pixels, and
    # reflection hot spots are averaged across adaptive threshold windows.
    gray_half = cv2.resize(gray, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    # Pass 1: full-frame sweep on half-res (always run — this is the authoritative result)
    corners_full, ids_full, _ = detector_full.detectMarkers(gray_half)

    # Deduplicate by ID: keep the detection with the largest perimeter per ID.
    # minMarkerDistanceRate=0.0 disables OpenCV's NMS entirely, so the same
    # physical marker appears once per threshold level. Picking the largest
    # perimeter selects the sharpest detection without suppressing neighbours.
    # Scale corners back to full-res coords (×2) after deduplication.
    best: dict[int, tuple[float, np.ndarray]] = {}
    if ids_full is not None:
        for j, mid in enumerate(ids_full.flatten()):
            c = corners_full[j][0] * 2.0  # scale to full-res
            perimeter = float(np.sum(np.linalg.norm(np.diff(c, axis=0, append=c[:1]), axis=1)))
            if mid not in best or perimeter > best[mid][0]:
                # wrap back into shape (1,4,2) to match the expected corners format
                best[mid] = (perimeter, c[np.newaxis])

    found_corners: list = [v[1] for v in best.values()]
    found_ids: list[int] = list(best.keys())
    seen_ids: set[int] = set(found_ids)

    # Pass 2: ROI crops in parallel for cached markers the full sweep missed.
    # detectMarkers releases the GIL, so ThreadPoolExecutor achieves true parallelism.
    missed = {mid: bbox for mid, bbox in roi_cache.items() if mid not in seen_ids}
    if missed and (frame_count % FULL_FRAME_INTERVAL == 0 or len(seen_ids) < len(roi_cache)):
        def _roi_task(marker_id: int, bbox: tuple) -> list:
            bx, by, bw, bh = bbox
            x1, y1 = max(0, bx), max(0, by)
            x2, y2 = min(w, bx + bw), min(h, by + bh)
            if x2 <= x1 or y2 <= y1:
                return []
            try:
                c_roi, ids_roi, _ = detector_roi.detectMarkers(gray[y1:y2, x1:x2])
            except Exception:
                return []
            results = []
            if ids_roi is not None:
                for j, mid in enumerate(ids_roi.flatten()):
                    if mid == marker_id:
                        c = c_roi[j].copy()
                        c[0][:, 0] += x1
                        c[0][:, 1] += y1
                        results.append((mid, c))
            return results

        futures = {_roi_executor.submit(_roi_task, mid, bbox): mid
                   for mid, bbox in missed.items()}
        for fut in as_completed(futures):
            try:
                for mid, c in fut.result():
                    if mid not in seen_ids:
                        found_corners.append(c)
                        found_ids.append(mid)
                        seen_ids.add(mid)
            except Exception:
                pass

    # Update ROI cache
    new_cache: dict = {}
    for k, mid in enumerate(found_ids):
        c = found_corners[k][0]
        xs, ys = c[:, 0], c[:, 1]
        bx = int(xs.min())
        by = int(ys.min())
        bw = max(1, int(xs.max() - xs.min()))
        bh = max(1, int(ys.max() - ys.min()))
        cx = bx + bw / 2.0
        cy = by + bh / 2.0
        if mid in prev_centers:
            px, py = prev_centers[mid]
            vel = float(np.hypot(cx - px, cy - py))
        else:
            vel = 0.0
        prev_centers[mid] = (cx, cy)
        vel_pad = vel * 1.5
        pad_x = max(24, int(bw * ROI_PADDING + vel_pad))
        pad_y = max(24, int(bh * ROI_PADDING + vel_pad))
        new_cache[mid] = (bx - pad_x, by - pad_y, bw + 2 * pad_x, bh + 2 * pad_y)
    roi_cache.clear()
    roi_cache.update(new_cache)

    if not found_ids:
        return [], None
    return found_corners, np.array(found_ids, dtype=np.int32).reshape(-1, 1)


# ---------------------------------------------------------------------------
# Camera helpers
# ---------------------------------------------------------------------------

def _find_gopro_index() -> int:
    """Find the AVFoundation index for the connected GoPro camera.

    Two-step lookup:
      1. system_profiler SPCameraDataType → find camera whose hardware model-id
         starts with GOPRO_MODEL_PREFIX (e.g. "HERO13 Black").  This is set by
         GoPro firmware and is immune to whatever display name macOS assigns.
      2. ffmpeg AVFoundation device list → match that verified display name to
         get the numeric index OpenCV needs.

    Raises RuntimeError if no GoPro is found or the index cannot be determined.
    """
    import json

    # Step 1: hardware-verified display name via system_profiler
    gopro_name: str | None = None
    try:
        result = subprocess.run(
            ["system_profiler", "SPCameraDataType", "-json"],
            capture_output=True, text=True, timeout=5,
        )
        data = json.loads(result.stdout)
        for cam in data.get("SPCameraDataType", []):
            model = cam.get("spcamera_model-id", "")
            if GOPRO_MODEL_PREFIX.upper() in model.upper():
                gopro_name = cam["_name"]
                print(f"[camera] Hardware model '{model}' → display name '{gopro_name}'")
                break
    except Exception as e:
        raise RuntimeError(f"system_profiler lookup failed: {e}") from e

    if gopro_name is None:
        raise RuntimeError(
            f"No GoPro camera found (looking for model-id containing '{GOPRO_MODEL_PREFIX}'). "
            "Is the GoPro connected and the Webcam extension active?"
        )

    # Step 2: map that display name to an AVFoundation index via ffmpeg
    try:
        result = subprocess.run(
            ["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
            capture_output=True, text=True, timeout=5,
        )
        in_video = False
        devices: dict[int, str] = {}
        for line in result.stderr.splitlines():
            if "AVFoundation video devices" in line:
                in_video = True
                continue
            if "AVFoundation audio devices" in line:
                in_video = False
                continue
            if in_video:
                m = re.search(r"\[(\d+)\]\s+(.+)$", line)
                if m:
                    devices[int(m.group(1))] = m.group(2).strip()
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        raise RuntimeError(f"ffmpeg device enumeration failed: {e}") from e

    print("Available video devices:")
    for idx, name in sorted(devices.items()):
        marker = " ◄" if name == gopro_name else ""
        print(f"  [{idx}] {name}{marker}")

    for idx, name in devices.items():
        if name == gopro_name:
            print(f"[camera] Selected AVFoundation index {idx} for '{gopro_name}'")
            return idx

    raise RuntimeError(
        f"Camera '{gopro_name}' (from system_profiler) not found in AVFoundation list.\n"
        f"Available: {list(devices.values())}"
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam", type=int, default=None,
                        help="Override camera index (skip auto-detection)")
    args = parser.parse_args()

    # 1. Dictionary + optimised detector parameters
    #
    # DICT_4X4_50 instead of DICT_APRILTAG_36h11:
    # The AprilTag backend inside OpenCV runs its own internal quad-NMS step
    # (aprilTagMaxNmsBboxes, default ~10) that silently caps detections before
    # they reach the ArUco layer — this is the hard limit of ~9 markers per frame.
    # The standard ArUco pipeline has no such hidden cap and handles dense grids
    # without issue.  4x4_50 gives 50 unique IDs and is the smallest/fastest dict.
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()

    # --- Threshold: finer steps + wider range catches markers at more sizes ---
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4

    # --- Candidate geometry: allow smaller and more densely packed markers ---
    params.minMarkerPerimeterRate = 0.01
    params.polygonalApproxAccuracyRate = 0.04
    params.minMarkerDistanceRate = 0.0     # disable inter-marker NMS entirely

    # --- Bit decoding ---
    params.perspectiveRemovePixelPerCell = 10
    params.errorCorrectionRate = 0.6

    # Corner refinement: CONTOUR is fast and improves quad precision
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

    # Shared bit-decode and reflection parameters
    params.perspectiveRemoveIgnoredMarginPerCell = 0.20  # pull sampling from glossy edges
    params.minOtsuStdDev                         = 12.0  # washed-out cells → mean decode

    # Full-frame sweep: finer threshold steps + higher reflection rejection
    params_full = cv2.aruco.DetectorParameters()
    params_full.adaptiveThreshWinSizeMin              = params.adaptiveThreshWinSizeMin
    params_full.adaptiveThreshWinSizeMax              = 35   # 17 levels instead of 26; faster + fewer large-window false-positives
    params_full.adaptiveThreshWinSizeStep             = 2
    params_full.adaptiveThreshConstant                = 20   # high: good lighting, reject specular highlights
    params_full.minMarkerPerimeterRate                = params.minMarkerPerimeterRate
    params_full.polygonalApproxAccuracyRate           = params.polygonalApproxAccuracyRate
    params_full.minMarkerDistanceRate                 = params.minMarkerDistanceRate
    params_full.perspectiveRemovePixelPerCell         = params.perspectiveRemovePixelPerCell
    params_full.perspectiveRemoveIgnoredMarginPerCell = params.perspectiveRemoveIgnoredMarginPerCell
    params_full.errorCorrectionRate                   = params.errorCorrectionRate
    params_full.minOtsuStdDev                         = params.minOtsuStdDev
    params_full.cornerRefinementMethod                = params.cornerRefinementMethod

    # ROI crops: coarser step is fine since crops are small
    params_roi = cv2.aruco.DetectorParameters()
    params_roi.adaptiveThreshWinSizeMin              = params.adaptiveThreshWinSizeMin
    params_roi.adaptiveThreshWinSizeMax              = params.adaptiveThreshWinSizeMax
    params_roi.adaptiveThreshWinSizeStep             = 4
    params_roi.adaptiveThreshConstant                = 17   # slightly lower than full: small crops have noisier local means
    params_roi.minMarkerPerimeterRate                = params.minMarkerPerimeterRate
    params_roi.polygonalApproxAccuracyRate           = params.polygonalApproxAccuracyRate
    params_roi.minMarkerDistanceRate                 = params.minMarkerDistanceRate
    params_roi.perspectiveRemovePixelPerCell         = params.perspectiveRemovePixelPerCell
    params_roi.perspectiveRemoveIgnoredMarginPerCell = params.perspectiveRemoveIgnoredMarginPerCell
    params_roi.errorCorrectionRate                   = params.errorCorrectionRate
    params_roi.minOtsuStdDev                         = params.minOtsuStdDev
    params_roi.cornerRefinementMethod                = params.cornerRefinementMethod

    detector_full = cv2.aruco.ArucoDetector(dictionary, params_full)
    detector_roi  = cv2.aruco.ArucoDetector(dictionary, params_roi)

    # 2. Camera — GoPro via AVFoundation webcam extension
    if args.cam is not None:
        cam_index = args.cam
        print(f"[camera] Manual override: using index {cam_index}")
    else:
        cam_index = _find_gopro_index()
    cap = cv2.VideoCapture(cam_index, cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f"Error: could not open camera at index {cam_index}.")
        return

    fw = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    fh = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Opened camera [{cam_index}] at {int(fw)}x{int(fh)}")

    # 3. Camera matrix
    focal = fw
    camera_matrix = np.array(
        [[focal, 0, fw / 2], [0, focal, fh / 2], [0, 0, 1]], dtype=np.float64
    )
    dist_coeffs = np.zeros(5, dtype=np.float64)

    half = MARKER_SIZE_M / 2
    obj_pts = np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        dtype=np.float32,
    )

    # 4. Start capture thread
    capture = CaptureThread(cap)
    capture.start()
    print("Kamera gestartet. Drücke 'q', um das Fenster zu schließen.")

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    roi_cache: dict = {}
    prev_centers: dict = {}
    frame_count = 0
    prev_time = time.time()
    fps = 0.0

    while True:
        frame = capture.read()
        if frame is None:
            time.sleep(0.001)
            continue

        # FPS
        now = time.time()
        fps = 1.0 / (now - prev_time) if (now - prev_time) > 0 else 0.0
        prev_time = now
        frame_count += 1

        if frame.ndim != 3 or frame.shape[2] != 3:
            print(f"[frame] unexpected shape {frame.shape}, skipping")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = clahe.apply(gray)

        # 5. ROI-first detection
        try:
            corners, ids = _detect_with_roi(detector_full, detector_roi, gray, roi_cache, frame_count, prev_centers)
        except Exception as e:
            print(f"[detect] {type(e).__name__}: {e}")
            ids = None
            corners = []

        # 6. Draw markers + pose
        h_frame, w_frame = frame.shape[:2]
        if ids is not None:
            for i in range(len(ids)):
                try:
                    c = corners[i][0].astype(np.float32)
                    pts = c.reshape((-1, 1, 2)).astype(int)
                    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=4)
                    cx = int((c[0][0] + c[2][0]) / 2)
                    cy = int((c[0][1] + c[2][1]) / 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                    label_x = int(np.clip(pts[0][0][0], 0, w_frame - 1))
                    label_y = int(np.clip(pts[0][0][1] - 10, 14, h_frame - 1))
                    cv2.putText(frame, str(ids[i][0]), (label_x, label_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    ok, rvec, tvec = cv2.solvePnP(
                        obj_pts, c, camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    valid = (
                        ok
                        and not np.any(np.isnan(rvec))
                        and not np.any(np.isnan(tvec))
                        and not np.any(np.isinf(tvec))
                        and np.linalg.norm(tvec) < 10.0   # sanity: must be < 10 m
                    )
                    if valid:
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                          rvec, tvec, MARKER_SIZE_M * 0.6)
                        R, _ = cv2.Rodrigues(rvec)
                        angles, *_ = cv2.RQDecomp3x3(R)
                        pitch, yaw, roll = angles
                        pose_x = int(np.clip(pts[0][0][0], 0, w_frame - 1))
                        pose_y = int(np.clip(pts[0][0][1] - 34, 14, h_frame - 1))
                        cv2.putText(frame,
                                    f"Y:{yaw:+.1f} P:{pitch:+.1f} R:{roll:+.1f}",
                                    (pose_x, pose_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
                except Exception as e:
                    print(f"[marker {i}] draw error: {type(e).__name__}: {e}")

        # 7. Debug overlay
        h, w = frame.shape[:2]
        n_markers = len(ids) if ids is not None else 0
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"RES: {w}x{h}", (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"TAGS: {n_markers}", (10, 88),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow('GoPro AprilTag Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capture.stop()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
