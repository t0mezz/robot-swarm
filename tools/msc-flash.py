#!/usr/bin/env python3
# Batch-deploys robot firmware over the board's USB mass-storage (MSC)
# interface instead of mpremote's raw-REPL `cp`.
#
# flash.py (mpremote cp + immediate on-device sha256 verify) can report a
# clean deploy while the on-device flash is still wrong after reboot: the
# verify reads the file back through the same live session that just wrote
# it, which can't distinguish "committed to flash" from "still cached in
# that session's own view of the filesystem." Rewriting the exact same
# files through the OS's own USB mass-storage stack instead — and letting
# `diskutil eject`/`udisksctl unmount` force a real flush before the volume
# disappears — fixed files that flash.py had already "verified" as correct.
#
# mpremote is still used here for the soft-resets that quiesce the running
# program (raw REPL suppresses main.py, so nothing on-device is touching
# the filesystem while we mount it) and, critically, for a *post-reboot*
# verify: the corruption this script exists for wasn't visible until the
# board actually rebooted into the new code, so that's the only check that
# actually proves anything.
#
# Requires: pip install mpremote

import shutil
import subprocess
import sys
import time
from pathlib import Path

from flash import (
    CONNECT_RETRIES,
    FILES,
    GREEN,
    POLL_INTERVAL_S,
    RED,
    RESET,
    SETTLE_DELAY_S,
    connected_robot_ports,
    onboard_verify_problems,
    unmount_micropython_volume,
)

MOUNT_WAIT_TIMEOUT_S = 8.0
MOUNT_POLL_S = 0.25


def mounted_volume():
    """Returns the mounted MicroPython volume's path, or None."""
    if sys.platform == "darwin":
        vol = Path("/Volumes/MicroPython")
        return vol if vol.exists() else None
    try:
        mounts = Path("/proc/mounts").read_text()
    except OSError:
        return None
    for line in mounts.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        mountpoint = parts[1].replace("\\040", " ")
        if "micropython" in mountpoint.lower():
            return Path(mountpoint)
    return None


def wait_for_mount(timeout=MOUNT_WAIT_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        vol = mounted_volume()
        if vol is not None:
            return vol
        time.sleep(MOUNT_POLL_S)
    return None


def copy_via_msc(mount):
    """Copies FILES onto the mounted volume. Returns problems (empty = ok).

    This is a sanity check only (catches disk-full/permission failures) —
    it can't prove durability, since a read right after our own write is
    served from the host's page cache, the same blind spot that let
    flash.py's on-device verify miss the original corruption. The real
    check is onboard_verify_problems() after a genuine reboot, below.
    """
    problems = []
    for f in FILES:
        try:
            shutil.copy2(f, mount / f.name)
        except OSError as e:
            problems.append(f"{f.name}: copy failed ({e})")
            continue
        try:
            got = (mount / f.name).stat().st_size
        except OSError:
            problems.append(f"{f.name}: missing on volume after copy")
            continue
        want = f.stat().st_size
        if got != want:
            problems.append(f"{f.name}: size mismatch after copy (volume {got}, local {want})")
    return problems


def flash(port):
    print(f"--> {port}: deploying via MSC: {', '.join(f.name for f in FILES)}")

    for attempt in range(1, CONNECT_RETRIES + 1):
        if attempt > 1:
            time.sleep(SETTLE_DELAY_S)

        # Quiesce the running program first: raw REPL suppresses main.py,
        # so nothing on-device is touching the filesystem while we mount it.
        result = subprocess.run(["mpremote", "connect", port, "soft-reset"],
                                 capture_output=True, text=True)
        if result.returncode != 0:
            print(f"    attempt {attempt}/{CONNECT_RETRIES}: could not soft-reset device: "
                  f"{result.stderr.strip()}")
            continue

        vol = wait_for_mount()
        if vol is None:
            print(f"    attempt {attempt}/{CONNECT_RETRIES}: MicroPython volume never mounted")
            continue

        problems = copy_via_msc(vol)
        unmount_micropython_volume()  # forces the flush the corruption needed
        if problems:
            print(f"    attempt {attempt}/{CONNECT_RETRIES} failed:")
            for p in problems:
                print(f"      {p}")
            continue

        # Reboot into the new code and verify on-device, post-reboot — the
        # check that actually matters (see module docstring above).
        problems = onboard_verify_problems(port)
        if not problems:
            print(f"    {GREEN}done — all {len(FILES)} files verified on device after reboot{RESET}")
            return
        print(f"    attempt {attempt}/{CONNECT_RETRIES} failed post-reboot verify:")
        for p in problems:
            print(f"      {p}")

    print(f"    {RED}FAILED — files on this robot do NOT match the repo; "
          f"do not trust it until reflashed{RESET}")


def main():
    print(f"Deploy set (MSC): {', '.join(f.name for f in FILES)}")
    print("Watching for robots (Ctrl-C to stop)...")
    known = connected_robot_ports()
    while True:
        time.sleep(POLL_INTERVAL_S)
        current = connected_robot_ports()
        new_ports = current - known
        if not new_ports:
            known = current
            continue
        for port in new_ports:
            time.sleep(SETTLE_DELAY_S)
            flash(port)
        known = connected_robot_ports()
        print("Waiting for next robot...")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except FileNotFoundError:
        print("mpremote not found. Install with: pip install mpremote", file=sys.stderr)
        sys.exit(1)
