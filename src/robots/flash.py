#!/usr/bin/env python3
# Batch-deploys this directory's robot firmware over USB serial.
#
# Watches for a Pololu 3pi+ 2040's serial port to appear, unmounts the
# board's USB drive if the host auto-mounted it, copies every *.py file
# in this directory (including main.py — our customized splash loader
# whose default_program chains to uart_controller.py) onto the board via
# mpremote, verifies every on-device copy by size+sha256, soft-resets it
# so the new code starts running, then waits for it to be unplugged
# before watching for the next one.
#
# Requires: pip install mpremote

import hashlib
import subprocess
import sys
import time
from pathlib import Path

import serial.tools.list_ports

ROBOT_DIR = Path(__file__).resolve().parent
SKIP_FILES = {Path(__file__).name}  # glob("*.py") skips dotfiles already; only exclude this script
FILES = sorted(p for p in ROBOT_DIR.glob("*.py") if p.name not in SKIP_FILES)

POLL_INTERVAL_S = 0.5
SETTLE_DELAY_S = 1.0
CONNECT_RETRIES = 3

RED = "\033[1;31m"
GREEN = "\033[32m"
RESET = "\033[0m"

# Runs on the board after cp: report size + sha256 of every deployed file so
# the host can detect silently corrupted/truncated writes — mpremote has been
# seen logging "Input/Output error" mid-cp while flash.py still looked
# successful, leaving truncated files behind (TODO.md URGENT entry).
VERIFY_CODE_TEMPLATE = """\
import os, binascii
try:
    import hashlib
except ImportError:
    hashlib = None
for name in {names}:
    try:
        size = os.stat(name)[6]
    except OSError:
        print('VERIFY', name, -1, '-')
        continue
    digest = '-'
    if hashlib:
        h = hashlib.sha256()
        with open(name, 'rb') as fp:
            chunk = fp.read(512)
            while chunk:
                h.update(chunk)
                chunk = fp.read(512)
        digest = binascii.hexlify(h.digest()).decode()
    print('VERIFY', name, size, digest)
"""


def connected_robot_ports():
    # Matches mpremote's own "auto" device filter: a real USB serial
    # device has a vendor/product id, unlike macOS's virtual Bluetooth
    # serial ports (which show up with vid/pid None).
    return {
        p.device
        for p in serial.tools.list_ports.comports()
        if p.vid is not None and p.pid is not None
    }


def eject_micropython_volume():
    # On macOS the board also mounts as a "MicroPython" disk; force-eject it
    # so it doesn't linger (and nag about improper removal) after unplugging.
    vol = Path("/Volumes/MicroPython")
    if not vol.exists():
        return
    if subprocess.run(["diskutil", "eject", str(vol)]).returncode == 0:
        print("    ejected MicroPython volume")
    else:
        print("    could not eject MicroPython volume")


def _try_unmount_linux(dev, mountpoint):
    for cmd in (["udisksctl", "unmount", "-b", dev], ["umount", mountpoint]):
        try:
            if subprocess.run(cmd).returncode == 0:
                return True
        except FileNotFoundError:
            continue
    return False


def unmount_micropython_volume():
    # The board exposes its flash as a USB drive too. If the host still has it
    # mounted (GNOME auto-mounts it on Linux) while mpremote rewrites the
    # littlefs underneath, the host's cached view and the device diverge —
    # prime suspect for the corrupt/truncated deploys. Unmount BEFORE copying.
    if sys.platform == "darwin":
        eject_micropython_volume()
        return
    try:
        mounts = Path("/proc/mounts").read_text()
    except OSError:
        return
    for line in mounts.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        dev, mountpoint = parts[0], parts[1].replace("\\040", " ")
        if "micropython" not in mountpoint.lower():
            continue
        if _try_unmount_linux(dev, mountpoint):
            print(f"    unmounted {mountpoint}")
        else:
            print(f"    {RED}WARNING: could not unmount {mountpoint} — "
                  f"deploying to a host-mounted drive can corrupt files{RESET}")


def deploy_cmd(port):
    verify_code = VERIFY_CODE_TEMPLATE.format(names=repr([f.name for f in FILES]))
    # "+" separates mpremote sub-commands: without it, cp greedily swallows
    # the following token as a copy destination and fails. The leading
    # soft-reset quiesces a running uart_controller before flash writes
    # (raw REPL suppresses main.py); the trailing one starts the fresh code.
    return ["mpremote", "connect", port,
            "soft-reset", "+",
            "cp", *(str(f) for f in FILES), ":", "+",
            "exec", verify_code, "+",
            "soft-reset"]


def deploy_problems(result):
    """Returns human-readable problems with a deploy attempt; empty = verified OK."""
    problems = []
    output = result.stdout + result.stderr

    if result.returncode != 0:
        problems.append(f"mpremote exited with code {result.returncode}")
    # mpremote can print I/O errors mid-cp and still exit 0 — treat any error
    # line as a failure, don't trust the exit code alone.
    for line in output.splitlines():
        if "error" in line.lower() and not line.startswith("VERIFY "):
            problems.append(f"mpremote: {line.strip()}")

    reported = {}
    for line in output.splitlines():
        if line.startswith("VERIFY "):
            _, name, size, digest = line.split()
            reported[name] = (int(size), digest)

    for f in FILES:
        data = f.read_bytes()
        got = reported.get(f.name)
        if got is None:
            problems.append(f"{f.name}: no verification result from device")
        elif got[0] == -1:
            problems.append(f"{f.name}: missing on device")
        elif got[0] != len(data):
            problems.append(f"{f.name}: size mismatch (device {got[0]}, local {len(data)})")
        elif got[1] != "-" and got[1] != hashlib.sha256(data).hexdigest():
            problems.append(f"{f.name}: content mismatch (corrupt copy)")
    return problems


def flash(port):
    print(f"--> {port}: deploying {', '.join(f.name for f in FILES)}")
    unmount_micropython_volume()
    for attempt in range(1, CONNECT_RETRIES + 1):
        if attempt > 1:
            time.sleep(SETTLE_DELAY_S)
        result = subprocess.run(deploy_cmd(port), capture_output=True, text=True)
        for line in (result.stdout + result.stderr).splitlines():
            if not line.startswith("VERIFY "):
                print(f"    {line}")
        problems = deploy_problems(result)
        if not problems:
            print(f"    {GREEN}done — all {len(FILES)} files verified on device{RESET}")
            eject_micropython_volume()
            return
        print(f"    attempt {attempt}/{CONNECT_RETRIES} failed:")
        for p in problems:
            print(f"      {p}")
    print(f"    {RED}FAILED — files on this robot do NOT match the repo; "
          f"do not trust it until reflashed{RESET}")
    print(f"    {RED}(flash full/corrupt? host re-mounted the USB drive mid-copy? "
          f"see TODO.md URGENT entry){RESET}")


def main():
    print(f"Deploy set: {', '.join(f.name for f in FILES)}")
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
