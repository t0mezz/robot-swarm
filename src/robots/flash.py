#!/usr/bin/env python3
# Batch-deploys this directory's robot firmware over USB serial.
#
# Watches for a Pololu 3pi+ 2040's serial port to appear, copies every
# *.py file in this directory (except main.py, which is Pololu's stock
# splash loader and isn't ours to overwrite) onto the board via mpremote,
# soft-resets it so the new code starts running, then waits for it to be
# unplugged before watching for the next one.
#
# Requires: pip install mpremote

import subprocess
import sys
import time
from pathlib import Path

import serial.tools.list_ports

ROBOT_DIR = Path(__file__).resolve().parent
SKIP_FILES = {".main.py", Path(__file__).name}
FILES = sorted(p for p in ROBOT_DIR.glob("*.py") if p.name not in SKIP_FILES)

POLL_INTERVAL_S = 0.5
SETTLE_DELAY_S = 1.0
CONNECT_RETRIES = 3


def connected_robot_ports():
    # Matches mpremote's own "auto" device filter: a real USB serial
    # device has a vendor/product id, unlike macOS's virtual Bluetooth
    # serial ports (which show up with vid/pid None).
    return {
        p.device
        for p in serial.tools.list_ports.comports()
        if p.vid is not None and p.pid is not None
    }


def flash(port):
    print(f"--> {port}: deploying {', '.join(f.name for f in FILES)}")
    # "+" separates mpremote sub-commands: without it, cp greedily swallows
    # the following "soft-reset" token as a copy destination and fails.
    cmd = ["mpremote", "connect", port, "cp", *(str(f) for f in FILES), ":", "+", "soft-reset"]
    for attempt in range(1, CONNECT_RETRIES + 1):
        if subprocess.run(cmd).returncode == 0:
            print("    done")
            return
        if attempt < CONNECT_RETRIES:
            time.sleep(SETTLE_DELAY_S)
    print("    FAILED -- is this actually a robot in MicroPython mode?")


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
