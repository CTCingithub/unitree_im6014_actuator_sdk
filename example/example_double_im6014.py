#!/usr/bin/env python3
"""Dual IM6014 motor velocity-control example — matches example_double_im6014.cpp."""

import math
import os
import signal
import sys
import time

try:
    from unitree_im6014 import Motor, State
except ModuleNotFoundError:
    _project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, os.path.join(_project_root, "python"))
    from unitree_im6014 import Motor, State  # type: ignore[no-redef]

keep_running = True


def _on_sigint(_signum, _frame):
    global keep_running
    keep_running = False


def main():
    global keep_running
    motor = Motor()
    if not motor.init("/dev/ttyUSB0", 6000000):
        print("Failed to open serial port!", file=sys.stderr)
        return 1

    signal.signal(signal.SIGINT, _on_sigint)

    id1, id2 = 0, 1
    kp, kd = 0.0, 2.5

    print("=== IM6014 Dual Motor Example (Output-end units) ===")

    s1, s2 = State(), State()
    t = 0.0
    while keep_running:
        # Velocity control with sinusoidal trajectory (output-end units)
        motor.send_cmd(id1, 0.0, -0.5 * (1 - math.cos(3 * t)), 0, kp, kd)
        motor.send_cmd(id2, 0.0, 1.5 * (1 - math.cos(2 * t)), 0, kp, kd)

        motor.poll_states(timeout_ms=2)
        ok1 = motor.get_state(id1, s1)
        ok2 = motor.get_state(id2, s2)

        if ok1 and ok2:
            sys.stdout.write("\033c")
            sys.stdout.write(
                f"[M1] Pos: {s1.pos:.3f} rad, "
                f"Spd: {s1.speed:.3f} rad/s, "
                f"Tor: {s1.torque:.3f} Nm\n"
                f"[M2] Pos: {s2.pos:.3f} rad, "
                f"Spd: {s2.speed:.3f} rad/s, "
                f"Tor: {s2.torque:.3f} Nm\n"
            )
            sys.stdout.flush()

        time.sleep(0.002)
        t += 0.002

    # Graceful stop
    print("\nCaught Ctrl+C, stopping motors...")
    motor.send_cmd(id1, 0, 0, 0, 0, 0, status=0)
    motor.send_cmd(id2, 0, 0, 0, 0, 0, status=0)
    time.sleep(1.0)

    motor.close()
    print("Motors stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
