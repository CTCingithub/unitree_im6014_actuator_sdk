#!/usr/bin/env python3
import pyim6014
import time


def main():
    motor = pyim6014.IM6014Motor()
    if not motor.init("/dev/ttyUSB0", 4000000):
        print("Failed to open serial port!")
        return

    kp, kd = 20.0, 1.0  # 输出端单位: Nm/rad, Nm/(rad/s)

    print("=== IM6014 Python Example (Output-end units) ===")
    print("Motor 1: +3.0 rad, Motor 2: -1.5 rad")

    for i in range(150):
        # 直接使用输出端单位
        motor.send_cmd(1, 0.0, 0.0, 3.0, kp, kd)  # 电机1 → +3.0 rad
        motor.send_cmd(2, 0.0, 0.0, -1.5, kp, kd)  # 电机2 → -1.5 rad

        s1, s2 = pyim6014.IM6014State(), pyim6014.IM6014State()
        motor.recv_state(1, s1)
        motor.recv_state(2, s2)

        if s1.valid:
            print(
                f"[M1] pos={s1.pos:6.3f} rad, spd={s1.speed:6.3f} rad/s, tor={s1.torque:6.3f} Nm"
            )
        if s2.valid:
            print(
                f"[M2] pos={s2.pos:6.3f} rad, spd={s2.speed:6.3f} rad/s, tor={s2.torque:6.3f} Nm"
            )

        time.sleep(0.01)

    # 停机
    motor.send_cmd(1, 0, 0, 0, 0, 0, 0, 1)
    motor.send_cmd(2, 0, 0, 0, 0, 0, 0, 1)
    motor.close()
    print("Done.")


if __name__ == "__main__":
    main()
