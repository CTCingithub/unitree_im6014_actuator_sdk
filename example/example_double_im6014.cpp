#include "unitree_im6014/im6014_motor.hpp"
#include <chrono>
#include <iostream>
#include <thread>

using namespace unitree::IM6014;

int main() {
  IM6014Motor motor;
  // 请替换为实际串口号，波特率支持 4000000 或 6000000
  if (!motor.init("/dev/ttyUSB0", 4000000)) {
    std::cerr << "Failed to open serial port!" << std::endl;
    return -1;
  }

  uint8_t id1 = 1, id2 = 2;
  // PD 参数 (输出端单位): Kp [Nm/rad], Kd [Nm/(rad/s)]
  float kp = 0, kd = 0.25;

  std::cout << "=== IM6014 Dual Motor Example (Output-end units) ==="
            << std::endl;

  IM6014State s1, s2;
  float t = 0.0;
  while (true) {
    // 直接使用输出端单位: Nm, rad/s, rad
    motor.send_cmd(id1, 0.0, -0.2 * (1 - std::cos(0.5 * t)), 0, kp, kd, 1,
                   1); // 电机1
    motor.send_cmd(id2, 0.0, 0.1 * (1 - std::cos(0.5 * t)), 0, kp, kd, 1,
                   1); // 电机2

    motor.recv_state(id1, s1, 2);
    motor.recv_state(id2, s2, 2);

    if (s1.valid && s2.valid) {
      std::cout << "\033c" << std::flush;
      std::cout << "[M1] Pos: " << s1.pos << " rad, "
                << "Spd: " << s1.speed << " rad/s, "
                << "Tor: " << s1.torque << " Nm" << std::endl
                << "[M2] Pos: " << s2.pos << " rad, "
                << "Spd: " << s2.speed << " rad/s, "
                << "Tor: " << s2.torque << " Nm" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    t += 0.002;
  }

  // 停机 (status=0)
  motor.send_cmd(id1, 0, 0, 0, 0, 0, 0, 1);
  motor.send_cmd(id2, 0, 0, 0, 0, 0, 0, 1);
  motor.close();
  std::cout << "Done." << std::endl;
  return 0;
}