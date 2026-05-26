#include "unitree_im6014/im6014_motor.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

std::atomic<bool> keep_running{true};
void signal_handler(int signum) {
  if (signum == SIGINT) {
    keep_running.store(false);
  }
}

int main() {
  unitree::IM6014::Motor motor;
  // 请替换为实际串口号，波特率支持 4000000 或 6000000
  if (!motor.init("/dev/ttyUSB0", 6000000)) {
    std::cerr << "Failed to open serial port!" << std::endl;
    return -1;
  }

  std::signal(SIGINT, signal_handler);

  uint8_t id1 = 0, id2 = 1;
  // PD gains: Kp [Nm/rad], Kd [Nm/(rad/s)]
  float kp = 0, kd = 2.5;

  std::cout << "=== IM6014 Dual Motor Example (Output-end units) ==="
            << std::endl;

  unitree::IM6014::State s1, s2;
  float t = 0.0;
  while (keep_running.load()) {
    motor.send_cmd(id1, 0.0, -0.5 * (1 - std::cos(3 * t)), 0, kp, kd, 1,
                   0); // motor 0
    motor.send_cmd(id2, 0.0, 1.5 * (1 - std::cos(2 * t)), 0, kp, kd, 1,
                   0); // motor 1

    motor.recv_state(id1, s1, 0);
    motor.recv_state(id2, s2, 0);

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

  // Stop motors
  std::cout << "\nCaught Ctrl+C, stopping motors..." << std::endl;
  motor.send_cmd(id1, 0, 0, 0, 0, 0, 0, 1);
  motor.send_cmd(id2, 0, 0, 0, 0, 0, 0, 1);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  motor.close();
  std::cout << "Motors stopped." << std::endl;
  return 0;
}